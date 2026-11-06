/* Exercise the production worker and synchronous request ABI with real threads. */
#include "core.h"
#include "ssf_mems_iio.h"
#include <linux/kthread.h>
#include <linux/completion.h>
#include <assert.h>
#include <time.h>
#include <unistd.h>

unsigned long jiffies;
unsigned int test_wake_count;
static struct ssf_mems_data device;
static struct serdev_device serial;
static struct task_struct worker;
static pthread_mutex_t gate = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t changed = PTHREAD_COND_INITIALIZER;
static atomic_bool stop_worker, interrupt_wait, interrupt_observed;
static bool block_raw, raw_entered, release_raw;
static bool block_sound, sound_entered, release_sound;
static atomic_bool sound_writer_finished;
static unsigned int raw_count, sound_count, trace_count;
static unsigned int sound_duration;
static enum ssf_mems_acquisition_route trace[32];
static int raw_result, sound_result;
static bool reset_flags[32];

static void deadline(struct timespec *until, unsigned int ms) {
  clock_gettime(CLOCK_REALTIME, until);
  until->tv_nsec += ms * 1000000UL;
  until->tv_sec += until->tv_nsec / 1000000000UL;
  until->tv_nsec %= 1000000000UL;
}
static void wait_for_raw(void) {
  struct timespec until;
  deadline(&until, 2000);
  pthread_mutex_lock(&gate);
  while (!raw_entered) assert(pthread_cond_timedwait(&changed, &gate, &until) == 0);
  pthread_mutex_unlock(&gate);
}
static void unblock_raw(void) {
  pthread_mutex_lock(&gate);
  release_raw = true;
  pthread_cond_broadcast(&changed);
  pthread_mutex_unlock(&gate);
}
void test_completion_notify(void) {
  pthread_mutex_lock(&gate);
  pthread_cond_broadcast(&changed);
  pthread_mutex_unlock(&gate);
}
int test_completion_wait(struct completion *done, bool interruptible) {
  struct timespec until;
  if (interruptible && atomic_exchange(&interrupt_wait, false)) {
    atomic_store(&interrupt_observed, true);
    return -ERESTARTSYS;
  }
  deadline(&until, 3000);
  pthread_mutex_lock(&gate);
  while (!atomic_load(&done->done)) {
    if (interruptible && atomic_exchange(&interrupt_wait, false)) {
      atomic_store(&interrupt_observed, true);
      pthread_mutex_unlock(&gate);
      return -ERESTARTSYS;
    }
    assert(pthread_cond_timedwait(&changed, &gate, &until) == 0);
  }
  pthread_mutex_unlock(&gate);
  return 0;
}
void test_acquisition_wait(unsigned long timeout) {
  struct timespec until;
  deadline(&until, 1);
  pthread_mutex_lock(&gate);
  (void)pthread_cond_timedwait(&changed, &gate, &until);
  pthread_mutex_unlock(&gate);
  jiffies += timeout;
}
static void *run_worker(void *context) { (void)context; worker.fn(worker.context); return NULL; }
struct task_struct *test_kthread_run(int (*fn)(void *), void *context, const char *name, ...) {
  (void)name;
  worker.fn = fn;
  worker.context = context;
  assert(pthread_create(&worker.native, NULL, run_worker, NULL) == 0);
  return &worker;
}
bool kthread_should_stop(void) { return atomic_load(&stop_worker); }
int kthread_stop(struct task_struct *task) {
  assert(task == &worker);
  atomic_store(&stop_worker, true);
  test_completion_notify();
  assert(pthread_join(worker.native, NULL) == 0);
  return 0;
}
int ssf_mems_baudrate_to_value(const struct ssf_mems_sensor *model, enum ssf_mems_baudrate baud) {
  (void)model; (void)baud; return 128000;
}
int ssf_mems_protocol_get_baudrate(struct serdev_device *dev, enum ssf_mems_baudrate *baud, unsigned int timeout) {
  (void)dev; (void)timeout; *baud = SSF_MEMS_BAUDRATE_128000; return 0;
}
int ssf_mems_protocol_set_baudrate(struct serdev_device *dev, enum ssf_mems_baudrate baud, unsigned int timeout) {
  (void)dev; (void)baud; (void)timeout; return 0;
}
int ssf_mems_protocol_write_setting(struct serdev_device *dev, enum ssf_mems_sensor_setting setting,
                                   u16 value, unsigned int timeout) {
  (void)dev; (void)setting; (void)value; (void)timeout; return 0;
}
int ssf_mems_protocol_recover_capture(struct ssf_mems_data *data) {
  assert(data == &device && data->acquisition.stopping);
  return 0;
}
void ssf_mems_protocol_invalidate_features(struct ssf_mems_data *data) { data->protocol.valid = false; }
int ssf_mems_protocol_get_features(struct serdev_device *dev, struct ssf_mems_sensor_data *sample) {
  (void)dev; *sample = device.protocol.features; return 0;
}
int ssf_mems_iio_publish_features(struct ssf_mems_data *data,
    const struct ssf_mems_sensor_data *sample, s64 stamp) {
  (void)data; (void)sample; (void)stamp; return 0;
}
bool ssf_mems_iio_raw_enabled(struct ssf_mems_data *data) { return data == &device; }
bool ssf_mems_iio_sound_enabled(struct ssf_mems_data *data) { return data == &device; }
static int collect_feature(struct ssf_mems_data *data, unsigned int timeout) {
  (void)timeout; data->protocol.valid = true; jiffies += 5; return 0;
}
static int collect_raw(struct ssf_mems_data *data, u32 samples, bool reset) {
  assert(pthread_equal(pthread_self(), worker.native));
  assert(data == &device && samples == 1024);
  pthread_mutex_lock(&gate);
  assert(trace_count < ARRAY_SIZE(trace) && raw_count < ARRAY_SIZE(reset_flags));
  trace[trace_count++] = SSF_MEMS_ACQ_ROUTE_RAW;
  reset_flags[raw_count++] = reset;
  if (block_raw && raw_count == 1) {
    raw_entered = true;
    pthread_cond_broadcast(&changed);
    while (!release_raw) pthread_cond_wait(&changed, &gate);
  }
  pthread_mutex_unlock(&gate);
  jiffies += 100;
  return READ_ONCE(data->acquisition.stopping) ? -ENODEV : 0;
}
static int collect_sound(struct ssf_mems_data *data) {
  assert(pthread_equal(pthread_self(), worker.native));
  pthread_mutex_lock(&gate);
  assert(trace_count < ARRAY_SIZE(trace));
  trace[trace_count++] = SSF_MEMS_ACQ_ROUTE_SOUND;
  sound_count++;
  sound_entered = true;
  pthread_cond_broadcast(&changed);
  while (block_sound && !release_sound) pthread_cond_wait(&changed, &gate);
  pthread_mutex_unlock(&gate);
  jiffies += sound_duration;
  (void)data;
  return 0;
}
static const struct ssf_mems_sensor_ops ops = {
  .read_features = collect_feature, .collect_raw = collect_raw, .collect_sound = collect_sound,
};
static const struct ssf_mems_sensor model = {
  .ops = &ops, .profile = { .default_work_baudrate = SSF_MEMS_BAUDRATE_128000,
      .default_sampling_rate_index = 6, .raw_batch_max_samples = 32768 },
};
static void *raw_writer(void *context) { (void)context; raw_result = ssf_mems_acquisition_capture_window(&device); return NULL; }
static void *sound_writer(void *context) { (void)context; sound_result = ssf_mems_acquisition_capture_sound(&device); atomic_store(&sound_writer_finished, true); return NULL; }
static struct ssf_mems_acquisition_status status(void) {
  struct ssf_mems_acquisition_status snapshot;
  ssf_mems_acquisition_get_status(&device, &snapshot);
  return snapshot;
}
static void wait_online(void) {
  for (unsigned int i = 0; i < 2000; i++) {
    if (status().online) return;
    usleep(1000);
  }
  abort();
}
static void wait_queued(bool interrupted) {
  for (unsigned int i = 0; i < 2000; i++) {
    if (status().manual_pending && (!interrupted || atomic_load(&interrupt_observed))) return;
    usleep(1000);
  }
  abort();
}
static void wait_slot(enum ssf_mems_acquisition_route route) {
  for (unsigned int i = 0; ; i++) {
    bool queued;
    mutex_lock(&device.acquisition.status_lock);
    queued = device.acquisition.requests[route] != NULL;
    mutex_unlock(&device.acquisition.status_lock);
    if (queued) return;
    assert(i < 2000); usleep(1000);
  }
}
static void setup(bool manual) {
  memset(&device, 0, sizeof(device));
  device.serdev = &serial; device.sensor = &model; device.sensor_ops = &ops;
  device.indio_dev = (void *)&serial; device.protocol.host_baudrate = 128000;
  assert(ssf_mems_acquisition_init(&device) == 0);
  assert(ssf_mems_acquisition_set_mode(&device, SSF_MEMS_ACQ_MODE_RAW) == 0);
  assert(ssf_mems_acquisition_set_raw_manual(&device, manual) == 0);
  assert(ssf_mems_acquisition_set_duration(&device, SSF_MEMS_ACQ_ROUTE_RAW, 450) == 0);
  atomic_store(&stop_worker, false); atomic_store(&interrupt_wait, false);
  atomic_store(&interrupt_observed, false);
  block_raw = true; raw_entered = release_raw = false;
  block_sound = sound_entered = release_sound = false;
  atomic_store(&sound_writer_finished, false);
  raw_count = sound_count = trace_count = 0; sound_duration = 80; jiffies = 0;
  assert(ssf_mems_acquisition_start(&device) == 0);
  wait_online();
}
static void teardown(void) {
  ssf_mems_acquisition_stop(&device);
  assert(!status().manual_pending && !status().busy);
  pthread_mutex_destroy(&device.acquisition.lock.native);
  pthread_mutex_destroy(&device.acquisition.status_lock.native);
}
int main(void) {
  pthread_t raw, sound;
  struct ssf_mems_acquisition_status before, pending, after;
  /* Sound must run between batches of a synchronous raw window. */
  setup(true);
  assert(pthread_create(&raw, NULL, raw_writer, NULL) == 0);
  wait_for_raw();
  before = status();
  assert(ssf_mems_acquisition_manual_pending(&device));
  assert(!ssf_mems_acquisition_manual_pending(NULL));
  assert(before.busy && before.manual_active && before.route == SSF_MEMS_ACQ_ROUTE_RAW);
  assert(ssf_mems_acquisition_capture_window(&device) == -EBUSY);
  assert(ssf_mems_acquisition_set_mode(&device, SSF_MEMS_ACQ_MODE_FEATURE) == 0);
  assert(ssf_mems_acquisition_set_duration(&device, SSF_MEMS_ACQ_ROUTE_RAW, 100) == 0);
  pending = status();
  assert(pending.requested_mode == SSF_MEMS_ACQ_MODE_FEATURE);
  assert(pending.applied_mode == SSF_MEMS_ACQ_MODE_RAW);
  assert(pending.config_version > pending.applied_version);
  assert(pending.applied_version == before.applied_version);
  assert(pthread_create(&sound, NULL, sound_writer, NULL) == 0);
  wait_slot(SSF_MEMS_ACQ_ROUTE_SOUND);
  unblock_raw();
  pthread_join(raw, NULL); pthread_join(sound, NULL);
  assert(raw_result == 0 && sound_result == 0);
  assert(trace_count >= 4 && trace[0] == SSF_MEMS_ACQ_ROUTE_RAW);
  assert(trace[1] == SSF_MEMS_ACQ_ROUTE_SOUND && trace[2] == SSF_MEMS_ACQ_ROUTE_RAW);
  assert(raw_count >= 3 && sound_count == 1);
  assert(reset_flags[0]);
  for (unsigned int i = 1; i < raw_count; i++) assert(!reset_flags[i]);
  for (unsigned int i = 0; ; i++) {
    after = status();
    if (after.applied_version == after.config_version) break;
    assert(i < 2000); usleep(1000);
  }
  assert(after.applied_mode == SSF_MEMS_ACQ_MODE_FEATURE);
  teardown();

  /* A long inserted sound consumes the raw window, without starting an extra raw batch. */
  setup(true); sound_duration = 500;
  assert(pthread_create(&raw, NULL, raw_writer, NULL) == 0); wait_for_raw();
  assert(pthread_create(&sound, NULL, sound_writer, NULL) == 0); wait_slot(SSF_MEMS_ACQ_ROUTE_SOUND);
  unblock_raw(); pthread_join(raw, NULL); pthread_join(sound, NULL);
  assert(raw_result == 0 && sound_result == 0 && raw_count == 1 && sound_count == 1);
  teardown();

  /* A signal cancels a queued sound without issuing the trigger. */
  setup(false); wait_for_raw();
  atomic_store(&interrupt_wait, true);
  assert(pthread_create(&sound, NULL, sound_writer, NULL) == 0);
  wait_queued(true); unblock_raw(); pthread_join(sound, NULL);
  assert(sound_result == -EINTR && sound_count == 0);
  teardown();

  /* Cancelling an executing sound must wait for its safe completion. */
  setup(true);
  pthread_mutex_lock(&gate); block_sound = true; pthread_mutex_unlock(&gate);
  assert(pthread_create(&sound, NULL, sound_writer, NULL) == 0);
  pthread_mutex_lock(&gate);
  while (!sound_entered) pthread_cond_wait(&changed, &gate);
  pthread_mutex_unlock(&gate);
  assert(status().busy && status().manual_active && status().route == SSF_MEMS_ACQ_ROUTE_SOUND);
  atomic_store(&interrupt_wait, true); test_completion_notify();
  for (unsigned int i = 0; !atomic_load(&interrupt_observed); i++) {
    assert(i < 2000); usleep(1000);
  }
  assert(!atomic_load(&sound_writer_finished));
  pthread_mutex_lock(&gate); release_sound = true; pthread_cond_broadcast(&changed); pthread_mutex_unlock(&gate);
  pthread_join(sound, NULL);
  assert(sound_result == -EINTR && sound_count == 1);
  teardown();

  /* Removal completes the blocked writer, without leaving its stack queued. */
  setup(true);
  assert(pthread_create(&raw, NULL, raw_writer, NULL) == 0); wait_for_raw();
  assert(pthread_create(&sound, NULL, sound_writer, NULL) == 0); wait_slot(SSF_MEMS_ACQ_ROUTE_SOUND);
  WRITE_ONCE(device.acquisition.stopping, true);
  unblock_raw(); ssf_mems_acquisition_stop(&device);
  pthread_join(raw, NULL); pthread_join(sound, NULL);
  teardown();
  assert(raw_result == -ENODEV && sound_result == -ENODEV);
  puts("PASS real worker arbitration, insertion, configuration versions, cancellation and shutdown");
  return 0;
}

int ssf_mems_iio_begin_raw(struct ssf_mems_data *data) {
  return ssf_mems_iio_raw_enabled(data) ? 0 : -ENODATA;
}
int ssf_mems_iio_begin_sound(struct ssf_mems_data *data) {
  return ssf_mems_iio_sound_enabled(data) ? 0 : -ENODATA;
}
