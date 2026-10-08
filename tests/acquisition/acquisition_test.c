#include "ssf_mems_acquisition.h"
#include "core.h"
#include "ssf_mems_iio.h"
#include <linux/err.h>
#include <linux/jiffies.h>
#include <linux/kthread.h>
#include <limits.h>

static unsigned int checks;
#define CHECK(condition) do { checks++; if (!(condition)) { \
  fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #condition); abort(); \
} } while (0)

unsigned long jiffies;
unsigned int test_wake_count;
static struct ssf_mems_xyzs_data device;
static struct serdev_device serial;
static struct task_struct task;
static int (*thread_function)(void *);
static void *thread_context;
static bool stop_requested, fail_thread_start;
static unsigned int get_count, set_count, read_count, publish_count;
static unsigned int failure_count, wait_count, backoff_count;
static unsigned int backoffs[10];
static enum ssf_mems_baudrate sensor_baudrate;
static int get_error, set_error, publish_error;
static bool fail_reads;
static struct ssf_mems_sensor_data published;
enum scenario {
  CHANGING, CONSTANT, LINK_FAILURE, EXCEPTION, NO_SENSOR, MANUAL,
  STOP_STARTUP, STOP_VERIFY, STOP_READ
};
static enum scenario scenario;

struct task_struct *test_kthread_run(int (*fn)(void *), void *data,
                                    const char *name, ...) {
  (void)name;
  if (fail_thread_start)
    return (void *)(intptr_t)-ENOMEM;
  thread_function = fn;
  thread_context = data;
  return &task;
}

bool kthread_should_stop(void) { return stop_requested; }
int kthread_stop(struct task_struct *thread) {
  CHECK(thread == &task);
  stop_requested = true;
  return 0;
}

int ssf_mems_baudrate_to_value(enum ssf_mems_baudrate baudrate) {
  static const int rates[] = {9600, 2400, 4800, 9600, 19200, 38400,
                             57600, 115200};
  CHECK((unsigned int)baudrate < sizeof(rates) / sizeof(rates[0]));
  return rates[baudrate];
}

int ssf_mems_protocol_get_baudrate(struct serdev_device *s,
                                  enum ssf_mems_baudrate *baudrate,
                                  unsigned int timeout_ms) {
  CHECK(s == &serial && timeout_ms == 200);
  get_count++;
  if (get_error) {
    jiffies += timeout_ms;
    return get_error;
  }
  *baudrate = sensor_baudrate;
  device.protocol.baudrate = sensor_baudrate;
  device.protocol.host_baudrate = ssf_mems_baudrate_to_value(sensor_baudrate);
  return 0;
}

int ssf_mems_protocol_set_baudrate(struct serdev_device *s,
                                  enum ssf_mems_baudrate baudrate,
                                  unsigned int timeout_ms) {
  CHECK(s == &serial && timeout_ms == 200);
  set_count++;
  /* 写请求必须在旧速率完成；模拟丢应答时传感器已切换而主机仍在旧速率。 */
  CHECK(device.protocol.host_baudrate ==
        (unsigned int)ssf_mems_baudrate_to_value(sensor_baudrate));
  sensor_baudrate = baudrate;
  if (set_error)
    return set_error;
  device.protocol.baudrate = baudrate;
  device.protocol.host_baudrate = ssf_mems_baudrate_to_value(baudrate);
  return 0;
}

void ssf_mems_protocol_invalidate_features(struct ssf_mems_xyzs_data *data) {
  data->protocol.valid = false;
}

int ssf_mems_protocol_read_features(struct serdev_device *s,
                                   unsigned int timeout_ms) {
  CHECK(s == &serial);
  CHECK(device.protocol.host_baudrate ==
        (unsigned int)ssf_mems_baudrate_to_value(sensor_baudrate));
  CHECK(timeout_ms > 200);
  if (sensor_baudrate == SSF_MEMS_BAUDRATE_2400)
    CHECK(timeout_ms > 1000);
  read_count++;
  if (scenario == STOP_READ)
    stop_requested = true;
  if (fail_reads && read_count > 1) {
    failure_count++;
    jiffies += timeout_ms;
    return scenario == EXCEPTION ? -EREMOTEIO : -ETIMEDOUT;
  }
  jiffies += 5;
  memset(&device.protocol.features, 0, sizeof(device.protocol.features));
  if (scenario != CONSTANT)
    device.protocol.features.x.acc_rms_x100 = read_count;
  device.protocol.valid = true;
  return 0;
}

int ssf_mems_protocol_get_features(struct serdev_device *s,
                                  struct ssf_mems_sensor_data *result) {
  CHECK(s == &serial);
  if (!device.protocol.valid)
    return -ENODATA;
  *result = device.protocol.features;
  return 0;
}

int ssf_mems_iio_publish_features(struct ssf_mems_xyzs_data *data,
                                 const struct ssf_mems_sensor_data *features,
                                 s64 timestamp_ns) {
  CHECK(data == &device && timestamp_ns == 0);
  CHECK(data->indio_dev != NULL && data->protocol.valid);
  published = *features;
  publish_count++;
  return publish_error;
}

void test_acquisition_wait(unsigned long timeout) {
  CHECK(++wait_count < 300);
  if (wait_count == 1) {
    CHECK(timeout == 1000 && read_count == 0 && get_count == 0);
    if (scenario == STOP_STARTUP)
      stop_requested = true;
  } else if (scenario == STOP_VERIFY &&
             device.acquisition.phase == SSF_MEMS_ACQ_VERIFY) {
    stop_requested = true;
  } else if (device.acquisition.phase == SSF_MEMS_ACQ_BACKOFF) {
    CHECK(!device.acquisition.online && !device.protocol.valid);
    CHECK(backoff_count < sizeof(backoffs) / sizeof(backoffs[0]));
    backoffs[backoff_count++] = timeout;
    if (scenario == NO_SENSOR) {
      if (backoff_count == 7)
        stop_requested = true;
    } else {
      CHECK(scenario == LINK_FAILURE && failure_count >= 5);
      CHECK(publish_count == 1);
      /* 模拟传感器掉电，启动后回到默认速率。 */
      sensor_baudrate = SSF_MEMS_BAUDRATE_9600;
      fail_reads = false;
    }
  } else if (device.acquisition.phase == SSF_MEMS_ACQ_POLL) {
    CHECK(timeout >= 2);
    if (scenario == CHANGING || scenario == CONSTANT) {
      if (publish_count >= 70)
        stop_requested = true;
    } else if (scenario == LINK_FAILURE && publish_count == 2) {
      CHECK(get_count == 2 && set_count == 2);
      CHECK(device.acquisition.policy.interval_ms == 100);
      stop_requested = true;
    } else if (scenario == EXCEPTION && failure_count >= 20) {
      CHECK(get_count == 1 && set_count == 1);
      CHECK(device.acquisition.online && !device.protocol.valid);
      stop_requested = true;
    } else if (scenario == MANUAL && publish_count == 1) {
      stop_requested = true;
    }
  }
  jiffies += timeout;
}

static void reset_device(enum scenario next) {
  memset(&device, 0, sizeof(device));
  memset(&serial, 0, sizeof(serial));
  device.serdev = &serial;
  device.indio_dev = (void *)&task;
  device.protocol.host_baudrate = 9600;
  device.protocol.baudrate = SSF_MEMS_BAUDRATE_9600;
  sensor_baudrate = SSF_MEMS_BAUDRATE_9600;
  jiffies = 0;
  get_count = set_count = read_count = publish_count = failure_count = 0;
  wait_count = backoff_count = 0;
  get_error = set_error = publish_error = 0;
  fail_reads = fail_thread_start = stop_requested = false;
  scenario = next;
  CHECK(ssf_mems_acquisition_init(&device) == 0);
}

static void run_thread(void) {
  CHECK(ssf_mems_acquisition_start(&device) == 0);
  CHECK(ssf_mems_acquisition_start(&device) == -EBUSY);
  CHECK(thread_function(thread_context) == 0);
  CHECK(device.acquisition.phase == SSF_MEMS_ACQ_STOPPED);
  CHECK(!device.acquisition.online && !device.protocol.valid);
  ssf_mems_acquisition_stop(&device);
  CHECK(device.acquisition.thread == NULL && device.acquisition.stopping);
  CHECK(ssf_mems_acquisition_start(&device) == -ENODEV);
  ssf_mems_acquisition_stop(&device);
}

static void test_thread_scenarios(void) {
  struct ssf_mems_acquisition_status status;
  enum ssf_mems_baudrate baudrate;

  reset_device(CHANGING);
  ssf_mems_acquisition_get_status(&device, &status);
  CHECK(!status.online && !status.have_sample && status.interval_ms == 100);
  run_thread();
  CHECK(get_count == 1 && set_count == 1 && publish_count == 70);
  CHECK(device.acquisition.policy.interval_ms < 100);
  CHECK(published.x.acc_rms_x100 == read_count);
  ssf_mems_acquisition_get_status(&device, &status);
  CHECK(status.have_sample && !status.online && status.sample_age_ms > 0);
  CHECK(ssf_mems_acquisition_set_baudrate(&device, SSF_MEMS_BAUDRATE_57600) == -ENODEV);
  CHECK(ssf_mems_acquisition_get_baudrate(&device, &baudrate) == -ENODEV);

  reset_device(CONSTANT);
  run_thread();
  CHECK(device.acquisition.policy.interval_ms > 100);

  reset_device(LINK_FAILURE);
  fail_reads = true;
  run_thread();
  CHECK(backoff_count == 1 && backoffs[0] == 1000 && publish_count == 2);

  reset_device(EXCEPTION);
  fail_reads = true;
  run_thread();
  CHECK(backoff_count == 0 && publish_count == 1);

  reset_device(NO_SENSOR);
  get_error = -ETIMEDOUT;
  run_thread();
  CHECK(backoff_count == 7 && read_count == 0 && publish_count == 0);
  CHECK(backoffs[0] == 1000 && backoffs[1] == 2000 && backoffs[2] == 4000);
  CHECK(backoffs[3] == 8000 && backoffs[4] == 16000);
  CHECK(backoffs[5] == 30000 && backoffs[6] == 30000);

  reset_device(MANUAL);
  CHECK(ssf_mems_acquisition_set_baudrate(&device, SSF_MEMS_BAUDRATE_57600) == 0);
  CHECK(device.acquisition.target_baudrate == SSF_MEMS_BAUDRATE_57600);
  CHECK(device.acquisition.reconnect);
  CHECK(ssf_mems_acquisition_get_baudrate(&device, &baudrate) == 0);
  CHECK(baudrate == SSF_MEMS_BAUDRATE_57600);
  /* 启动等待断言只统计线程自身的操作。 */
  get_count = set_count = 0;
  run_thread();
  CHECK(get_count == 1 && set_count == 0);
  CHECK(sensor_baudrate == SSF_MEMS_BAUDRATE_57600);

  reset_device(MANUAL);
  set_error = -ETIMEDOUT;
  CHECK(ssf_mems_acquisition_set_baudrate(&device, SSF_MEMS_BAUDRATE_57600) == -ETIMEDOUT);
  CHECK(device.acquisition.target_baudrate == SSF_MEMS_BAUDRATE_115200);
  CHECK(device.protocol.host_baudrate == 9600 && device.acquisition.reconnect);
  set_error = 0;
  get_count = set_count = 0;
  run_thread();
  CHECK(sensor_baudrate == SSF_MEMS_BAUDRATE_115200 && set_count == 1);

  reset_device(MANUAL);
  sensor_baudrate = SSF_MEMS_BAUDRATE_115200;
  device.protocol.host_baudrate = 115200;
  device.protocol.baudrate = sensor_baudrate;
  publish_error = -EIO;
  run_thread();
  CHECK(set_count == 0 && publish_count == 1 && backoff_count == 0);

  reset_device(MANUAL);
  CHECK(ssf_mems_acquisition_set_baudrate(&device, SSF_MEMS_BAUDRATE_2400) == 0);
  get_count = set_count = 0;
  run_thread();
  CHECK(sensor_baudrate == SSF_MEMS_BAUDRATE_2400);

  reset_device(MANUAL);
  fail_thread_start = true;
  CHECK(ssf_mems_acquisition_start(&device) == -ENOMEM);
  CHECK(device.acquisition.thread == NULL);
  ssf_mems_acquisition_stop(&device);

  reset_device(STOP_STARTUP);
  run_thread();
  CHECK(get_count == 0 && set_count == 0 && read_count == 0);
  reset_device(STOP_VERIFY);
  run_thread();
  CHECK(get_count == 1 && set_count == 1 && publish_count == 0);
  reset_device(STOP_READ);
  run_thread();
  CHECK(read_count == 1 && publish_count == 0);

  reset_device(MANUAL);
  CHECK(ssf_mems_acquisition_get_baudrate(&device, &baudrate) == 0);
  CHECK(baudrate == SSF_MEMS_BAUDRATE_9600 && device.acquisition.reconnect);
  get_error = -ETIMEDOUT;
  CHECK(ssf_mems_acquisition_get_baudrate(&device, &baudrate) == -ETIMEDOUT);
  CHECK(!device.acquisition.online && !device.protocol.valid);
  CHECK(ssf_mems_acquisition_get_baudrate(NULL, &baudrate) == -EINVAL);
  CHECK(ssf_mems_acquisition_get_baudrate(&device, NULL) == -EINVAL);
  CHECK(ssf_mems_acquisition_set_baudrate(NULL, SSF_MEMS_BAUDRATE_115200) == -EINVAL);
  CHECK(ssf_mems_acquisition_set_baudrate(&device, SSF_MEMS_BAUDRATE_MAX) == -EINVAL);
}

static void window(struct ssf_mems_acquisition_policy *policy,
                    struct ssf_mems_sensor_data *sample, unsigned int repeats,
                    unsigned int read_ms) {
  unsigned int i;
  ssf_mems_acquisition_policy_sample(policy, sample, read_ms, 0);
  for (i = 0; i < 32; i++) {
    if (i >= repeats)
      sample->temperature_x100++;
    ssf_mems_acquisition_policy_sample(policy, sample, read_ms, 0);
  }
}

static void test_policy(void) {
  struct ssf_mems_acquisition_policy policy;
  struct ssf_mems_sensor_data sample = {0};
  unsigned int i;

  ssf_mems_acquisition_policy_init(&policy, 0);
  window(&policy, &sample, 32, 5);
  CHECK(policy.interval_ms == 120 && policy.comparisons == 0);
  ssf_mems_acquisition_policy_init(&policy, 0);
  window(&policy, &sample, 0, 5);
  CHECK(policy.interval_ms == 95);
  ssf_mems_acquisition_policy_init(&policy, 0);
  window(&policy, &sample, 16, 5);
  CHECK(policy.interval_ms == 100);
  ssf_mems_acquisition_policy_init(&policy, 0);
  window(&policy, &sample, 0, 51);
  CHECK(policy.interval_ms == 100);

  ssf_mems_acquisition_policy_init(&policy, 0);
  policy.interval_ms = 999;
  window(&policy, &sample, 32, 5);
  CHECK(policy.interval_ms == 1000);
  ssf_mems_acquisition_policy_init(&policy, 0);
  policy.interval_ms = 20;
  window(&policy, &sample, 0, 5);
  CHECK(policy.interval_ms == 20);

  ssf_mems_acquisition_policy_init(&policy, 0);
  policy.interval_ms = 1000;
  ssf_mems_acquisition_policy_sample(&policy, &sample, 5, 30000);
  CHECK(policy.interval_ms == 950 && policy.next_probe == 60000);

  /* 每个字段变化都必须判为新数据，未使用的填充字节必须被忽略。 */
  for (i = 0; i < sizeof(sample); i++) {
    memset(&sample, 0, sizeof(sample));
    ssf_mems_acquisition_policy_init(&policy, 0);
    ssf_mems_acquisition_policy_sample(&policy, &sample, 5, 0);
    ((u8 *)&sample)[i] = 1;
    ssf_mems_acquisition_policy_sample(&policy, &sample, 5, 0);
    CHECK(policy.comparisons == 1);
    CHECK(policy.repeats == (i > offsetof(struct ssf_mems_sensor_data, startup_flags)));
  }

  ssf_mems_acquisition_policy_init(&policy, 0);
  for (i = 0; i < 4; i++)
    CHECK(!ssf_mems_acquisition_policy_error(&policy, -ETIMEDOUT, 3000));
  CHECK(ssf_mems_acquisition_policy_error(&policy, -ETIMEDOUT, 3000));
  CHECK(!ssf_mems_acquisition_policy_error(&policy, -EREMOTEIO, 3000));
  CHECK(policy.link_failures == 0 && policy.last_contact == 3000);
  CHECK(!ssf_mems_acquisition_policy_error(&policy, -EINVAL, 9000));
  CHECK(!ssf_mems_acquisition_policy_error(&policy, -ENOMEM, 9000));
  CHECK(policy.link_failures == 0);

  ssf_mems_acquisition_policy_init(&policy, 0);
  for (i = 0; i < 10; i++)
    CHECK(!ssf_mems_acquisition_policy_error(&policy, -EIO, 2999));
  CHECK(ssf_mems_acquisition_policy_error(&policy, -EIO, 3000));
  ssf_mems_acquisition_policy_sample(&policy, &sample, 5, 3001);
  CHECK(policy.link_failures == 0 && policy.last_contact == 3001);
  CHECK(!ssf_mems_acquisition_policy_error(&policy, -ETIMEDOUT, 3002));
  CHECK(!policy.have_previous && policy.comparisons == 0);

  ssf_mems_acquisition_policy_init(&policy, ULONG_MAX - 1000UL);
  for (i = 0; i < 4; i++)
    CHECK(!ssf_mems_acquisition_policy_error(&policy, -ETIMEDOUT, 1999UL));
  CHECK(ssf_mems_acquisition_policy_error(&policy, -ETIMEDOUT, 1999UL));
}

int main(void) {
  test_policy();
  test_thread_scenarios();
  printf("PASS acquisition policy and lifecycle: %u checks\n", checks);
  return 0;
}
