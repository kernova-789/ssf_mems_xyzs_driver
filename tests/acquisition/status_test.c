/* The production status reader must work while a serial transaction owns lock. */
#include "core.h"
#include "ssf_mems_acquisition.h"
#include <assert.h>

unsigned long jiffies = 12345;
unsigned int test_wake_count;
static int unexpected_feature(struct ssf_mems_data *data, unsigned int timeout) {
  (void)data; (void)timeout; abort();
}
static int unexpected_raw(struct ssf_mems_data *data, u32 samples, bool reset) {
  (void)data; (void)samples; (void)reset; abort();
}
static int unexpected_sound(struct ssf_mems_data *data) {
  (void)data;
  abort(); /* 主机配置不能执行采集回调。 */
}

int main(void) {
  struct ssf_mems_data sensor = {0};
  struct ssf_mems_acquisition_status status;
  struct ssf_mems_acquisition_config config;
  struct ssf_mems_sensor_ops ops = {.read_features = unexpected_feature, .collect_sound = unexpected_sound, .collect_raw = unexpected_raw};
  struct ssf_mems_sensor model = {.profile.raw_batch_max_samples = 32768};
  struct ssf_mems_acquisition_state *acq = &sensor.acquisition;

  sensor.sensor_ops = &ops;
  sensor.sensor = &model;

  mutex_init(&acq->lock);
  mutex_init(&acq->status_lock);
  acq->online = acq->have_sample = true;
  acq->last_sample = 12000;
  acq->policy.interval_ms = 1234;
  acq->config.mode = SSF_MEMS_ACQ_MODE_ALTERNATING;
  acq->config.raw_duration_ms = 5000;
  acq->config.feature_duration_ms = 7000;
  acq->policy.config.mode = SSF_MEMS_POLL_ADAPTIVE;
  acq->policy.config.fixed_interval_ms = 1000;
  acq->policy.config.min_interval_ms = 500;
  acq->policy.config.max_interval_ms = 5000;

  acq->config.poll = acq->policy.config;

  /* Using the transaction mutex in get_status would deadlock this call. */
  mutex_lock(&acq->lock);
  ssf_mems_acquisition_get_status(&sensor, &status);
  assert(status.online && status.have_sample);
  assert(status.interval_ms == 1234 && status.sample_age_ms == 345);
  /* 主机配置查询也必须避开正在执行原始采集或升级的事务锁。 */
  assert(ssf_mems_acquisition_get_config(&sensor, &config) == 0);
  assert(config.mode == SSF_MEMS_ACQ_MODE_ALTERNATING);
  assert(config.raw_duration_ms == 5000 && config.feature_duration_ms == 7000);
  assert(config.poll.mode == SSF_MEMS_POLL_ADAPTIVE);
  assert(config.poll.fixed_interval_ms == 1000 && config.poll.min_interval_ms == 500 &&
         config.poll.max_interval_ms == 5000);
  acq->have_sample = false;
  ssf_mems_acquisition_get_status(&sensor, &status);
  assert(!status.have_sample && status.sample_age_ms == 0);
  /* 所有主机配置写入也必须在事务锁被占用时立即返回。 */
  acq->config.alternating_en[SSF_MEMS_ACQ_ROUTE_FEATURE] = true;
  acq->config.alternating_en[SSF_MEMS_ACQ_ROUTE_RAW] = true;
  assert(ssf_mems_acquisition_set_mode(&sensor, SSF_MEMS_ACQ_MODE_RAW) == 0);
  assert(ssf_mems_acquisition_set_duration(&sensor, SSF_MEMS_ACQ_ROUTE_RAW, 300) == 0);
  assert(ssf_mems_acquisition_set_duration(&sensor, SSF_MEMS_ACQ_ROUTE_FEATURE, 400) == 0);
  assert(ssf_mems_acquisition_set_duration(&sensor, SSF_MEMS_ACQ_ROUTE_SOUND, 400) == 0);
  assert(ssf_mems_acquisition_set_raw_samples(&sensor, 2048) == 0);
  assert(ssf_mems_acquisition_set_mode(&sensor, SSF_MEMS_ACQ_MODE_SOUND) == 0);
  assert(ssf_mems_acquisition_set_mode(&sensor, SSF_MEMS_ACQ_MODE_RAW) == 0);
  assert(ssf_mems_acquisition_set_alternating_en(&sensor, SSF_MEMS_ACQ_ROUTE_RAW, false) == 0);
  assert(ssf_mems_acquisition_set_raw_manual(&sensor, true) == 0);
  assert(ssf_mems_acquisition_set_poll_setting(&sensor, SSF_MEMS_POLL_SETTING_FIXED_INTERVAL, 800) == 0);
  assert(ssf_mems_acquisition_get_config(&sensor, &config) == 0);
  assert(config.mode == SSF_MEMS_ACQ_MODE_RAW && config.raw_duration_ms == 300);
  assert(config.feature_duration_ms == 400 && config.raw_manual);
  assert(config.sound_duration_ms == 400 && config.raw_batch_samples == 2048);
  assert(!config.alternating_en[SSF_MEMS_ACQ_ROUTE_RAW]);
  assert(config.poll.fixed_interval_ms == 800 && acq->config_changed);
  mutex_unlock(&acq->lock);
  assert(mutex_trylock(&acq->status_lock));
  mutex_unlock(&acq->status_lock);
  pthread_mutex_destroy(&acq->status_lock.native);
  pthread_mutex_destroy(&acq->lock.native);
  puts("PASS status/config reads and writes while acquisition transaction lock is held");
  return 0;
}

bool ssf_mems_iio_raw_enabled(struct ssf_mems_data *data) { (void)data; return false; }
bool ssf_mems_iio_sound_enabled(struct ssf_mems_data *data) { (void)data; return false; }

int ssf_mems_iio_begin_raw(struct ssf_mems_data *data) {
  return ssf_mems_iio_raw_enabled(data) ? 0 : -ENODATA;
}
int ssf_mems_iio_begin_sound(struct ssf_mems_data *data) {
  return ssf_mems_iio_sound_enabled(data) ? 0 : -ENODATA;
}
