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
static struct ssf_mems_data device;
static struct serdev_device serial;
static struct task_struct task;
static int (*thread_function)(void *);
static void *thread_context;
static bool stop_requested, fail_thread_start;
static unsigned int get_count, set_count, read_count, publish_count;
static unsigned long read_started_at[80];
static unsigned int failure_count, wait_count, backoff_count;
static unsigned int backoffs[10];
static enum ssf_mems_baudrate sensor_baudrate;
static int get_error, set_error, publish_error;
static bool fail_reads;
static int setting_error;
static unsigned int setting_calls;
static unsigned int sampling_config_calls, sensor_setting_writes;
static u16 sensor_sampling_index;
static unsigned int raw_captures;
static unsigned int first_match_baudrate;
static struct ssf_mems_sensor_data published;
enum scenario {
  CHANGING, CONSTANT, LINK_FAILURE, EXCEPTION, NO_SENSOR, MANUAL,
  STOP_STARTUP, STOP_VERIFY, STOP_READ, STOP_CONFIGURE, CONFIG_FAILURE,
  RAW_RECOVERY_FAILURE, RAW_ONLY, MODE_SWITCH, RAW_ONLY_RECOVERY,
  RAW_ONLY_DROPS, SHORT_WINDOWS, OTA_BLOCKED, POLL_SWITCH,
  SAMPLING_TARGET_RECONNECT, RAW_UNSUPPORTED, RAW_ONLY_UNSUPPORTED, MANUAL_RAW_IDLE
};
static enum scenario scenario;
static unsigned int mode_switch_step; /* 记录运行中模式切换的测试阶段。 */
static unsigned long raw_started_at[80]; /* 检查短窗口不会被 1 s 轮询间隔拖长。 */

/* 停止接口无论线程在哪个阶段退出，都必须检查协议层残留的私有流。 */
int ssf_mems_protocol_recover_raw(struct ssf_mems_data *data) {
  CHECK(data == &device && data->acquisition.stopping);
  CHECK(data->acquisition.thread == NULL);
  data->protocol.mode = SSF_MEMS_LINK_MODBUS;
  return 0;
}

/* 原始模式由协议模块测试；这里核对采集状态机的交替顺序和计时。 */
int ssf_mems_raw_collect_finite(struct ssf_mems_data *data,
                                 unsigned int duration_ms, u32 length) {
  (void)length;
  CHECK(data == &device && duration_ms == device.acquisition.raw_duration_ms);
  CHECK(data->acquisition.phase == SSF_MEMS_ACQ_RAW);
  CHECK(raw_captures < ARRAY_SIZE(raw_started_at));
  raw_started_at[raw_captures++] = jiffies;
  if (scenario == RAW_UNSUPPORTED || scenario == RAW_ONLY_UNSUPPORTED) {
    jiffies += 5;
    return -EOPNOTSUPP;
  }
  jiffies += duration_ms;
  if (scenario == RAW_RECOVERY_FAILURE ||
      (scenario == RAW_ONLY_RECOVERY && raw_captures == 1)) {
    data->protocol.mode = SSF_MEMS_LINK_RAW_RECOVERING;
    return -ETIMEDOUT;
  }
  if (scenario == RAW_ONLY_DROPS && raw_captures == 2)
    return -ENOSPC;
  return 0;
}

int ssf_mems_protocol_read_setting(struct serdev_device *s,
                                  enum ssf_mems_sensor_setting setting,
                                  u16 *value, unsigned int timeout_ms) {
  CHECK(s == &serial && setting == SSF_MEMS_SETTING_SAMPLING_RATE);
  CHECK(timeout_ms > 200);
  setting_calls++;
  *value = sensor_sampling_index;
  return setting_error;
}

int ssf_mems_protocol_write_setting(struct serdev_device *s,
                                   enum ssf_mems_sensor_setting setting,
                                   u16 value, unsigned int timeout_ms) {
  CHECK(s == &serial && setting == SSF_MEMS_SETTING_SAMPLING_RATE);
  CHECK(timeout_ms > 200);
  if (value > (ssf_mems_xyzs.profile.sampling_frequency_count - 1U))
    return -EINVAL;
  setting_calls++;
  if (device.acquisition.phase == SSF_MEMS_ACQ_CONFIGURE) {
    /* 自动配置必须在匹配/切换波特率后、读取和发布样本之前进行。 */
    CHECK(get_count > 0 && !device.acquisition.online);
    CHECK(value == device.acquisition.target_sampling_rate_index);
    CHECK(device.protocol.host_baudrate ==
          (unsigned int)ssf_mems_baudrate_to_value(&ssf_mems_xyzs, device.acquisition.target_baudrate));
    sampling_config_calls++;
    if (!setting_error || scenario == CONFIG_FAILURE) {
      /* CONFIG_FAILURE 模拟写入生效但确认丢失；重试不能重复写同一值。 */
      if (value != sensor_sampling_index) {
        sensor_sampling_index = value;
        sensor_setting_writes++;
      }
    }
    if (scenario == STOP_CONFIGURE)
      stop_requested = true;
  } else if (!setting_error && value != sensor_sampling_index) {
    sensor_sampling_index = value;
    sensor_setting_writes++;
  }
  return setting_error;
}

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

ssize_t serdev_device_write(struct serdev_device *dev, const u8 *buf,
                            size_t len, unsigned long timeout) {
  (void)dev; (void)buf; (void)len; (void)timeout;
  abort(); /* 采集层测试使用协议替身，不应直接发送串口数据。 */
}

int ssf_mems_baudrate_to_value(const struct ssf_mems_sensor *model, enum ssf_mems_baudrate baudrate) {
  (void)model;
  static const int rates[] = {9600, 2400, 4800, 9600, 19200, 38400,
                             57600, 115200, 128000, 230400, 256000, 460800,
                             500000, 512000, 600000, 750000, 921600, 1000000};
  CHECK((unsigned int)baudrate < sizeof(rates) / sizeof(rates[0]));
  return rates[baudrate];
}

int ssf_mems_protocol_get_baudrate(struct serdev_device *s,
                                  enum ssf_mems_baudrate *baudrate,
                                  unsigned int timeout_ms) {
  CHECK(s == &serial && timeout_ms == 200);
  get_count++;
  if ((scenario == RAW_RECOVERY_FAILURE || scenario == RAW_ONLY_RECOVERY) && get_count > 1) {
    CHECK(device.protocol.mode == SSF_MEMS_LINK_RAW_RECOVERING);
    CHECK(device.acquisition.phase == SSF_MEMS_ACQ_MATCH);
    device.protocol.mode = SSF_MEMS_LINK_MODBUS;
  }
  if (get_count == 1)
    first_match_baudrate = device.protocol.host_baudrate;
  if (get_error) {
    jiffies += timeout_ms;
    return get_error;
  }
  *baudrate = sensor_baudrate;
  device.protocol.baudrate = sensor_baudrate;
  device.protocol.host_baudrate = ssf_mems_baudrate_to_value(&ssf_mems_xyzs, sensor_baudrate);
  return 0;
}

int ssf_mems_protocol_set_baudrate(struct serdev_device *s,
                                  enum ssf_mems_baudrate baudrate,
                                  unsigned int timeout_ms) {
  CHECK(s == &serial && timeout_ms == 200);
  set_count++;
  /* 写请求必须在旧速率完成；模拟丢应答时传感器已切换而主机仍在旧速率。 */
  CHECK(device.protocol.host_baudrate ==
        (unsigned int)ssf_mems_baudrate_to_value(&ssf_mems_xyzs, sensor_baudrate));
  sensor_baudrate = baudrate;
  if (set_error)
    return set_error;
  device.protocol.baudrate = baudrate;
  device.protocol.host_baudrate = ssf_mems_baudrate_to_value(&ssf_mems_xyzs, baudrate);
  return 0;
}

void ssf_mems_protocol_invalidate_features(struct ssf_mems_data *data) {
  data->protocol.valid = false;
}

int ssf_mems_sensor_read_features(struct ssf_mems_data *data,
                                   unsigned int timeout_ms) {
  CHECK(data->serdev == &serial);
  CHECK(device.protocol.mode == SSF_MEMS_LINK_MODBUS);
  CHECK(device.protocol.host_baudrate ==
        (unsigned int)ssf_mems_baudrate_to_value(&ssf_mems_xyzs, sensor_baudrate));
  CHECK(timeout_ms > 200);
  if (sensor_baudrate == SSF_MEMS_BAUDRATE_2400)
    CHECK(timeout_ms > 1000);
  CHECK(read_count < ARRAY_SIZE(read_started_at));
  read_started_at[read_count++] = jiffies;
  CHECK(sampling_config_calls > 0 &&
        sensor_sampling_index == device.acquisition.target_sampling_rate_index);
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
    device.protocol.features.xyzs.x.acc_rms_x100 = read_count;
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

int ssf_mems_iio_publish_features(struct ssf_mems_data *data,
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
  } else if (scenario == MANUAL_RAW_IDLE && wait_count >= 4) {
    CHECK(raw_captures == 0);
    stop_requested = true;
  } else if (scenario == OTA_BLOCKED) {
    CHECK(device.acquisition.firmware_blocked == true);
    CHECK(get_count == 0 && set_count == 0 && read_count == 0 && raw_captures == 0);
    if (wait_count == 4)
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
    } else if (scenario == CONFIG_FAILURE) {
      CHECK(read_count == 0 && publish_count == 0);
      CHECK(set_count == 1 && sampling_config_calls == 1);
      setting_error = 0;
    } else if (scenario == RAW_RECOVERY_FAILURE) {
      CHECK(raw_captures == 1 && get_count == 1 && publish_count >= 2);
      CHECK(device.protocol.mode == SSF_MEMS_LINK_RAW_RECOVERING);
    } else if (scenario == RAW_ONLY_RECOVERY) {
      CHECK(raw_captures == 1 && get_count == 1 && publish_count == 0);
      CHECK(device.protocol.mode == SSF_MEMS_LINK_RAW_RECOVERING);
    } else {
      CHECK((scenario == LINK_FAILURE || scenario == SAMPLING_TARGET_RECONNECT) &&
            failure_count >= 5);
      CHECK(publish_count == 1);
      /* 模拟传感器掉电，启动后回到默认速率。 */
      sensor_baudrate = SSF_MEMS_BAUDRATE_9600;
      sensor_sampling_index = 0;
      fail_reads = false;
    }
  } else if (device.acquisition.phase == SSF_MEMS_ACQ_POLL) {
    CHECK(timeout >= 2);
    if (scenario == SAMPLING_TARGET_RECONNECT && mode_switch_step == 0) {
      CHECK(ssf_mems_acquisition_write_setting(
          &device, SSF_MEMS_SETTING_SAMPLING_RATE, 4) == 0);
      CHECK(device.acquisition.target_sampling_rate_index == 4);
      setting_error = -ETIMEDOUT;
      CHECK(ssf_mems_acquisition_write_setting(
          &device, SSF_MEMS_SETTING_SAMPLING_RATE, 5) == -ETIMEDOUT);
      CHECK(device.acquisition.target_sampling_rate_index == 4);
      setting_error = 0;
      CHECK(ssf_mems_acquisition_write_setting(
          &device, SSF_MEMS_SETTING_SAMPLING_RATE, 99) == -EINVAL);
      CHECK(device.acquisition.target_sampling_rate_index == 4);
      mode_switch_step++;
      fail_reads = true;
    } else if (scenario == RAW_UNSUPPORTED) {
      CHECK(device.acquisition.online && device.protocol.valid && get_count == 1);
      if (raw_captures == 3)
        stop_requested = true;
    } else if (scenario == RAW_ONLY || scenario == RAW_ONLY_RECOVERY ||
               scenario == RAW_ONLY_DROPS || scenario == RAW_ONLY_UNSUPPORTED) {
      CHECK(device.acquisition.online && !device.protocol.valid);
      CHECK(publish_count == 0 && read_count == 0);
      if (scenario == RAW_ONLY_UNSUPPORTED && raw_captures != 0)
        CHECK(timeout >= 1000);
      if (raw_captures == 3)
        stop_requested = true;
    } else if (scenario == SHORT_WINDOWS) {
      CHECK(device.acquisition.online && publish_count == raw_captures + 1);
      if (raw_captures == 5)
        stop_requested = true;
    } else if (scenario == MODE_SWITCH) {
      if (mode_switch_step == 0) {
        CHECK(publish_count == 1 && raw_captures == 0);
        CHECK(ssf_mems_acquisition_set_mode(&device, SSF_MEMS_ACQ_MODE_RAW) == 0);
        mode_switch_step++;
      } else if (mode_switch_step == 1 && raw_captures == 2) {
        CHECK(publish_count == 1);
        CHECK(ssf_mems_acquisition_set_mode(&device, SSF_MEMS_ACQ_MODE_FEATURE) == 0);
        mode_switch_step++;
      } else if (mode_switch_step == 2 && publish_count == 3) {
        CHECK(raw_captures == 2);
        CHECK(ssf_mems_acquisition_set_duration(&device, true, 100) == 0);
        CHECK(ssf_mems_acquisition_set_duration(&device, false, 100) == 0);
        CHECK(ssf_mems_acquisition_set_mode(&device, SSF_MEMS_ACQ_MODE_ALTERNATING) == 0);
        mode_switch_step++;
      } else if (mode_switch_step == 3 && raw_captures >= 3) {
        CHECK(publish_count >= 4);
        stop_requested = true;
      }
    } else if (scenario == POLL_SWITCH) {
      if (mode_switch_step == 0) {
        unsigned long until = device.acquisition.feature_until;
        CHECK(publish_count == 1);
        CHECK(ssf_mems_acquisition_set_poll_setting(
            &device, SSF_MEMS_POLL_SETTING_FIXED_INTERVAL, 2500) == 0);
        CHECK(device.acquisition.poll_changed && !device.acquisition.config_changed);
        CHECK(device.acquisition.feature_until == until);
        mode_switch_step++;
      } else if (mode_switch_step == 1 && publish_count == 4) {
        CHECK(ssf_mems_acquisition_set_poll_setting(
            &device, SSF_MEMS_POLL_SETTING_MODE, SSF_MEMS_POLL_ADAPTIVE) == 0);
        mode_switch_step++;
      } else if (mode_switch_step == 2 && publish_count == 40) {
        CHECK(device.acquisition.policy.interval_ms < 2500);
        CHECK(ssf_mems_acquisition_set_poll_setting(
            &device, SSF_MEMS_POLL_SETTING_MODE, SSF_MEMS_POLL_FIXED) == 0);
        CHECK(device.acquisition.policy.interval_ms == 2500);
        mode_switch_step++;
      } else if (mode_switch_step == 3 && publish_count == 45) {
        stop_requested = true;
      }
    } else if (scenario == CHANGING || scenario == CONSTANT) {
      CHECK(device.acquisition.online && device.protocol.valid);
      if (publish_count >= 70)
        stop_requested = true;
    } else if (scenario == RAW_RECOVERY_FAILURE && get_count == 2) {
      CHECK(raw_captures == 1 && device.acquisition.online);
      CHECK(device.protocol.mode == SSF_MEMS_LINK_MODBUS);
      stop_requested = true;
    } else if ((scenario == LINK_FAILURE || scenario == SAMPLING_TARGET_RECONNECT) &&
               publish_count == 2) {
      CHECK(get_count == 2 && set_count == 2);
      CHECK(device.acquisition.policy.interval_ms ==
            device.acquisition.policy.config.fixed_interval_ms);
      stop_requested = true;
    } else if (scenario == EXCEPTION && failure_count >= 20) {
      CHECK(get_count == 1 && set_count == 1);
      CHECK(device.acquisition.online && !device.protocol.valid);
      stop_requested = true;
    } else if ((scenario == MANUAL || scenario == CONFIG_FAILURE) && publish_count == 1) {
      stop_requested = true;
    }
  }
  jiffies += timeout;
}

static void reset_device(enum scenario next) {
  memset(&device, 0, sizeof(device));
  memset(&serial, 0, sizeof(serial));
  device.serdev = &serial;
  device.sensor = &ssf_mems_xyzs;
  device.sensor_ops = ssf_mems_xyzs.ops;
  device.indio_dev = (void *)&task;
  device.protocol.host_baudrate = 9600;
  device.protocol.baudrate = SSF_MEMS_BAUDRATE_9600;
  sensor_baudrate = SSF_MEMS_BAUDRATE_9600;
  sensor_sampling_index = 9; /* 模拟开发板传感器仍保留上一版的最大采样率。 */
  jiffies = 0;
  get_count = set_count = read_count = publish_count = failure_count = 0;
  wait_count = backoff_count = 0;
  get_error = set_error = publish_error = 0;
  setting_error = 0;
  setting_calls = sampling_config_calls = sensor_setting_writes = 0;
  fail_reads = fail_thread_start = stop_requested = false;
  scenario = next;
  /* 旧场景显式选择 115200 目标，随后从已匹配的 9600 链路模拟各种恢复情况。 */
  device.protocol.baudrate = SSF_MEMS_BAUDRATE_128000;
  CHECK(ssf_mems_acquisition_init(&device) == 0);
  device.protocol.baudrate = SSF_MEMS_BAUDRATE_9600;
  first_match_baudrate = 0;
  CHECK(device.acquisition.raw_duration_ms == 5000);
  CHECK(device.acquisition.feature_duration_ms == 5000);
  CHECK(device.acquisition.target_baudrate == ssf_mems_xyzs.profile.default_work_baudrate);
  CHECK(device.acquisition.target_sampling_rate_index ==
        ssf_mems_xyzs.profile.default_sampling_rate_index);
  device.acquisition.mode = SSF_MEMS_ACQ_MODE_FEATURE;
  mode_switch_step = 0;
  raw_captures = 0;
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
  CHECK(!status.online && !status.have_sample && status.interval_ms == 1000);
  run_thread();
  CHECK(get_count == 1 && set_count == 1 && publish_count == 70);
  CHECK(sensor_baudrate == SSF_MEMS_BAUDRATE_128000);
  CHECK(sampling_config_calls == 1 && sensor_setting_writes == 1);
  CHECK(device.acquisition.policy.interval_ms == 1000);
  /* 排除首次连接验证后完整等待的一轮，普通轮询应扣除读取耗时。
   * 验证真实调度调用间隔，而不仅检查周期配置值。 */
  for (unsigned int i = 2; i < read_count; i++)
    CHECK(read_started_at[i] - read_started_at[i - 1] == 1000);
  CHECK(published.xyzs.x.acc_rms_x100 == read_count);
  ssf_mems_acquisition_get_status(&device, &status);
  CHECK(status.have_sample && !status.online && status.sample_age_ms > 0);
  CHECK(ssf_mems_acquisition_set_baudrate(&device, SSF_MEMS_BAUDRATE_57600) == -ENODEV);
  CHECK(ssf_mems_acquisition_get_baudrate(&device, &baudrate) == -ENODEV);

  reset_device(CONSTANT);
  run_thread();
  CHECK(device.acquisition.policy.interval_ms == 1000);
  for (unsigned int i = 2; i < read_count; i++)
    CHECK(read_started_at[i] - read_started_at[i - 1] == 1000);

  reset_device(LINK_FAILURE);
  fail_reads = true;
  run_thread();
  CHECK(backoff_count == 1 && backoffs[0] == 1000 && publish_count == 2);
  CHECK(sampling_config_calls == 2 && sensor_setting_writes == 2);

  reset_device(SAMPLING_TARGET_RECONNECT);
  run_thread();
  CHECK(backoff_count == 1 && sampling_config_calls == 2);
  CHECK(device.acquisition.target_sampling_rate_index == 4 && sensor_sampling_index == 4);
  CHECK(sensor_setting_writes == 3); /* 首次默认值、手动值、掉电后恢复手动值。 */

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
  CHECK(device.acquisition.target_baudrate == SSF_MEMS_BAUDRATE_128000);
  CHECK(device.protocol.host_baudrate == 9600 && device.acquisition.reconnect);
  set_error = 0;
  get_count = set_count = 0;
  run_thread();
  CHECK(sensor_baudrate == SSF_MEMS_BAUDRATE_128000 && set_count == 1);

  reset_device(MANUAL);
  sensor_baudrate = SSF_MEMS_BAUDRATE_128000;
  device.protocol.host_baudrate = 115200;
  device.protocol.baudrate = sensor_baudrate;
  sensor_sampling_index = 6;
  publish_error = -EIO;
  run_thread();
  CHECK(set_count == 0 && publish_count == 1 && backoff_count == 0);
  CHECK(sampling_config_calls == 1 && sensor_setting_writes == 0);

  /* 已保存上一版最高速率的传感器，加载新驱动后应降到本次目标。 */
  reset_device(MANUAL);
  sensor_baudrate = SSF_MEMS_BAUDRATE_1000000;
  device.protocol.host_baudrate = 1000000;
  device.protocol.baudrate = sensor_baudrate;
  run_thread();
  CHECK(sensor_baudrate == SSF_MEMS_BAUDRATE_128000 && set_count == 1);
  CHECK(sensor_sampling_index == 6 && sensor_setting_writes == 1);

  reset_device(CONFIG_FAILURE);
  setting_error = -ETIMEDOUT;
  run_thread();
  CHECK(backoff_count == 1 && publish_count == 1);
  CHECK(get_count == 2 && set_count == 1);
  CHECK(sampling_config_calls == 2 && sensor_setting_writes == 1);

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
  CHECK(get_count == 0 && set_count == 0 && read_count == 0 && setting_calls == 0);
  reset_device(STOP_VERIFY);
  run_thread();
  CHECK(get_count == 1 && set_count == 1 && publish_count == 0 && setting_calls == 0);
  reset_device(STOP_CONFIGURE);
  run_thread();
  CHECK(sampling_config_calls == 1 && read_count == 0 && publish_count == 0);
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
  CHECK(ssf_mems_acquisition_set_baudrate(NULL, SSF_MEMS_BAUDRATE_128000) == -EINVAL);
  CHECK(ssf_mems_acquisition_set_baudrate(&device, SSF_MEMS_BAUDRATE_MAX) == -EINVAL);
}

static void window(struct ssf_mems_acquisition_policy *policy,
                    struct ssf_mems_sensor_data *sample, unsigned int repeats,
                    unsigned int read_ms) {
  unsigned int i;
  ssf_mems_acquisition_policy_sample(policy, sample, read_ms, 0);
  for (i = 0; i < 32; i++) {
    if (i >= repeats)
      sample->xyzs.temperature_x100++;
    ssf_mems_acquisition_policy_sample(policy, sample, read_ms, 0);
  }
}

static void adaptive_policy(struct ssf_mems_acquisition_policy *policy,
                              unsigned long now) {
  struct ssf_mems_poll_config config;

  ssf_mems_acquisition_policy_init(policy, &ssf_mems_xyzs, now);
  config = policy->config;
  config.mode = SSF_MEMS_POLL_ADAPTIVE;
  CHECK(ssf_mems_acquisition_policy_configure(policy, &config, now) == 0);
}

static void test_policy(void) {
  struct ssf_mems_acquisition_policy policy;
  struct ssf_mems_sensor_data sample = {0};
  struct ssf_mems_poll_config config;
  unsigned int i;

  ssf_mems_acquisition_policy_init(&policy, &ssf_mems_xyzs, 0);
  window(&policy, &sample, 32, 5);
  CHECK(policy.interval_ms == 1000 && policy.comparisons == 0);
  ssf_mems_acquisition_policy_init(&policy, &ssf_mems_xyzs, 0);
  window(&policy, &sample, 0, 5);
  CHECK(policy.interval_ms == 1000);
  ssf_mems_acquisition_policy_init(&policy, &ssf_mems_xyzs, 0);
  window(&policy, &sample, 16, 5);
  CHECK(policy.interval_ms == 1000);
  ssf_mems_acquisition_policy_init(&policy, &ssf_mems_xyzs, 0);
  window(&policy, &sample, 0, 501);
  CHECK(policy.interval_ms == 1000);

  adaptive_policy(&policy, 0);
  policy.interval_ms = policy.config.max_interval_ms;
  window(&policy, &sample, 32, 5);
  CHECK(policy.interval_ms == policy.config.max_interval_ms);
  adaptive_policy(&policy, 0);
  policy.interval_ms = policy.config.min_interval_ms;
  window(&policy, &sample, 0, 5);
  CHECK(policy.interval_ms == policy.config.min_interval_ms);

  adaptive_policy(&policy, 0);
  policy.interval_ms = 1000;
  ssf_mems_acquisition_policy_sample(&policy, &sample, 5, 30000);
  CHECK(policy.interval_ms == 1000 && policy.next_probe == 60000);
  sample.xyzs.temperature_x100++;
  ssf_mems_acquisition_policy_sample(&policy, &sample, 5, 60000);
  CHECK(policy.interval_ms == 950 && policy.next_probe == 90000);

  /* 每个字段变化都必须判为新数据，未使用的填充字节必须被忽略。 */
  for (i = 0; i < sizeof(sample); i++) {
    memset(&sample, 0, sizeof(sample));
    adaptive_policy(&policy, 0);
    ssf_mems_acquisition_policy_sample(&policy, &sample, 5, 0);
    ((u8 *)&sample)[i] = 1;
    ssf_mems_acquisition_policy_sample(&policy, &sample, 5, 0);
    CHECK(policy.comparisons == 1);
    CHECK(policy.repeats == (i > offsetof(struct ssf_mems_sensor_data, xyzs.startup_flags)));
  }

  ssf_mems_acquisition_policy_init(&policy, &ssf_mems_xyzs, 0);
  for (i = 0; i < 4; i++)
    CHECK(!ssf_mems_acquisition_policy_error(&policy, -ETIMEDOUT, 3000));
  CHECK(ssf_mems_acquisition_policy_error(&policy, -ETIMEDOUT, 3000));
  CHECK(!ssf_mems_acquisition_policy_error(&policy, -EREMOTEIO, 3000));
  CHECK(policy.link_failures == 0 && policy.last_contact == 3000);
  CHECK(!ssf_mems_acquisition_policy_error(&policy, -EINVAL, 9000));
  CHECK(!ssf_mems_acquisition_policy_error(&policy, -ENOMEM, 9000));
  CHECK(policy.link_failures == 0);

  ssf_mems_acquisition_policy_init(&policy, &ssf_mems_xyzs, 0);
  for (i = 0; i < 10; i++)
    CHECK(!ssf_mems_acquisition_policy_error(&policy, -EIO, 2999));
  CHECK(ssf_mems_acquisition_policy_error(&policy, -EIO, 3000));
  ssf_mems_acquisition_policy_sample(&policy, &sample, 5, 3001);
  CHECK(policy.link_failures == 0 && policy.last_contact == 3001);
  CHECK(!ssf_mems_acquisition_policy_error(&policy, -ETIMEDOUT, 3002));
  CHECK(!policy.have_previous && policy.comparisons == 0);

  ssf_mems_acquisition_policy_init(&policy, &ssf_mems_xyzs, ULONG_MAX - 1000UL);
  for (i = 0; i < 4; i++)
    CHECK(!ssf_mems_acquisition_policy_error(&policy, -ETIMEDOUT, 1999UL));
  CHECK(ssf_mems_acquisition_policy_error(&policy, -ETIMEDOUT, 1999UL));

  adaptive_policy(&policy, 0);
  window(&policy, &sample, 32, 5);
  CHECK(policy.interval_ms == 1200);
  adaptive_policy(&policy, 0);
  window(&policy, &sample, 0, 5);
  CHECK(policy.interval_ms == 950);
  adaptive_policy(&policy, 0);
  window(&policy, &sample, 16, 5);
  CHECK(policy.interval_ms == 1000);
  adaptive_policy(&policy, 0);
  window(&policy, &sample, 0, 501);
  CHECK(policy.interval_ms == 1000);

  /* 30 秒试探保留已有统计，第 32 次比较仍能按重复率调整。 */
  adaptive_policy(&policy, 0);
  ssf_mems_acquisition_policy_sample(&policy, &sample, 5, 0);
  for (i = 1; i <= 30; i++)
    ssf_mems_acquisition_policy_sample(&policy, &sample, 5, i * 1000UL);
  CHECK(policy.comparisons == 30 && policy.repeats == 30 && policy.interval_ms == 1000);
  ssf_mems_acquisition_policy_contact(&policy, 30500);
  CHECK(policy.comparisons == 30 && policy.interval_ms == 1000);
  ssf_mems_acquisition_policy_sample(&policy, &sample, 5, 31000);
  ssf_mems_acquisition_policy_sample(&policy, &sample, 5, 32000);
  CHECK(policy.comparisons == 0 && policy.interval_ms == 1200);

  /* 完整窗口与定期试探同时到期时，只执行窗口决策。 */
  adaptive_policy(&policy, 0);
  for (i = 0; i < 32; i++)
    ssf_mems_acquisition_policy_sample(&policy, &sample, 5, 0);
  ssf_mems_acquisition_policy_sample(&policy, &sample, 5, 30000);
  CHECK(policy.interval_ms == 1200 && policy.next_probe == 60000);

  /* 重连保留学习到的周期和用户配置，配置变更不伪造有效通信。 */
  ssf_mems_acquisition_policy_restart(&policy, 40000);
  CHECK(policy.interval_ms == 1200 && policy.config.mode == SSF_MEMS_POLL_ADAPTIVE);
  CHECK(!policy.have_previous && policy.comparisons == 0);
  policy.link_failures = 3;
  policy.last_contact = 123;
  config = policy.config;
  config.min_interval_ms = 1500;
  CHECK(ssf_mems_acquisition_policy_configure(&policy, &config, 41000) == 0);
  CHECK(policy.interval_ms == 1500 && policy.link_failures == 3 && policy.last_contact == 123);
  ssf_mems_acquisition_policy_sample(&policy, &sample, 5, 42000);
  ssf_mems_acquisition_policy_sample(&policy, &sample, 5, 43000);
  config = policy.config;
  config.fixed_interval_ms = 2200; /* 非当前模式的配置不清空学习统计。 */
  CHECK(ssf_mems_acquisition_policy_configure(&policy, &config, 44000) == 0);
  CHECK(policy.comparisons == 1 && policy.interval_ms == 1500);
  config.mode = SSF_MEMS_POLL_FIXED;
  CHECK(ssf_mems_acquisition_policy_configure(&policy, &config, 45000) == 0);
  CHECK(policy.interval_ms == 2200);
  window(&policy, &sample, 32, 5);
  ssf_mems_acquisition_policy_sample(&policy, &sample, 5, 100000);
  CHECK(policy.interval_ms == 2200 && policy.comparisons == 0);
}

/* 每次按生效周期推进时间，验证长周期不会被 30 秒试探反向缩短。 */
static void test_timed_policy(void) {
  struct ssf_mems_acquisition_policy policy;
  struct ssf_mems_sensor_data sample = {0};
  struct ssf_mems_poll_config config;
  unsigned long now;
  unsigned int previous_interval;

  for (unsigned int start = 20000; start <= 60000; start += 40000) {
    ssf_mems_acquisition_policy_init(&policy, &ssf_mems_xyzs, 0);
    config = policy.config;
    config.fixed_interval_ms = start;
    config.max_interval_ms = 60000;
    CHECK(ssf_mems_acquisition_policy_configure(&policy, &config, 0) == 0);
    config.mode = SSF_MEMS_POLL_ADAPTIVE;
    CHECK(ssf_mems_acquisition_policy_configure(&policy, &config, 0) == 0);
    now = 0;
    ssf_mems_acquisition_policy_sample(&policy, &sample, 5, now);
    for (unsigned int i = 0; i < 96; i++) {
      previous_interval = policy.interval_ms;
      now += previous_interval;
      ssf_mems_acquisition_policy_sample(&policy, &sample, 5, now);
      CHECK(policy.interval_ms >= previous_interval && policy.interval_ms <= 60000);
      if (i % 7 == 0) {
        /* 原始窗口只更新通信状态，不丢弃尚未完成的比较统计。 */
        unsigned int comparisons = policy.comparisons;
        now += 5000;
        ssf_mems_acquisition_policy_contact(&policy, now);
        CHECK(policy.comparisons == comparisons);
      }
    }
    CHECK(policy.interval_ms == (start == 60000 ? 60000 : 34560));
  }

  /* 有持续变化且通信有余量时，长周期仍能试探提速。 */
  adaptive_policy(&policy, 0);
  config = policy.config;
  config.max_interval_ms = 60000;
  CHECK(ssf_mems_acquisition_policy_configure(&policy, &config, 0) == 0);
  policy.interval_ms = 60000;
  now = 0;
  ssf_mems_acquisition_policy_sample(&policy, &sample, 5, now);
  for (unsigned int i = 0; i < 8; i++) {
    previous_interval = policy.interval_ms;
    now += previous_interval;
    sample.xyzs.temperature_x100++;
    ssf_mems_acquisition_policy_sample(&policy, &sample, 5, now);
    CHECK(policy.interval_ms < previous_interval);
  }
  CHECK(policy.comparisons == 8 && policy.repeats == 0);

  /* 少量变化不能绕过窗口中已经明确的高重复率。 */
  adaptive_policy(&policy, 0);
  now = 0;
  ssf_mems_acquisition_policy_sample(&policy, &sample, 5, now);
  for (unsigned int i = 0; i < 10; i++) {
    now += policy.interval_ms;
    ssf_mems_acquisition_policy_sample(&policy, &sample, 5, now);
  }
  now = policy.next_probe;
  sample.xyzs.temperature_x100++;
  ssf_mems_acquisition_policy_sample(&policy, &sample, 5, now);
  CHECK(policy.interval_ms == 1000 && policy.repeats == 10);

  /* 数据虽变化，读取占用超过半个周期也不能试探提速。 */
  adaptive_policy(&policy, 0);
  ssf_mems_acquisition_policy_sample(&policy, &sample, 501, 0);
  sample.xyzs.temperature_x100++;
  ssf_mems_acquisition_policy_sample(&policy, &sample, 501, 30000);
  CHECK(policy.interval_ms == 1000);
}

/* 配置在主机生效；非法写入不改变已设置的周期或上下限。 */
static void test_poll_configuration(void) {
  struct ssf_mems_acquisition_config config;
  unsigned int wakes;

  reset_device(MANUAL);
  CHECK(ssf_mems_acquisition_get_config(&device, &config) == 0);
  CHECK(config.poll.mode == SSF_MEMS_POLL_FIXED && config.poll.fixed_interval_ms == 1000);
  CHECK(config.poll.min_interval_ms == 500 && config.poll.max_interval_ms == 5000);
  wakes = test_wake_count;
  CHECK(ssf_mems_acquisition_set_poll_setting(
      &device, SSF_MEMS_POLL_SETTING_FIXED_INTERVAL, 1000) == 0);
  CHECK(test_wake_count == wakes && !device.acquisition.poll_changed);
  CHECK(ssf_mems_acquisition_set_poll_setting(
      &device, SSF_MEMS_POLL_SETTING_FIXED_INTERVAL, 1700) == 0);
  CHECK(device.acquisition.policy.interval_ms == 1700 && device.acquisition.poll_changed);
  CHECK(!device.acquisition.config_changed && !device.acquisition.reconnect);
  device.acquisition.poll_changed = false;
  wakes = test_wake_count;
  CHECK(ssf_mems_acquisition_set_poll_setting(
      &device, SSF_MEMS_POLL_SETTING_MIN_INTERVAL, 200) == 0);
  CHECK(device.acquisition.policy.interval_ms == 1700 && test_wake_count == wakes);
  CHECK(ssf_mems_acquisition_set_poll_setting(
      &device, SSF_MEMS_POLL_SETTING_MODE, SSF_MEMS_POLL_ADAPTIVE) == 0);
  CHECK(ssf_mems_acquisition_set_poll_setting(
      &device, SSF_MEMS_POLL_SETTING_MIN_INTERVAL, 1800) == 0);
  CHECK(device.acquisition.policy.interval_ms == 1800);
  CHECK(ssf_mems_acquisition_set_poll_setting(
      &device, SSF_MEMS_POLL_SETTING_MAX_INTERVAL, 1900) == 0);
  CHECK(ssf_mems_acquisition_set_poll_setting(
      &device, SSF_MEMS_POLL_SETTING_MAX_INTERVAL, 1700) == -EINVAL);
  CHECK(ssf_mems_acquisition_set_poll_setting(
      &device, SSF_MEMS_POLL_SETTING_MIN_INTERVAL, 2000) == -EINVAL);
  for (unsigned int field = SSF_MEMS_POLL_SETTING_FIXED_INTERVAL;
       field <= SSF_MEMS_POLL_SETTING_MAX_INTERVAL; field++) {
    CHECK(ssf_mems_acquisition_set_poll_setting(&device, field, 0) == -EINVAL);
    CHECK(ssf_mems_acquisition_set_poll_setting(&device, field, 99) == -EINVAL);
    CHECK(ssf_mems_acquisition_set_poll_setting(&device, field, 60001) == -EINVAL);
    CHECK(ssf_mems_acquisition_set_poll_setting(&device, field, UINT_MAX) == -EINVAL);
  }
  CHECK(ssf_mems_acquisition_set_poll_setting(
      &device, SSF_MEMS_POLL_SETTING_MODE, SSF_MEMS_POLL_MODE_COUNT) == -EINVAL);
  CHECK(ssf_mems_acquisition_set_poll_setting(&device, -1, 1000) == -EINVAL);
  CHECK(ssf_mems_acquisition_set_poll_setting(NULL, SSF_MEMS_POLL_SETTING_MODE, 0) == -EINVAL);
  CHECK(ssf_mems_acquisition_get_config(&device, &config) == 0);
  CHECK(config.poll.fixed_interval_ms == 1700 && config.poll.min_interval_ms == 1800 &&
        config.poll.max_interval_ms == 1900 && device.acquisition.policy.interval_ms == 1800);
  CHECK(ssf_mems_acquisition_set_poll_setting(
      &device, SSF_MEMS_POLL_SETTING_FIXED_INTERVAL, 60000) == 0);
  CHECK(device.acquisition.policy.interval_ms == 1800);
  CHECK(ssf_mems_acquisition_set_poll_setting(
      &device, SSF_MEMS_POLL_SETTING_MODE, SSF_MEMS_POLL_FIXED) == 0);
  CHECK(device.acquisition.policy.interval_ms == 60000);
  CHECK(ssf_mems_acquisition_set_poll_setting(
      &device, SSF_MEMS_POLL_SETTING_FIXED_INTERVAL, 100) == 0);
  CHECK(device.acquisition.policy.interval_ms == 100);
  CHECK(ssf_mems_acquisition_set_poll_setting(
      &device, SSF_MEMS_POLL_SETTING_MAX_INTERVAL, 60000) == 0);
  CHECK(ssf_mems_acquisition_set_poll_setting(
      &device, SSF_MEMS_POLL_SETTING_MIN_INTERVAL, 100) == 0);
  CHECK(get_count == 0 && set_count == 0 && read_count == 0 && setting_calls == 0);
  device.acquisition.stopping = true;
  CHECK(ssf_mems_acquisition_set_poll_setting(
      &device, SSF_MEMS_POLL_SETTING_FIXED_INTERVAL, 1000) == -ENODEV);
}

/* 通过真实采集线程验证固定周期、自适应、交替保留统计和运行中切换。 */
static void test_poll_scheduling(void) {
  unsigned int i;

  reset_device(CHANGING);
  CHECK(ssf_mems_acquisition_set_poll_setting(
      &device, SSF_MEMS_POLL_SETTING_FIXED_INTERVAL, 1750) == 0);
  run_thread();
  for (i = 2; i < read_count; i++)
    CHECK(read_started_at[i] - read_started_at[i - 1] == 1750);
  CHECK(device.acquisition.policy.config.fixed_interval_ms == 1750);

  reset_device(CHANGING);
  CHECK(ssf_mems_acquisition_set_poll_setting(
      &device, SSF_MEMS_POLL_SETTING_MODE, SSF_MEMS_POLL_ADAPTIVE) == 0);
  run_thread();
  CHECK(device.acquisition.policy.interval_ms < 1000 &&
        device.acquisition.policy.interval_ms >= 500);

  reset_device(CONSTANT);
  CHECK(ssf_mems_acquisition_set_poll_setting(
      &device, SSF_MEMS_POLL_SETTING_MODE, SSF_MEMS_POLL_ADAPTIVE) == 0);
  device.acquisition.mode = SSF_MEMS_ACQ_MODE_ALTERNATING;
  device.acquisition.raw_duration_ms = 100;
  device.acquisition.feature_duration_ms = 2000;
  run_thread();
  CHECK(raw_captures > 0 && publish_count == 70);
  CHECK(device.acquisition.policy.interval_ms > 1000 &&
        device.acquisition.policy.interval_ms <= 5000);

  reset_device(LINK_FAILURE);
  CHECK(ssf_mems_acquisition_set_poll_setting(
      &device, SSF_MEMS_POLL_SETTING_FIXED_INTERVAL, 1700) == 0);
  fail_reads = true;
  run_thread();
  CHECK(get_count == 2 && device.acquisition.policy.interval_ms == 1700);

  reset_device(POLL_SWITCH);
  run_thread();
  CHECK(mode_switch_step == 3 && publish_count == 45);
  CHECK(device.acquisition.policy.config.mode == SSF_MEMS_POLL_FIXED);
  for (i = 41; i < read_count; i++)
    CHECK(read_started_at[i] - read_started_at[i - 1] == 2500);
}

static void test_settings(void) {
  u16 value;

  /* 最后一个线程场景可能已停止，单独初始化配置访问状态。 */
  memset(&device, 0, sizeof(device));
  device.serdev = &serial;
  device.sensor = &ssf_mems_xyzs;
  device.sensor_ops = ssf_mems_xyzs.ops;
  device.protocol.host_baudrate = 115200;
  CHECK(ssf_mems_acquisition_init(&device) == 0);
  sensor_sampling_index = 6;
  setting_calls = 0;
  setting_error = 0;
  CHECK(ssf_mems_acquisition_read_setting(&device, SSF_MEMS_SETTING_SAMPLING_RATE,
                                          &value) == -ENODATA);
  CHECK(ssf_mems_acquisition_write_setting(&device, SSF_MEMS_SETTING_SAMPLING_RATE,
                                           6) == -ENODATA);
  CHECK(setting_calls == 0);
  device.acquisition.online = true;
  CHECK(ssf_mems_acquisition_read_setting(&device, SSF_MEMS_SETTING_SAMPLING_RATE,
                                          &value) == 0 && value == 6);
  device.acquisition.have_sample = device.protocol.valid = true;
  device.acquisition.policy.interval_ms = 1000;
  CHECK(ssf_mems_acquisition_write_setting(&device, SSF_MEMS_SETTING_SAMPLING_RATE,
                                           6) == 0);
  CHECK(!device.protocol.valid && !device.acquisition.have_sample);
  CHECK(device.acquisition.policy.interval_ms == 1000 && device.acquisition.online);
  setting_error = -ETIMEDOUT;
  device.acquisition.have_sample = device.protocol.valid = true;
  CHECK(ssf_mems_acquisition_write_setting(&device, SSF_MEMS_SETTING_SAMPLING_RATE,
                                           6) == -ETIMEDOUT);
  CHECK(!device.protocol.valid && !device.acquisition.have_sample);
  device.acquisition.stopping = true;
  setting_calls = 0;
  CHECK(ssf_mems_acquisition_read_setting(&device, SSF_MEMS_SETTING_SAMPLING_RATE,
                                          &value) == -ENODEV);
  CHECK(ssf_mems_acquisition_write_setting(&device, SSF_MEMS_SETTING_SAMPLING_RATE,
                                           6) == -ENODEV);
  CHECK(setting_calls == 0);
}

/* 时段属性缺失、读失败或非法时均回退；0 仅允许用于关闭原始采集。 */
static void test_capture_properties(void) {
  reset_device(MANUAL);
  serial.dev.has_raw_duration = serial.dev.has_feature_duration = true;
  serial.dev.raw_duration_ms = 2000;
  serial.dev.feature_duration_ms = 3000;
  CHECK(ssf_mems_acquisition_init(&device) == 0);
  CHECK(device.acquisition.raw_duration_ms == 2000 &&
        device.acquisition.feature_duration_ms == 3000);
  serial.dev.raw_duration_ms = serial.dev.feature_duration_ms = 0;
  CHECK(ssf_mems_acquisition_init(&device) == 0);
  CHECK(device.acquisition.raw_duration_ms == 5000 &&
        device.acquisition.feature_duration_ms == 5000 &&
        device.acquisition.mode == SSF_MEMS_ACQ_MODE_FEATURE);
  serial.dev.raw_duration_ms = 99;
  serial.dev.feature_duration_ms = 60001;
  CHECK(ssf_mems_acquisition_init(&device) == 0);
  CHECK(device.acquisition.raw_duration_ms == 5000 &&
        device.acquisition.feature_duration_ms == 5000);
  serial.dev.raw_duration_ms = 1000;
  serial.dev.feature_duration_ms = 2000;
  serial.dev.duration_read_error = -EIO;
  CHECK(ssf_mems_acquisition_init(&device) == 0);
  CHECK(device.acquisition.raw_duration_ms == 5000 &&
        device.acquisition.feature_duration_ms == 5000);
  serial.dev.duration_read_error = 0;
  device.protocol.parity = SSF_MEMS_PARITY_EVEN;
  CHECK(ssf_mems_acquisition_init(&device) == 0);
  CHECK(device.acquisition.mode == SSF_MEMS_ACQ_MODE_FEATURE);
}

/* 主机配置可在离线时修改；非法值、同值写和关闭不能额外触发串口请求。 */
static void test_runtime_controls(void) {
  struct ssf_mems_acquisition_config config;
  unsigned int wakes;

  reset_device(MANUAL);
  CHECK(ssf_mems_acquisition_get_config(&device, &config) == 0);
  CHECK(config.mode == SSF_MEMS_ACQ_MODE_FEATURE && config.raw_duration_ms == 5000);
  CHECK(ssf_mems_acquisition_get_config(NULL, &config) == -EINVAL);
  CHECK(ssf_mems_acquisition_get_config(&device, NULL) == -EINVAL);
  CHECK(ssf_mems_acquisition_set_mode(NULL, SSF_MEMS_ACQ_MODE_RAW) == -EINVAL);
  CHECK(ssf_mems_acquisition_set_mode(&device, SSF_MEMS_ACQ_MODE_COUNT) == -EINVAL);
  CHECK(ssf_mems_acquisition_set_mode(&device, -1) == -EINVAL);
  CHECK(ssf_mems_acquisition_set_duration(NULL, true, 5000) == -EINVAL);
  CHECK(ssf_mems_acquisition_set_duration(&device, true, 0) == -EINVAL);
  CHECK(ssf_mems_acquisition_set_duration(&device, true, 99) == -EINVAL);
  CHECK(ssf_mems_acquisition_set_duration(&device, false, 60001) == -EINVAL);
  CHECK(ssf_mems_acquisition_set_duration(&device, false, UINT_MAX) == -EINVAL);
  wakes = test_wake_count;
  CHECK(ssf_mems_acquisition_set_mode(&device, SSF_MEMS_ACQ_MODE_FEATURE) == 0);
  CHECK(ssf_mems_acquisition_set_duration(&device, true, 5000) == 0);
  CHECK(test_wake_count == wakes && !device.acquisition.config_changed);
  device.protocol.valid = device.acquisition.have_sample = true;
  CHECK(ssf_mems_acquisition_set_mode(&device, SSF_MEMS_ACQ_MODE_RAW) == 0);
  CHECK(device.acquisition.config_changed && !device.acquisition.reconnect);
  CHECK(!device.protocol.valid && !device.acquisition.have_sample);
  CHECK(ssf_mems_acquisition_set_duration(&device, true, 100) == 0);
  CHECK(ssf_mems_acquisition_set_duration(&device, false, 60000) == 0);
  CHECK(ssf_mems_acquisition_get_config(&device, &config) == 0);
  CHECK(config.raw_duration_ms == 100 && config.feature_duration_ms == 60000);
  device.protocol.parity = SSF_MEMS_PARITY_EVEN;
  CHECK(ssf_mems_acquisition_set_mode(&device, SSF_MEMS_ACQ_MODE_ALTERNATING) == -EOPNOTSUPP);
  CHECK(ssf_mems_acquisition_set_mode(&device, SSF_MEMS_ACQ_MODE_FEATURE) == 0);
  device.acquisition.stopping = true;
  CHECK(ssf_mems_acquisition_get_config(&device, &config) == -ENODEV);
  CHECK(ssf_mems_acquisition_set_mode(&device, SSF_MEMS_ACQ_MODE_FEATURE) == -ENODEV);
  CHECK(ssf_mems_acquisition_set_duration(&device, true, 5000) == -ENODEV);
  CHECK(get_count == 0 && set_count == 0 && read_count == 0 && setting_calls == 0);
}

/* 初次匹配保留启动提示，工作目标取自型号表，不随扫描结果改变。 */
static void test_startup_baudrate_target(void) {
  static const enum ssf_mems_baudrate hints[] = {
      SSF_MEMS_BAUDRATE_DEFAULT, SSF_MEMS_BAUDRATE_57600,
      SSF_MEMS_BAUDRATE_128000,
  };
  unsigned int i;

  for (i = 0; i < ARRAY_SIZE(hints); i++) {
    reset_device(MANUAL);
    device.protocol.baudrate = hints[i];
    device.protocol.host_baudrate = ssf_mems_baudrate_to_value(&ssf_mems_xyzs, hints[i]);
    CHECK(ssf_mems_acquisition_init(&device) == 0);
    device.acquisition.mode = SSF_MEMS_ACQ_MODE_FEATURE;
    CHECK(device.acquisition.target_baudrate == SSF_MEMS_BAUDRATE_128000);
    run_thread();
    CHECK(first_match_baudrate == (unsigned int)ssf_mems_baudrate_to_value(&ssf_mems_xyzs, hints[i]));
    CHECK(sensor_baudrate == SSF_MEMS_BAUDRATE_128000 && set_count == 1);
    CHECK(device.acquisition.target_baudrate == SSF_MEMS_BAUDRATE_128000);
  }
  reset_device(MANUAL);
  device.protocol.baudrate = SSF_MEMS_BAUDRATE_128000;
  device.protocol.host_baudrate = 128000;
  sensor_baudrate = SSF_MEMS_BAUDRATE_128000;
  CHECK(ssf_mems_acquisition_init(&device) == 0);
  device.acquisition.mode = SSF_MEMS_ACQ_MODE_FEATURE;
  run_thread();
  CHECK(first_match_baudrate == 128000 && set_count == 0);
  reset_device(NO_SENSOR);
  device.protocol.baudrate = SSF_MEMS_BAUDRATE_57600;
  device.protocol.host_baudrate = 57600;
  CHECK(ssf_mems_acquisition_init(&device) == 0);
  get_error = -ETIMEDOUT;
  run_thread();
  CHECK(first_match_baudrate == 57600 && set_count == 0 && publish_count == 0);
  CHECK(device.acquisition.target_baudrate == SSF_MEMS_BAUDRATE_128000);
}

/* 手动窗口与自动 raw 调度隔离，长度配置和错误返回不能越过门禁。 */
static void test_manual_raw_window(void) {
  struct ssf_mems_acquisition_config config;
  reset_device(MANUAL);
  CHECK(ssf_mems_acquisition_set_raw_samples(&device, 2048) == 0);
  CHECK(ssf_mems_acquisition_set_raw_samples(&device, 1000) == -EINVAL);
  CHECK(ssf_mems_acquisition_set_raw_samples(&device, 65536) == -EINVAL);
  CHECK(ssf_mems_acquisition_set_raw_manual(&device, true) == 0);
  CHECK(ssf_mems_acquisition_get_config(&device, &config) == 0);
  CHECK(config.raw_batch_samples == 2048 && config.raw_manual);
  CHECK(ssf_mems_acquisition_capture_window(&device) == -EINVAL);
  CHECK(ssf_mems_acquisition_set_mode(&device, SSF_MEMS_ACQ_MODE_RAW) == 0);
  CHECK(ssf_mems_acquisition_capture_window(&device) == -ENODATA);
  device.acquisition.online = true;
  device.acquisition.phase = SSF_MEMS_ACQ_POLL;
  CHECK(ssf_mems_acquisition_capture_window(&device) == 0);
  CHECK(raw_captures == 1 && device.acquisition.phase == SSF_MEMS_ACQ_POLL);
  device.acquisition.firmware_blocked = true;
  CHECK(ssf_mems_acquisition_capture_window(&device) == -EBUSY);
  device.acquisition.stopping = true;
  CHECK(ssf_mems_acquisition_capture_window(&device) == -ENODEV);
  CHECK(ssf_mems_acquisition_set_raw_samples(&device, 1024) == -ENODEV);
  CHECK(ssf_mems_acquisition_set_raw_manual(&device, false) == -ENODEV);
  reset_device(MANUAL_RAW_IDLE);
  CHECK(ssf_mems_acquisition_set_mode(&device, SSF_MEMS_ACQ_MODE_RAW) == 0);
  CHECK(ssf_mems_acquisition_set_raw_manual(&device, true) == 0);
  run_thread();
  CHECK(raw_captures == 0);

}

static void test_sound_gate(void) {
  reset_device(MANUAL);
  CHECK(ssf_mems_acquisition_capture_sound(NULL) == -EINVAL);
  CHECK(ssf_mems_acquisition_capture_sound(&device) == -ENODATA);
  device.acquisition.online = true;
  device.acquisition.phase = SSF_MEMS_ACQ_POLL;
  device.protocol.mode = SSF_MEMS_LINK_MODBUS;
  enum ssf_mems_acquisition_mode original_mode = device.acquisition.mode;
  unsigned int original_raw_captures = raw_captures;
  CHECK(ssf_mems_acquisition_capture_sound(&device) == 0);
  CHECK(raw_captures == original_raw_captures + 1 && device.acquisition.mode == original_mode);
  CHECK(device.acquisition.raw_batch_samples == 1024);
  setting_error = -ETIMEDOUT;
  CHECK(ssf_mems_acquisition_capture_sound(&device) == -ETIMEDOUT);
  device.acquisition.firmware_blocked = true;
  CHECK(ssf_mems_acquisition_capture_sound(&device) == -EBUSY);
  device.acquisition.stopping = true;
  CHECK(ssf_mems_acquisition_capture_sound(&device) == -ENODEV);
  struct ssf_mems_sensor_ops unsupported = *device.sensor_ops;
  unsupported.collect_sound = NULL;
  device.sensor_ops = &unsupported;
  CHECK(ssf_mems_acquisition_capture_sound(&device) == -EOPNOTSUPP);
}

int main(void) {
  test_manual_raw_window();
  test_sound_gate();
  enum ssf_mems_baudrate baud;
  u16 value;

  test_policy();
  test_timed_policy();
  test_poll_configuration();
  test_poll_scheduling();
  test_thread_scenarios();
  test_settings();
  reset_device(CHANGING);
  device.acquisition.mode = SSF_MEMS_ACQ_MODE_ALTERNATING;
  device.acquisition.raw_duration_ms = 100;
  device.acquisition.feature_duration_ms = 2000;
  run_thread();
  CHECK(raw_captures > 10 && publish_count == 70);
  CHECK(get_count == 1 && set_count == 1); /* 正常交替不重新保存波特率。 */
  reset_device(RAW_RECOVERY_FAILURE);
  device.acquisition.mode = SSF_MEMS_ACQ_MODE_ALTERNATING;
  device.acquisition.raw_duration_ms = 100;
  device.acquisition.feature_duration_ms = 2000;
  run_thread();
  CHECK(raw_captures == 1 && backoff_count == 1 && get_count == 2);
  CHECK(set_count == 1); /* 恢复状态不会被当作普通波特率变化重复保存。 */
  test_capture_properties();
  test_runtime_controls();
  test_startup_baudrate_target();
  reset_device(RAW_ONLY);
  device.acquisition.mode = SSF_MEMS_ACQ_MODE_RAW;
  run_thread();
  CHECK(raw_captures == 3 && read_count == 0 && publish_count == 0);
  CHECK(get_count == 1 && set_count == 1);
  reset_device(MODE_SWITCH);
  run_thread();
  CHECK(mode_switch_step == 3 && get_count == 1 && set_count == 1);
  reset_device(RAW_ONLY_RECOVERY);
  device.acquisition.mode = SSF_MEMS_ACQ_MODE_RAW;
  run_thread();
  CHECK(raw_captures == 3 && backoff_count == 1 && get_count == 2);
  CHECK(set_count == 1 && read_count == 0 && publish_count == 0);
  reset_device(RAW_ONLY_DROPS);
  device.acquisition.mode = SSF_MEMS_ACQ_MODE_RAW;
  run_thread();
  CHECK(raw_captures == 3 && backoff_count == 0 && get_count == 1 && set_count == 1);
  reset_device(RAW_UNSUPPORTED);
  device.acquisition.mode = SSF_MEMS_ACQ_MODE_ALTERNATING;
  run_thread();
  CHECK(raw_captures == 3 && publish_count > 3 && backoff_count == 0);
  CHECK(get_count == 1 && set_count == 1);
  reset_device(RAW_ONLY_UNSUPPORTED);
  device.acquisition.mode = SSF_MEMS_ACQ_MODE_RAW;
  run_thread();
  CHECK(raw_captures == 3 && publish_count == 0 && backoff_count == 0);
  CHECK(get_count == 1 && set_count == 1);
  reset_device(SHORT_WINDOWS);
  device.acquisition.mode = SSF_MEMS_ACQ_MODE_ALTERNATING;
  device.acquisition.raw_duration_ms = device.acquisition.feature_duration_ms = 100;
  run_thread();
  CHECK(raw_started_at[0] - read_started_at[0] == 105);
  for (unsigned int i = 1; i < raw_captures; i++)
    CHECK(raw_started_at[i] - raw_started_at[i - 1] == 200);
  reset_device(OTA_BLOCKED);
  device.acquisition.firmware_blocked = true;
  device.acquisition.online = false;
  CHECK(ssf_mems_acquisition_get_baudrate(&device, &baud) == -EBUSY);
  CHECK(ssf_mems_acquisition_set_baudrate(&device, SSF_MEMS_BAUDRATE_9600) == -EBUSY);
  CHECK(ssf_mems_acquisition_read_setting(&device, SSF_MEMS_SETTING_SAMPLING_RATE,
                                          &value) == -EBUSY);
  CHECK(ssf_mems_acquisition_write_setting(&device, SSF_MEMS_SETTING_SAMPLING_RATE,
                                           0) == -EBUSY);
  run_thread();
  CHECK(get_count == 0 && set_count == 0 && read_count == 0 && raw_captures == 0);
  printf("PASS acquisition policy and lifecycle: %u checks\n", checks);
  return 0;
}

int ssf_mems_sound_collect_finite(struct ssf_mems_data *data) {
  CHECK(data == &device && data->acquisition.phase == SSF_MEMS_ACQ_RAW);
  CHECK(data->sensor == &ssf_mems_xyzs);
  raw_captures++;
  return setting_error;
}
