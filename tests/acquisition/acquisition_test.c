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
  RAW_ONLY_DROPS, SHORT_WINDOWS
};
static enum scenario scenario;
static unsigned int mode_switch_step; /* 记录运行中模式切换的测试阶段。 */
static unsigned long raw_started_at[80]; /* 检查短窗口不会被 1 s 轮询间隔拖长。 */

/* 停止接口无论线程在哪个阶段退出，都必须检查协议层残留的私有流。 */
int ssf_mems_protocol_recover_raw(struct ssf_mems_xyzs_data *data) {
  CHECK(data == &device && data->acquisition.stopping);
  CHECK(data->acquisition.thread == NULL);
  data->protocol.mode = SSF_MEMS_LINK_MODBUS;
  return 0;
}

/* 原始模式由协议模块测试；这里核对采集状态机的交替顺序和计时。 */
int ssf_mems_protocol_collect_raw(struct ssf_mems_xyzs_data *data,
                                 unsigned int duration_ms) {
  CHECK(data == &device && duration_ms == device.acquisition.raw_duration_ms);
  CHECK(data->acquisition.phase == SSF_MEMS_ACQ_RAW);
  CHECK(raw_captures < ARRAY_SIZE(raw_started_at));
  raw_started_at[raw_captures++] = jiffies;
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
  CHECK(value <= SSF_MEMS_SAMPLING_RATE_MAX_INDEX);
  CHECK(timeout_ms > 200);
  setting_calls++;
  if (device.acquisition.phase == SSF_MEMS_ACQ_CONFIGURE) {
    /* 自动配置必须在匹配/切换波特率后、读取和发布样本之前进行。 */
    CHECK(get_count > 0 && !device.acquisition.online);
    CHECK(value == 6);
    CHECK(device.protocol.host_baudrate ==
          (unsigned int)ssf_mems_baudrate_to_value(device.acquisition.target_baudrate));
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

int ssf_mems_baudrate_to_value(enum ssf_mems_baudrate baudrate) {
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
  CHECK(device.protocol.mode == SSF_MEMS_LINK_MODBUS);
  CHECK(device.protocol.host_baudrate ==
        (unsigned int)ssf_mems_baudrate_to_value(sensor_baudrate));
  CHECK(timeout_ms > 200);
  if (sensor_baudrate == SSF_MEMS_BAUDRATE_2400)
    CHECK(timeout_ms > 1000);
  CHECK(read_count < ARRAY_SIZE(read_started_at));
  read_started_at[read_count++] = jiffies;
  CHECK(sampling_config_calls > 0 &&
        sensor_sampling_index == 6);
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
      CHECK(scenario == LINK_FAILURE && failure_count >= 5);
      CHECK(publish_count == 1);
      /* 模拟传感器掉电，启动后回到默认速率。 */
      sensor_baudrate = SSF_MEMS_BAUDRATE_9600;
      sensor_sampling_index = 0;
      fail_reads = false;
    }
  } else if (device.acquisition.phase == SSF_MEMS_ACQ_POLL) {
    CHECK(timeout >= 2);
    if (scenario == RAW_ONLY || scenario == RAW_ONLY_RECOVERY || scenario == RAW_ONLY_DROPS) {
      CHECK(device.acquisition.online && !device.protocol.valid);
      CHECK(publish_count == 0 && read_count == 0);
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
    } else if (scenario == CHANGING || scenario == CONSTANT) {
      CHECK(device.acquisition.online && device.protocol.valid);
      if (publish_count >= 70)
        stop_requested = true;
    } else if (scenario == RAW_RECOVERY_FAILURE && get_count == 2) {
      CHECK(raw_captures == 1 && device.acquisition.online);
      CHECK(device.protocol.mode == SSF_MEMS_LINK_MODBUS);
      stop_requested = true;
    } else if (scenario == LINK_FAILURE && publish_count == 2) {
      CHECK(get_count == 2 && set_count == 2);
      CHECK(device.acquisition.policy.interval_ms == 1000);
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
  CHECK(published.x.acc_rms_x100 == read_count);
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
  CHECK(policy.interval_ms == 1000 && policy.comparisons == 0);
  ssf_mems_acquisition_policy_init(&policy, 0);
  window(&policy, &sample, 0, 5);
  CHECK(policy.interval_ms == 1000);
  ssf_mems_acquisition_policy_init(&policy, 0);
  window(&policy, &sample, 16, 5);
  CHECK(policy.interval_ms == 1000);
  ssf_mems_acquisition_policy_init(&policy, 0);
  window(&policy, &sample, 0, 501);
  CHECK(policy.interval_ms == 1000);

  ssf_mems_acquisition_policy_init(&policy, 0);
  policy.interval_ms = SSF_MEMS_POLL_MAX_MS;
  window(&policy, &sample, 32, 5);
  CHECK(policy.interval_ms == 1000);
  ssf_mems_acquisition_policy_init(&policy, 0);
  policy.interval_ms = SSF_MEMS_POLL_MIN_MS;
  window(&policy, &sample, 0, 5);
  CHECK(policy.interval_ms == 1000);

  ssf_mems_acquisition_policy_init(&policy, 0);
  policy.interval_ms = 1000;
  ssf_mems_acquisition_policy_sample(&policy, &sample, 5, 30000);
  CHECK(policy.interval_ms == 1000 && policy.next_probe == 60000);

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

static void test_settings(void) {
  u16 value;

  /* 最后一个线程场景可能已停止，单独初始化配置访问状态。 */
  memset(&device, 0, sizeof(device));
  device.serdev = &serial;
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

/* 初次匹配保留启动提示，正式运行始终使用 128000，不随扫描结果改变。 */
static void test_startup_baudrate_target(void) {
  static const enum ssf_mems_baudrate hints[] = {
      SSF_MEMS_BAUDRATE_DEFAULT, SSF_MEMS_BAUDRATE_57600,
      SSF_MEMS_BAUDRATE_128000,
  };
  unsigned int i;

  for (i = 0; i < ARRAY_SIZE(hints); i++) {
    reset_device(MANUAL);
    device.protocol.baudrate = hints[i];
    device.protocol.host_baudrate = ssf_mems_baudrate_to_value(hints[i]);
    CHECK(ssf_mems_acquisition_init(&device) == 0);
    device.acquisition.mode = SSF_MEMS_ACQ_MODE_FEATURE;
    CHECK(device.acquisition.target_baudrate == SSF_MEMS_BAUDRATE_128000);
    run_thread();
    CHECK(first_match_baudrate == (unsigned int)ssf_mems_baudrate_to_value(hints[i]));
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

int main(void) {
  test_policy();
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
  reset_device(SHORT_WINDOWS);
  device.acquisition.mode = SSF_MEMS_ACQ_MODE_ALTERNATING;
  device.acquisition.raw_duration_ms = device.acquisition.feature_duration_ms = 100;
  run_thread();
  CHECK(raw_started_at[0] - read_started_at[0] == 105);
  for (unsigned int i = 1; i < raw_captures; i++)
    CHECK(raw_started_at[i] - raw_started_at[i - 1] == 200);
  printf("PASS acquisition policy and lifecycle: %u checks\n", checks);
  return 0;
}
