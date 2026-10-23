#include <assert.h>
#include <errno.h>
#include <sched.h>
#include <stdarg.h>

/* Compile the complete production source; the scan packing is not copied. */
#include "../../iio/ssf_mems_iio.c"

static struct ssf_mems_data sensor;
static struct iio_dev indio;
static struct iio_dev raw_indio;
static struct iio_dev *xyz_feature_dev;
static struct ssf_mems_raw_sample raw_sample = {
    .xyz = {-2, -32768, 5399}, .packet_sequence = 0x12345678, .sample_index = 63,
};
/* 独立夹具：每个通道数值等于 scan_index+1，温度和运行位单独指定。 */
static struct ssf_mems_sensor_data sample = {
    .xyzs = {
    .x = {1, 2, 8, 11, 14, 17, 20},
    .y = {3, 4, 9, 12, 15, 18, 21},
    .z = {5, 6, 10, 13, 16, 19, 22},
    .temperature_x100 = -321,
    .sound = {23, 24, 25},
    .zero_crossing_rate_percent = 26,
    .spectral_centroid_hz_x10 = 27,
    .spectral_flux_x100 = 28,
    .startup_flags = 7,
    },
};
static pthread_barrier_t start;
static unsigned int pushes;
static const s64 timestamp = 123456789;
static int setting_error;
static u16 setting_value;
static enum ssf_mems_sensor_setting last_setting;
static struct ssf_mems_acquisition_config acquisition_config; /* 模拟采集层配置。 */
static int control_error; /* 模拟配置层关闭或不支持的错误。 */
static unsigned int control_writes; /* 检查非法 sysfs 输入不会进入配置层。 */
static unsigned int raw_capture_calls; /* 每次显式触发只调用一次采集层。 */
static enum ssf_mems_poll_setting last_poll_setting;
static unsigned int last_poll_value;
static unsigned int firmware_updates; /* 检查升级属性仅转交合法文件名。 */

/* 只模拟资源分配和注册结果，执行生产注册/撤销流程。 */
static struct iio_dev *allocated_devices[2];
static size_t allocated_count;
static unsigned int allocation_calls, buffer_calls, register_calls;
static unsigned int live_buffers, registered_devices;
static unsigned int fail_allocation, fail_buffer, fail_register;

struct iio_dev *devm_iio_device_alloc(struct device *dev, int private_size) {
  struct iio_dev *allocated;

  if (++allocation_calls == fail_allocation)
    return NULL;
  assert(allocated_count < ARRAY_SIZE(allocated_devices));
  allocated = calloc(1, sizeof(*allocated));
  assert(allocated);
  allocated->private = calloc(1, private_size);
  assert(allocated->private);
  mutex_init(&allocated->mlock);
  allocated_devices[allocated_count++] = allocated;
  return allocated;
}

struct iio_buffer *iio_kfifo_allocate(void) {
  struct iio_buffer *buffer;

  if (++buffer_calls == fail_buffer)
    return NULL;
  buffer = calloc(1, sizeof(*buffer));
  assert(buffer);
  live_buffers++;
  return buffer;
}

void iio_kfifo_free(struct iio_buffer *buffer) {
  assert(buffer && live_buffers > 0);
  live_buffers--;
  free(buffer);
}

void iio_device_attach_buffer(struct iio_dev *dev, struct iio_buffer *buffer) {
  assert(dev->buffer == NULL);
  dev->buffer = buffer;
}

int iio_device_register(struct iio_dev *dev) {
  assert(dev->buffer && dev->channels && dev->info);
  assert(dev->modes == (INDIO_DIRECT_MODE | INDIO_BUFFER_SOFTWARE));
  if (++register_calls == fail_register)
    return -EIO;
  registered_devices++;
  return 0;
}

void iio_device_unregister(struct iio_dev *dev) {
  assert(dev && registered_devices > 0);
  registered_devices--;
}

static void release_test_devices(void) {
  while (allocated_count != 0) {
    struct iio_dev *dev = allocated_devices[--allocated_count];
    pthread_mutex_destroy(&dev->mlock.native);
    free(dev->private);
    free(dev);
  }
}

/* 验证两设备注册，以及每一步失败后的撤销和重复注册保护。 */
static void test_registration(void) {
  struct serdev_device serdev = {0};
  struct ssf_mems_data data = {.serdev = &serdev};
  unsigned int scenario;

  assert(ssf_mems_iio_register(NULL) == -EINVAL);
  data.serdev = NULL;
  data.sensor = &ssf_mems_xyzs;
  data.sensor_ops = ssf_mems_xyzs.ops;
  assert(ssf_mems_iio_register(&data) == -EINVAL);
  data.serdev = &serdev;
  data.sensor = &ssf_mems_xyzs;
  data.sensor_ops = ssf_mems_xyzs.ops;
  for (scenario = 0; scenario < 7; scenario++) {
    int expected = scenario == 0 ? 0 : scenario <= 4 ? -ENOMEM : -EIO;

    allocation_calls = buffer_calls = register_calls = 0;
    fail_allocation = scenario == 1 ? 1 : scenario == 2 ? 2 : 0;
    fail_buffer = scenario == 3 ? 1 : scenario == 4 ? 2 : 0;
    fail_register = scenario == 5 ? 1 : scenario == 6 ? 2 : 0;
    assert(ssf_mems_iio_register(&data) == expected);
    if (scenario == 0) {
      assert(registered_devices == 2 && live_buffers == 2);
      assert(strcmp(data.indio_dev->name, "ssf_mems_xyzs") == 0);
      assert(strcmp(data.raw_indio_dev->name, "ssf_mems_xyzs_raw") == 0);
      assert(data.indio_dev->num_channels == 30);
      for (size_t i = 0; i < ARRAY_SIZE(ssf_mems_iio_attributes); i++)
        assert(data.indio_dev->info->attrs->attrs[i] == ssf_mems_iio_attributes[i]);
      assert(data.indio_dev->info != data.raw_indio_dev->info);
      assert(data.raw_indio_dev->channels == ssf_mems_raw_channels);
      assert(((struct ssf_mems_iio_state *)iio_priv(data.indio_dev))->data == &data);
      assert(((struct ssf_mems_iio_state *)iio_priv(data.raw_indio_dev))->data == &data);
      assert(ssf_mems_iio_register(&data) == -EBUSY);
      assert(allocation_calls == 2);
      ssf_mems_iio_unregister(&data);
      ssf_mems_iio_unregister(&data);
    }
    assert(data.indio_dev == NULL && data.raw_indio_dev == NULL);
    assert(registered_devices == 0 && live_buffers == 0);
    release_test_devices();
  }
  ssf_mems_iio_unregister(NULL);
  fail_allocation = fail_buffer = fail_register = 0;
}

int ssf_mems_protocol_store_features(struct ssf_mems_data *data,
                                   const struct ssf_mems_sensor_data *features) {
  mutex_lock(&data->protocol.lock);
  data->protocol.features = *features;
  data->protocol.valid = true;
  mutex_unlock(&data->protocol.lock);
  return 0;
}

s64 iio_get_time_ns(struct iio_dev *dev) { return timestamp; }

int iio_push_to_buffers_with_timestamp(struct iio_dev *dev, void *scan, s64 ts) {
  u8 expected[SSF_MEMS_IIO_MAX_SCAN_BYTES] = {0};
  size_t offset = 0, bit, largest = 0;

  /* The same mutex must protect enabled state, mask, size, and the final push. */
  assert(pthread_mutex_trylock(&dev->mlock.native) == EBUSY);
  assert(dev->currentmode == INDIO_BUFFER_SOFTWARE && dev->active_scan_mask);
  assert(ts == timestamp);
  if (dev == xyz_feature_dev) {
    static const s16 values[] = {-3210, 20, 80, 140, 10, 70, 130, 30, 90, 150,
                                  40, 100, 160, 50, 110, 170, 60, 120, 180};
    for (bit = 0; bit < ARRAY_SIZE(values); bit++) {
      if ((*dev->active_scan_mask & BIT(bit)) == 0)
        continue;
      memcpy(expected + offset, &values[bit], 2);
      offset += 2;
    }
    if (dev->scan_timestamp) {
      offset = ALIGN(offset, 8);
      memcpy((u8 *)scan + offset, &ts, sizeof(ts));
      memcpy(expected + offset, &ts, sizeof(ts));
      offset += sizeof(ts);
    }
    assert(dev->scan_bytes == offset);
    assert(memcmp(scan, expected, offset) == 0);
    pushes++;
    return 0;
  }
  if (dev == &raw_indio) {
    for (bit = 0; bit < 5; bit++) {
      u32 value = bit == 0 ? 0xfffe : bit == 1 ? 0x8000 :
                  bit == 2 ? 5399 : bit == 3 ? 0x12345678 : 63;
      size_t width = bit == 3 ? 4 : 2;
      if ((*dev->active_scan_mask & BIT(bit)) == 0)
        continue;
      offset = ALIGN(offset, width);
      largest = max(largest, width);
      memcpy(expected + offset, &value, width);
      offset += width;
    }
    if (dev->scan_timestamp) {
      offset = ALIGN(offset, sizeof(ts));
      memcpy((u8 *)scan + offset, &ts, sizeof(ts));
      memcpy(expected + offset, &ts, sizeof(ts));
      offset += sizeof(ts);
      largest = sizeof(ts);
    }
    offset = ALIGN(offset, largest);
    assert(offset == dev->scan_bytes && memcmp(scan, expected, offset) == 0);
    pushes++;
    return 0;
  }
  for (bit = 0; bit < SSF_MEMS_IIO_FEATURE_MAX; bit++) {
    const struct iio_chan_spec *chan = &ssf_mems_iio_channels[bit];
    if (!(*dev->active_scan_mask & BIT(bit)))
      continue;
    assert(chan->scan_index == (int)bit);
    /* Expected channel order is independent of the shared field descriptors. */
    if (bit == SSF_MEMS_IIO_STARTUP_FLAGS) {
      expected[offset++] = 7;
      largest = max(largest, (size_t)1);
    } else {
      s16 value = bit == SSF_MEMS_IIO_TEMPERATURE ? -321 : bit + 1;
      offset = ALIGN(offset, sizeof(value));
      largest = max(largest, sizeof(value));
      memcpy(expected + offset, &value, sizeof(value));
      offset += sizeof(value);
    }
  }
  if (dev->scan_timestamp) {
    offset = ALIGN(offset, sizeof(ts));
    memcpy((u8 *)scan + offset, &ts, sizeof(ts));
    memcpy(expected + offset, &ts, sizeof(ts));
    offset += sizeof(ts);
    largest = sizeof(ts);
  }
  offset = ALIGN(offset, largest);
  assert(offset == dev->scan_bytes);
  assert(memcmp(scan, expected, offset) == 0);
  pushes++;
  return 0;
}

static void configure_buffer(unsigned int variant) {
  static const unsigned long masks[] = {
      BIT(0) | BIT(28), BIT(6), (BIT(29) - 1), BIT(28),
  };
  unsigned long *mask = malloc(sizeof(*mask));
  size_t bytes;

  assert(mask);
  *mask = masks[variant % ARRAY_SIZE(masks)];
  /* 与内核一致：整条记录的末尾也按最大字段宽度对齐。 */
  bytes = variant % 4 == 0 ? 4 : variant % 4 == 1 ? 2 : variant % 4 == 2 ? 58 : 1;
  mutex_lock(&indio.mlock);
  indio.currentmode = INDIO_DIRECT_MODE;
  free(indio.active_scan_mask);
  indio.active_scan_mask = NULL;
  /* Allow the producer to run while the old mask is freed and disabled. */
  mutex_unlock(&indio.mlock);
  sched_yield();
  mutex_lock(&indio.mlock);
  indio.scan_timestamp = !!(variant & 4);
  indio.scan_bytes = indio.scan_timestamp ? ALIGN(bytes, sizeof(s64)) + sizeof(s64) : bytes;
  indio.active_scan_mask = mask;
  indio.currentmode = INDIO_BUFFER_SOFTWARE;
  mutex_unlock(&indio.mlock);
}

static void *produce(void *unused) {
  size_t i;
  pthread_barrier_wait(&start);
  for (i = 0; i < 100000; i++)
    assert(ssf_mems_iio_publish_features(&sensor, &sample, i & 1 ? timestamp : 0) == 0);
  return NULL;
}

static void *reconfigure(void *unused) {
  size_t i;
  pthread_barrier_wait(&start);
  for (i = 0; i < 20000; i++)
    configure_buffer(i);
  return NULL;
}

/* 原始扫描支持全部字段子集；逐一验证对齐、符号、元数据和可选时间戳。 */
static void test_raw_buffer(void) {
  struct ssf_mems_iio_state state = {.data = &sensor};
  raw_indio.private = &state;
  unsigned long mask;
  unsigned int with_timestamp, bit;
  int val, val2;

  mutex_init(&raw_indio.mlock);
  sensor.raw_indio_dev = &raw_indio;
  raw_indio.currentmode = INDIO_BUFFER_SOFTWARE;
  raw_indio.active_scan_mask = &mask;
  for (mask = 1; mask < BIT(5); mask++) {
    size_t bytes = 0, largest = 0;
    for (bit = 0; bit < 5; bit++) {
      size_t width = bit == 3 ? 4 : 2;
      if ((mask & BIT(bit)) != 0) {
        bytes = ALIGN(bytes, width) + width;
        largest = max(largest, width);
      }
    }
    bytes = ALIGN(bytes, largest);
    if (mask == BIT(5) - 1)
      assert(bytes == 16);
    for (with_timestamp = 0; with_timestamp < 2; with_timestamp++) {
      raw_indio.scan_timestamp = with_timestamp;
      raw_indio.scan_bytes = with_timestamp ? ALIGN(bytes, 8) + 8 : bytes;
      assert(ssf_mems_iio_publish_raw(&sensor, &raw_sample) == 0);
    }
  }
  assert(ssf_mems_raw_read_scale(&raw_indio, &ssf_mems_raw_channels[0],
                                 &val, &val2, IIO_CHAN_INFO_SCALE) == IIO_VAL_INT_PLUS_NANO);
  assert(val == 0 && val2 == 4788403);
  raw_indio.scan_bytes = 25;
  assert(ssf_mems_iio_publish_raw(&sensor, &raw_sample) == -EOVERFLOW);
  raw_indio.active_scan_mask = NULL;
  assert(ssf_mems_iio_publish_raw(&sensor, &raw_sample) == -EINVAL);
  raw_indio.currentmode = INDIO_DIRECT_MODE;
  assert(ssf_mems_iio_publish_raw(&sensor, &raw_sample) == 0);
  assert(ssf_mems_iio_publish_raw(NULL, &raw_sample) == -EINVAL);
  assert(ssf_mems_iio_publish_raw(&sensor, NULL) == -EINVAL);
  sensor.raw_indio_dev = NULL;
  assert(ssf_mems_iio_publish_raw(&sensor, &raw_sample) == -ENODEV);
}

/* These unrelated interfaces are linked because sanitizer metadata retains
 * static channel/sysfs descriptors. They are never exercised by this test. */
int ssf_mems_protocol_get_features(struct serdev_device *s,
                                    struct ssf_mems_sensor_data *out) {
  assert(s == sensor.serdev);
  mutex_lock(&sensor.protocol.lock);
  if (!sensor.protocol.valid) {
    mutex_unlock(&sensor.protocol.lock);
    return -ENODATA;
  }
  *out = sensor.protocol.features;
  mutex_unlock(&sensor.protocol.lock);
  return 0;
}
int ssf_mems_acquisition_get_baudrate(struct ssf_mems_data *d, enum ssf_mems_baudrate *b) { abort(); }
int ssf_mems_acquisition_set_baudrate(struct ssf_mems_data *d, enum ssf_mems_baudrate b) { abort(); }
int ssf_mems_acquisition_read_setting(struct ssf_mems_data *d,
                                    enum ssf_mems_sensor_setting s, u16 *v) {
  assert(d == &sensor);
  last_setting = s;
  *v = setting_value;
  return setting_error;
}
int ssf_mems_acquisition_write_setting(struct ssf_mems_data *d,
                                     enum ssf_mems_sensor_setting s, u16 v) {
  assert(d == &sensor);
  last_setting = s;
  setting_value = v;
  return setting_error;
}
void ssf_mems_acquisition_get_status(struct ssf_mems_data *d, struct ssf_mems_acquisition_status *s) { abort(); }
/* IIO 用例只检查分派，调度和锁由 acquisition 用例验证。 */
int ssf_mems_acquisition_get_config(struct ssf_mems_data *d,
                                   struct ssf_mems_acquisition_config *c) {
  assert(d == &sensor);
  *c = acquisition_config;
  return control_error;
}
/* 模拟采集层的范围检查，验证 IIO 正确传递模式及错误。 */
int ssf_mems_acquisition_set_mode(struct ssf_mems_data *d,
                                 enum ssf_mems_acquisition_mode mode) {
  assert(d == &sensor && (unsigned int)mode < SSF_MEMS_ACQ_MODE_COUNT);
  control_writes++;
  if (control_error != 0)
    return control_error;
  acquisition_config.mode = mode;
  return 0;
}
/* 单独更新指定窗口，另一项不应被覆盖。 */
int ssf_mems_acquisition_set_duration(struct ssf_mems_data *d,
                                     bool raw_window, unsigned int duration) {
  assert(d == &sensor);
  control_writes++;
  if (control_error != 0)
    return control_error;
  if (duration < SSF_MEMS_ACQ_MIN_DURATION_MS || duration > SSF_MEMS_ACQ_MAX_DURATION_MS)
    return -EINVAL;
  if (raw_window == true)
    acquisition_config.raw_duration_ms = duration;
  else
    acquisition_config.feature_duration_ms = duration;
  return 0;
}
int ssf_mems_acquisition_set_raw_samples(struct ssf_mems_data *d, u32 samples) {
  if (!ssf_mems_sensor_raw_samples_valid(d->sensor, samples)) return -EINVAL;
  acquisition_config.raw_batch_samples = samples;
  return control_error;
}
int ssf_mems_acquisition_set_raw_manual(struct ssf_mems_data *d, bool manual) {
  acquisition_config.raw_manual = manual;
  return control_error;
}
int ssf_mems_acquisition_capture_window(struct ssf_mems_data *d) {
  assert(d == &sensor);
  raw_capture_calls++;
  return control_error;
}
/* 记录 IIO 分派结果；范围和上下限由真实 acquisition 用例验证。 */
int ssf_mems_acquisition_set_poll_setting(struct ssf_mems_data *d,
                                          enum ssf_mems_poll_setting setting,
                                          unsigned int value) {
  assert(d == &sensor);
  control_writes++;
  last_poll_setting = setting;
  last_poll_value = value;
  if (control_error != 0)
    return control_error;
  switch (setting) {
  case SSF_MEMS_POLL_SETTING_MODE: acquisition_config.poll.mode = value; break;
  case SSF_MEMS_POLL_SETTING_FIXED_INTERVAL:
    acquisition_config.poll.fixed_interval_ms = value; break;
  case SSF_MEMS_POLL_SETTING_MIN_INTERVAL:
    acquisition_config.poll.min_interval_ms = value; break;
  case SSF_MEMS_POLL_SETTING_MAX_INTERVAL:
    acquisition_config.poll.max_interval_ms = value; break;
  default: abort();
  }
  return 0;
}
int ssf_mems_baudrate_to_value(const struct ssf_mems_sensor *model, enum ssf_mems_baudrate b) { abort(); }
int ssf_mems_baudrate_from_value(const struct ssf_mems_sensor *model, unsigned int v, enum ssf_mems_baudrate *b) { abort(); }
int ssf_mems_acquisition_update_firmware(struct ssf_mems_data *data,
                                         const char *name) {
  assert(data == &sensor && strcmp(name, "update.bin") == 0);
  firmware_updates++;
  return control_error;
}
int sysfs_emit(char *buf, const char *fmt, ...) {
  va_list args;
  int count;
  va_start(args, fmt);
  count = vsnprintf(buf, 4096, fmt, args);
  va_end(args);
  return count;
}
int sysfs_emit_at(char *buf, int at, const char *fmt, ...) {
  va_list args;
  int count;
  assert(at >= 0 && at < 4096);
  va_start(args, fmt);
  count = vsnprintf(buf + at, 4096 - at, fmt, args);
  va_end(args);
  return count;
}
int kstrtouint(const char *buf, unsigned int base, unsigned int *value) {
  char *end;
  unsigned long parsed;
  errno = 0;
  if (*buf == '-')
    return -EINVAL;
  parsed = strtoul(buf, &end, base);
  if (end == buf || (*end && strcmp(end, "\n")) || errno || parsed > UINT32_MAX)
    return -EINVAL;
  *value = parsed;
  return 0;
}

static void test_configuration_attributes(void) {
  struct ssf_mems_iio_state state = {.data = &sensor};
  ssf_mems_iio_setup_features(&state);
  char buf[4096];
  struct device_attribute *attr;

  indio.private = &state;
  attr = &iio_dev_attr_sensor_sampling_rate_index_available.dev_attr;
  assert(attr->show(&indio.dev, attr, buf) == 20);
  assert(!strcmp(buf, "0 1 2 3 4 5 6 7 8 9\n"));
  attr = &iio_dev_attr_sensor_baudrate_available.dev_attr;
  assert(attr->show(&indio.dev, attr, buf) > 0);
  assert(!strcmp(buf, "2400 4800 9600 19200 38400 57600 115200 128000 230400 256000 "
                      "460800 500000 512000 600000 750000 921600 1000000\n"));
  setting_value = 6;
  attr = &iio_dev_attr_sensor_sampling_frequency.dev_attr;
  assert(attr->show(&indio.dev, attr, buf) == 7 && !strcmp(buf, "5333.4\n"));
  assert(last_setting == SSF_MEMS_SETTING_SAMPLING_RATE);
  setting_value = 10;
  assert(attr->show(&indio.dev, attr, buf) == -EINVAL);
  setting_error = -ENODATA;
  assert(attr->show(&indio.dev, attr, buf) == -ENODATA);
  setting_error = 0;
  setting_value = 0xfffd;
  attr = &iio_dev_attr_sensor_parameter_switch.dev_attr;
  assert(attr->show(&indio.dev, attr, buf) == 7 && !strcmp(buf, "0xfffd\n"));
  assert(last_setting == SSF_MEMS_SETTING_PARAMETER_SWITCH);
  attr = &iio_dev_attr_sensor_feature_enable.dev_attr;
  assert(attr->store(&indio.dev, attr, "63\n", 3) == 3);
  assert(last_setting == SSF_MEMS_SETTING_FEATURE_ENABLE && setting_value == 63);
  assert(attr->store(&indio.dev, attr, "65536\n", 6) == -EINVAL && setting_value == 63);
  assert(attr->store(&indio.dev, attr, "invalid\n", 8) == -EINVAL);
  attr = &iio_dev_attr_sensor_sampling_rate_index.dev_attr;
  assert(attr->store(&indio.dev, attr, "6\n", 2) == 2);
  assert(last_setting == SSF_MEMS_SETTING_SAMPLING_RATE && setting_value == 6);
  attr = &iio_dev_attr_sensor_sampling_length_index.dev_attr;
  assert(attr->store(&indio.dev, attr, "2\n", 2) == 2);
  assert(last_setting == SSF_MEMS_SETTING_SAMPLING_LENGTH && setting_value == 2);
  assert(attr->show(&indio.dev, attr, buf) == 2 && !strcmp(buf, "2\n"));
  setting_error = -EIO;
  assert(attr->store(&indio.dev, attr, "6\n", 2) == -EIO);
  setting_error = 0;
  indio.private = NULL;
}

/* 新统计属性均只读；有效采集、排空和业务模式分别输出，不能串错成员。 */
/* 覆盖模式字符串、两项时长、非法输入和关闭时错误的 sysfs 透传。 */
static void test_acquisition_controls(void) {
  struct ssf_mems_iio_state state = {.data = &sensor};
  ssf_mems_iio_setup_features(&state);
  struct device_attribute *mode = &iio_dev_attr_sensor_acquisition_mode.dev_attr;
  struct device_attribute *raw = &iio_dev_attr_sensor_raw_duration_ms.dev_attr;
  struct device_attribute *feature = &iio_dev_attr_sensor_feature_duration_ms.dev_attr;
  char buf[4096];
  unsigned int before;

  indio.private = &state;
  acquisition_config.mode = SSF_MEMS_ACQ_MODE_ALTERNATING;
  acquisition_config.raw_duration_ms = acquisition_config.feature_duration_ms = 5000;
  assert(mode->show(&indio.dev, mode, buf) == 12 && !strcmp(buf, "alternating\n"));
  assert(mode->store(&indio.dev, mode, "raw\n", 4) == 4);
  assert(acquisition_config.mode == SSF_MEMS_ACQ_MODE_RAW);
  assert(mode->store(&indio.dev, mode, "feature", 7) == 7);
  assert(acquisition_config.mode == SSF_MEMS_ACQ_MODE_FEATURE);
  assert(mode->store(&indio.dev, mode, "alternating\n", 12) == 12);
  before = control_writes;
  assert(mode->store(&indio.dev, mode, "RAW\n", 4) == -EINVAL);
  assert(mode->store(&indio.dev, mode, "raw extra\n", 10) == -EINVAL);
  assert(mode->store(&indio.dev, mode, "", 0) == -EINVAL);
  assert(control_writes == before);
  assert(raw->store(&indio.dev, raw, "2000\n", 5) == 5);
  assert(feature->store(&indio.dev, feature, "3000\n", 5) == 5);
  assert(acquisition_config.raw_duration_ms == 2000 && acquisition_config.feature_duration_ms == 3000);
  assert(raw->show(&indio.dev, raw, buf) == 5 && !strcmp(buf, "2000\n"));
  assert(feature->show(&indio.dev, feature, buf) == 5 && !strcmp(buf, "3000\n"));
  assert(raw->store(&indio.dev, raw, "99", 2) == -EINVAL);
  assert(feature->store(&indio.dev, feature, "60001", 5) == -EINVAL);
  before = control_writes;
  assert(raw->store(&indio.dev, raw, "-1", 2) == -EINVAL);
  assert(raw->store(&indio.dev, raw, "invalid", 7) == -EINVAL);
  assert(raw->store(&indio.dev, raw, "0x100", 5) == -EINVAL);
  assert(raw->store(&indio.dev, raw, "4294967296", 10) < 0);
  assert(control_writes == before);
  assert(acquisition_config.raw_duration_ms == 2000 && acquisition_config.feature_duration_ms == 3000);
  control_error = -ENODEV;
  assert(mode->show(&indio.dev, mode, buf) == -ENODEV);
  assert(mode->store(&indio.dev, mode, "raw", 3) == -ENODEV);
  assert(raw->store(&indio.dev, raw, "1000", 4) == -ENODEV);
  control_error = -EOPNOTSUPP;
  assert(mode->store(&indio.dev, mode, "raw", 3) == -EOPNOTSUPP);
  control_error = 0;
  indio.private = NULL;
}

/* 升级期间进度不依赖采集锁；非法名字不触发文件加载或传感器操作。 */
/* 轮询属性正确分派，非法文本不进入配置层，状态周期仍只读。 */
static void test_poll_controls(void) {
  struct ssf_mems_iio_state state = {.data = &sensor};
  ssf_mems_iio_setup_features(&state);
  struct device_attribute *mode = &iio_dev_attr_sensor_poll_mode.dev_attr;
  struct device_attribute *attrs[] = {
      &iio_dev_attr_sensor_poll_fixed_interval_ms.dev_attr,
      &iio_dev_attr_sensor_poll_min_interval_ms.dev_attr,
      &iio_dev_attr_sensor_poll_max_interval_ms.dev_attr,
  };
  const enum ssf_mems_poll_setting settings[] = {
      SSF_MEMS_POLL_SETTING_FIXED_INTERVAL, SSF_MEMS_POLL_SETTING_MIN_INTERVAL,
      SSF_MEMS_POLL_SETTING_MAX_INTERVAL,
  };
  char buf[4096];
  unsigned int before;
  size_t i;

  indio.private = &state;
  acquisition_config.poll = (struct ssf_mems_poll_config){
      .mode = SSF_MEMS_POLL_FIXED, .fixed_interval_ms = 1000,
      .min_interval_ms = 500, .max_interval_ms = 5000,
  };
  assert(mode->show(&indio.dev, mode, buf) == 6 && !strcmp(buf, "fixed\n"));
  assert(mode->store(&indio.dev, mode, "adaptive\n", 9) == 9);
  assert(last_poll_setting == SSF_MEMS_POLL_SETTING_MODE &&
         last_poll_value == SSF_MEMS_POLL_ADAPTIVE);
  assert(mode->show(&indio.dev, mode, buf) == 9 && !strcmp(buf, "adaptive\n"));
  assert(mode->store(&indio.dev, mode, "fixed", 5) == 5);
  assert(last_poll_value == SSF_MEMS_POLL_FIXED);
  before = control_writes;
  assert(mode->store(&indio.dev, mode, "ADAPTIVE", 8) == -EINVAL);
  assert(mode->store(&indio.dev, mode, "fixed extra", 11) == -EINVAL);
  assert(mode->store(&indio.dev, mode, "1", 1) == -EINVAL);
  assert(mode->store(&indio.dev, mode, "", 0) == -EINVAL);
  assert(control_writes == before);
  for (i = 0; i < ARRAY_SIZE(attrs); i++) {
    assert(attrs[i]->store(&indio.dev, attrs[i], "2000\n", 5) == 5);
    assert(last_poll_setting == settings[i] && last_poll_value == 2000);
    assert(attrs[i]->show(&indio.dev, attrs[i], buf) == 5 && !strcmp(buf, "2000\n"));
    before = control_writes;
    assert(attrs[i]->store(&indio.dev, attrs[i], "-1", 2) == -EINVAL);
    assert(attrs[i]->store(&indio.dev, attrs[i], "invalid", 7) == -EINVAL);
    assert(attrs[i]->store(&indio.dev, attrs[i], "0x100", 5) == -EINVAL);
    assert(attrs[i]->store(&indio.dev, attrs[i], "4294967296", 10) < 0);
    assert(control_writes == before);
    control_error = -EINVAL;
    assert(attrs[i]->store(&indio.dev, attrs[i], "99", 2) == -EINVAL);
    control_error = -ENODEV;
    assert(attrs[i]->show(&indio.dev, attrs[i], buf) == -ENODEV);
    assert(attrs[i]->store(&indio.dev, attrs[i], "1000", 4) == -ENODEV);
    control_error = 0;
  }
  control_error = -ENODEV;
  assert(mode->show(&indio.dev, mode, buf) == -ENODEV);
  assert(mode->store(&indio.dev, mode, "adaptive", 8) == -ENODEV);
  control_error = 0;
  assert(iio_dev_attr_sensor_poll_interval_ms.dev_attr.store == NULL);
  indio.private = NULL;
}

static void test_firmware_attributes(void) {
  struct ssf_mems_iio_state state = {.data = &sensor};
  ssf_mems_iio_setup_features(&state);
  struct device_attribute *update = &iio_dev_attr_sensor_firmware_update.dev_attr;
  struct device_attribute *status = &iio_dev_attr_sensor_firmware_update_status.dev_attr;
  char buf[4096];

  indio.private = &state;
  ssf_mems_ota_init(&sensor);
  assert(update->show == NULL && status->store == NULL);
  assert(update->store(&indio.dev, update, "update.bin\n", 11) == 11);
  assert(firmware_updates == 1);
  assert(update->store(&indio.dev, update, "../update.bin", 13) == -EINVAL);
  assert(update->store(&indio.dev, update, "", 0) == -EINVAL);
  assert(update->store(&indio.dev, update, "bad\0file.bin", 12) == -EINVAL);
  assert(firmware_updates == 1);
  control_error = -EBUSY;
  assert(update->store(&indio.dev, update, "update.bin", 10) == -EBUSY);
  control_error = 0;
  sensor.ota.status.stage = SSF_MEMS_OTA_SENDING;
  sensor.ota.status.acknowledged = 256;
  sensor.ota.status.total = 1001;
  assert(status->show(&indio.dev, status, buf) > 0);
  assert(strcmp(buf, "sending 256/1001 error=0 version=-1 blocked=0\n") == 0);
  pthread_mutex_destroy(&sensor.ota.lock.native);
  pthread_mutex_destroy(&sensor.ota.update_lock.native);
  indio.private = NULL;
}

static void test_raw_control_attributes(void) {
  struct ssf_mems_iio_state state = {.data = &sensor};
  struct device_attribute *samples = &iio_dev_attr_sensor_raw_batch_samples.dev_attr;
  struct device_attribute *manual = &iio_dev_attr_sensor_raw_manual.dev_attr;
  struct device_attribute *capture = &iio_dev_attr_sensor_raw_capture.dev_attr;
  struct device_attribute *available = &iio_dev_attr_sensor_raw_batch_samples_available.dev_attr;
  unsigned int before = raw_capture_calls;
  char buf[4096];
  indio.private = &state;
  control_error = 0;
  assert(available->show(&indio.dev, available, buf) > 0);
  assert(!strcmp(buf, "1024 2048 4096 8192 16384 32768\n"));
  assert(samples->store(&indio.dev, samples, "2048", 4) == 4);
  assert(samples->show(&indio.dev, samples, buf) == 5 && !strcmp(buf, "2048\n"));
  assert(samples->store(&indio.dev, samples, "1000", 4) == -EINVAL);
  assert(manual->store(&indio.dev, manual, "1", 1) == 1);
  assert(manual->show(&indio.dev, manual, buf) == 2 && !strcmp(buf, "1\n"));
  assert(manual->store(&indio.dev, manual, "2", 1) == -EINVAL);
  assert(capture->show == NULL);
  assert(capture->store(&indio.dev, capture, "0", 1) == -EINVAL);
  assert(raw_capture_calls == before);
  assert(capture->store(&indio.dev, capture, "1", 1) == 1);
  before = raw_capture_calls;
  /* 已有采集副作用的命令不能向 syscall 层返回可自动重启的错误码。 */
  control_error = -ERESTARTSYS;
  assert(capture->store(&indio.dev, capture, "1", 1) == -EINTR);
  assert(raw_capture_calls == before + 1);
  control_error = -EINTR;
  assert(capture->store(&indio.dev, capture, "1", 1) == -EINTR);
  control_error = -ENODATA;
  assert(capture->store(&indio.dev, capture, "1", 1) == -ENODATA);
  control_error = 0;
  indio.private = NULL;
}

static void test_raw_status_attributes(void) {
  struct ssf_mems_iio_state state = {.data = &sensor};
  ssf_mems_iio_setup_features(&state);
  struct device_attribute *attrs[] = {
      &iio_dev_attr_raw_packets.dev_attr,
      &iio_dev_attr_raw_sequence_gaps.dev_attr,
      &iio_dev_attr_raw_crc_errors.dev_attr,
      &iio_dev_attr_raw_buffer_drops.dev_attr,
      &iio_dev_attr_raw_drain_packets.dev_attr,
      &iio_dev_attr_raw_drain_sequence_gaps.dev_attr,
      &iio_dev_attr_raw_drain_crc_errors.dev_attr,
      &iio_dev_attr_raw_start_wait_ms.dev_attr,
      &iio_dev_attr_raw_link_mode.dev_attr,
      &iio_dev_attr_raw_publishing.dev_attr,
  };
  unsigned int expected[] = {417, 1, 2, 4, 85, 5, 6, 6000, 3, 0};
  char buf[4096], want[32];
  size_t i;

  indio.private = &state;
  sensor.raw.packets = 417;
  sensor.raw.discontinuities = 1;
  sensor.raw.crc_errors = 2;
  sensor.raw.buffer_errors = 4;
  sensor.raw.drain_packets = 85;
  sensor.raw.drain_discontinuities = 5;
  sensor.raw.drain_crc_errors = 6;
  sensor.raw.start_wait_ms = 6000;
  sensor.raw.publishing = false;
  sensor.protocol.mode = SSF_MEMS_LINK_RAW_RECOVERING;
  for (i = 0; i < ARRAY_SIZE(attrs); i++) {
    snprintf(want, sizeof(want), "%u\n", expected[i]);
    assert(attrs[i]->store == NULL);
    assert(attrs[i]->show(&indio.dev, attrs[i], buf) == (ssize_t)strlen(want));
    assert(strcmp(buf, want) == 0);
  }
  memset(&sensor.raw, 0, sizeof(sensor.raw));
  sensor.protocol.mode = SSF_MEMS_LINK_MODBUS;
  indio.private = NULL;
}

/* 验证公共字段描述没有改变通道顺序、符号和规范单位。 */
static void test_feature_channels(void) {
  struct ssf_mems_iio_state state = {.data = &sensor};
  ssf_mems_iio_setup_features(&state);
  unsigned int bit;
  int value, value2;

  assert(ssf_mems_sensor_field_desc(&ssf_mems_xyzs, SSF_MEMS_FIELD_NONE) == NULL);
  assert(ssf_mems_sensor_field_desc(&ssf_mems_xyzs, SSF_MEMS_FIELD_MAX) == NULL);
  assert(ssf_mems_sensor_field_desc(&ssf_mems_xyzs, -1) == NULL);
  indio.private = &state;
  sensor.protocol.valid = true;
  sensor.protocol.features = sample;
  assert(SSF_MEMS_IIO_FEATURE_MAX == 32);
  for (bit = 0; bit < 29; bit++) {
    const struct iio_chan_spec *chan = &ssf_mems_iio_channels[bit];
    int expected = bit == 6 ? -321 : bit == 28 ? 7 : bit + 1;
    int format, scale1 = 0, scale2;

    assert(chan->scan_index == (int)bit && chan->address == bit);
    assert(chan->scan_type.storagebits == (bit == 28 ? 8 : 16));
    assert(chan->scan_type.sign == (bit == 6 ? 's' : 'u'));
    assert(ssf_mems_iio_read_raw(&indio, chan, &value, &value2,
                                IIO_CHAN_INFO_RAW) == IIO_VAL_INT);
    assert(value == expected);
    if (bit == 0 || bit == 2 || bit == 4 || (bit >= 7 && bit <= 15)) {
      format = IIO_VAL_INT_PLUS_NANO;
      scale2 = 98066500;
    } else if (bit == 1 || bit == 3 || bit == 5 || (bit >= 19 && bit <= 21)) {
      format = IIO_VAL_INT_PLUS_MICRO;
      scale2 = 10;
    } else if (bit == 6 || bit == 25 || bit == 28) {
      format = IIO_VAL_INT;
      scale1 = bit == 6 ? 10 : 1;
      scale2 = 0;
    } else {
      format = IIO_VAL_INT_PLUS_MICRO;
      scale2 = bit == 26 ? 100000 : 10000;
    }
    assert(ssf_mems_iio_read_scale(&state, chan, &value, &value2) == format);
    assert(value == scale1 && value2 == scale2);
  }
  sensor.protocol.valid = false;
  assert(ssf_mems_iio_read_raw(&indio, &ssf_mems_iio_channels[0],
                              &value, &value2, IIO_CHAN_INFO_RAW) == -ENODATA);
  indio.private = NULL;
}

/* 使用真实注册流程验证 XYZ 通道、独立偏移、单位、能力和扫描打包。 */
static void test_xyz_iio(void) {
  struct serdev_device serial = {0};
  struct ssf_mems_data data = {.serdev = &serial, .sensor = &ssf_mems_xyz,
                               .sensor_ops = ssf_mems_xyz.ops};
  struct ssf_mems_sensor_data features = {.xyz = {
      .x = {10, 20, 30, 40, 50, 60},
      .y = {70, 80, 90, 100, 110, 120},
      .z = {130, 140, 150, 160, 170, 180},
      .temperature_x100 = -3210,
  }};
  struct ssf_mems_iio_state *state;
  unsigned long mask;
  size_t i;
  int value, scale;

  allocation_calls = buffer_calls = register_calls = 0;
  assert(ssf_mems_iio_register(&data) == 0);
  assert(strcmp(data.indio_dev->name, "ssf_mems_xyz") == 0);
  assert(strcmp(data.raw_indio_dev->name, "ssf_mems_xyz_raw") == 0);
  assert(data.indio_dev->num_channels == 20);
  state = iio_priv(data.indio_dev);
  assert(state->field_count == 19 && state->channels[19].type == IIO_TIMESTAMP);
  for (i = 0; i < state->field_count; i++) {
    const struct iio_chan_spec *chan = &state->channels[i];
    assert(chan->scan_index == (int)i && chan->scan_type.storagebits == 16);
    assert(chan->address + 1 != SSF_MEMS_FIELD_SOUND_RMS);
    assert(chan->address + 1 != SSF_MEMS_FIELD_X_HIGH_FREQ_ACC_RMS);
    if (i >= 16) {
      assert(chan->type == IIO_DISTANCE);
      assert(ssf_mems_iio_read_scale(state, chan, &value, &scale) == IIO_VAL_INT_PLUS_MICRO);
      assert(value == 0 && scale == 1);
    }
  }
  assert(state->channels[0].scan_type.sign == 's');
  assert(state->fields[0].offset == offsetof(struct ssf_mems_sensor_data, xyz.temperature_x100));
  assert(ssf_mems_iio_attr_visible(&data.indio_dev->dev.kobj,
      &iio_dev_attr_sensor_feature_enable.dev_attr.attr, 0) == 0);
  assert(ssf_mems_iio_attr_visible(&data.indio_dev->dev.kobj,
      &iio_dev_attr_sensor_firmware_update.dev_attr.attr, 0) == 0);
  assert(ssf_mems_iio_attr_visible(&data.indio_dev->dev.kobj,
      &iio_dev_attr_sensor_firmware_version.dev_attr.attr, 0) == 0);
  assert(ssf_mems_iio_attr_visible(&data.indio_dev->dev.kobj,
      &iio_dev_attr_sensor_sampling_rate_index.dev_attr.attr, 0) == 0644);
  /* 检查实际交给 IIO 核心的列表，不能只测试可见性回调本身。 */
  {
    struct attribute **attr = data.indio_dev->info->attrs->attrs;
    bool saw_rate = false, saw_length = false;
    assert(data.indio_dev->info->attrs->is_visible == NULL);
    for (; *attr != NULL; attr++) {
      assert(*attr != &iio_dev_attr_sensor_feature_enable.dev_attr.attr);
      assert(*attr != &iio_dev_attr_sensor_parameter_switch.dev_attr.attr);
      assert(*attr != &iio_dev_attr_sensor_firmware_update.dev_attr.attr);
      assert(*attr != &iio_dev_attr_sensor_firmware_update_status.dev_attr.attr);
      assert(*attr != &iio_dev_attr_sensor_firmware_version.dev_attr.attr);
      saw_rate |= *attr == &iio_dev_attr_sensor_sampling_rate_index.dev_attr.attr;
      saw_length |= *attr == &iio_dev_attr_sensor_sampling_length_index.dev_attr.attr;
    }
    assert(saw_rate && saw_length);
  }
  xyz_feature_dev = data.indio_dev;
  data.indio_dev->currentmode = INDIO_BUFFER_SOFTWARE;
  data.indio_dev->active_scan_mask = &mask;
  mask = BIT(19) - 1;
  data.indio_dev->scan_bytes = 38;
  assert(ssf_mems_iio_publish_features(&data, &features, timestamp) == 0);
  data.indio_dev->scan_timestamp = true;
  data.indio_dev->scan_bytes = 48;
  assert(ssf_mems_iio_publish_features(&data, &features, timestamp) == 0);
  mask = BIT(0) | BIT(16) | BIT(18);
  data.indio_dev->scan_bytes = 16;
  assert(ssf_mems_iio_publish_features(&data, &features, timestamp) == 0);
  data.indio_dev->active_scan_mask = NULL;
  xyz_feature_dev = NULL;
  ssf_mems_iio_unregister(&data);
  release_test_devices();
}

int main(void) {
  sensor.sensor = &ssf_mems_xyzs;
  sensor.sensor_ops = ssf_mems_xyzs.ops;
  pthread_t producer, controller;

  test_registration();
  test_configuration_attributes();
  test_acquisition_controls();
  test_poll_controls();
  test_firmware_attributes();
  test_raw_status_attributes();
  test_raw_control_attributes();
  struct ssf_mems_iio_state publish_state = {.data = &sensor};
  ssf_mems_iio_setup_features(&publish_state);
  indio.private = &publish_state;
  sensor.indio_dev = &indio;
  mutex_init(&sensor.protocol.lock);
  mutex_init(&indio.mlock);
  test_feature_channels();
  indio.private = &publish_state;
  configure_buffer(4);
  memset(&sensor.protocol.features, 0xa5, sizeof(sensor.protocol.features));
  struct ssf_mems_sensor_data cached_before = sensor.protocol.features;
  assert(ssf_mems_iio_publish_features(&sensor, &sample, timestamp) == 0);
  assert(pushes == 1);
  assert(pthread_barrier_init(&start, NULL, 2) == 0);
  assert(pthread_create(&producer, NULL, produce, NULL) == 0);
  assert(pthread_create(&controller, NULL, reconfigure, NULL) == 0);
  assert(pthread_join(producer, NULL) == 0);
  assert(pthread_join(controller, NULL) == 0);
  assert(!sensor.protocol.valid && memcmp(&sensor.protocol.features, &cached_before, sizeof(cached_before)) == 0);
  free(indio.active_scan_mask);
  pthread_barrier_destroy(&start);
  pthread_mutex_destroy(&indio.mlock.native);
  pthread_mutex_destroy(&sensor.protocol.lock.native);
  test_raw_buffer();
  pthread_mutex_destroy(&raw_indio.mlock.native);
  test_xyz_iio();
  printf("PASS IIO config attributes and publisher: 100000 publications / 20000 buffer reconfigurations, %u pushes\n", pushes);
  return 0;
}
