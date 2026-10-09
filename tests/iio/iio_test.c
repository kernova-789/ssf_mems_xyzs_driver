#include <assert.h>
#include <errno.h>
#include <sched.h>
#include <stdarg.h>

/* Compile the complete production source; the scan packing is not copied. */
#include "../../iio/ssf_mems_iio.c"

static struct ssf_mems_xyzs_data sensor;
static struct iio_dev indio;
static struct iio_dev raw_indio;
static struct ssf_mems_raw_sample raw_sample = {
    .xyz = {-2, -32768, 5399}, .packet_sequence = 0x12345678, .sample_index = 63,
};
static struct ssf_mems_sensor_data sample;
static pthread_barrier_t start;
static unsigned int pushes;
static const s64 timestamp = 123456789;
static int setting_error;
static u16 setting_value;
static enum ssf_mems_sensor_setting last_setting;

int ssf_mems_protocol_store_features(struct ssf_mems_xyzs_data *data,
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
  size_t offset = 0, bit;

  /* The same mutex must protect enabled state, mask, size, and the final push. */
  assert(pthread_mutex_trylock(&dev->mlock.native) == EBUSY);
  assert(dev->currentmode == INDIO_BUFFER_SOFTWARE && dev->active_scan_mask);
  assert(ts == timestamp);
  if (dev == &raw_indio) {
    for (bit = 0; bit < 5; bit++) {
      u32 value = bit == 0 ? 0xfffe : bit == 1 ? 0x8000 :
                  bit == 2 ? 5399 : bit == 3 ? 0x12345678 : 63;
      size_t width = bit == 3 ? 4 : 2;
      if ((*dev->active_scan_mask & BIT(bit)) == 0)
        continue;
      offset = ALIGN(offset, width);
      memcpy(expected + offset, &value, width);
      offset += width;
    }
    if (dev->scan_timestamp) {
      offset = ALIGN(offset, sizeof(ts));
      memcpy((u8 *)scan + offset, &ts, sizeof(ts));
      memcpy(expected + offset, &ts, sizeof(ts));
      offset += sizeof(ts);
    }
    assert(offset == dev->scan_bytes && memcmp(scan, expected, offset) == 0);
    pushes++;
    return 0;
  }
  for (bit = 0; bit < SSF_MEMS_IIO_FEATURE_MAX; bit++) {
    const struct iio_chan_spec *chan = &ssf_mems_iio_channels[bit];
    if (!(*dev->active_scan_mask & BIT(bit)))
      continue;
    assert(chan->scan_index == (int)bit);
    /* Independent expected values: all 16-bit fields are 0x1234 except temp. */
    if (bit == SSF_MEMS_IIO_STARTUP_FLAGS) {
      expected[offset++] = 7;
    } else {
      s16 value = bit == SSF_MEMS_IIO_TEMPERATURE ? -321 : 0x1234;
      offset = ALIGN(offset, sizeof(value));
      memcpy(expected + offset, &value, sizeof(value));
      offset += sizeof(value);
    }
  }
  if (dev->scan_timestamp) {
    offset = ALIGN(offset, sizeof(ts));
    memcpy((u8 *)scan + offset, &ts, sizeof(ts));
    memcpy(expected + offset, &ts, sizeof(ts));
    offset += sizeof(ts);
  }
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
  bytes = variant % 4 == 0 ? 3 : variant % 4 == 1 ? 2 : variant % 4 == 2 ? 57 : 1;
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
  unsigned long mask;
  unsigned int with_timestamp, bit;
  int val, val2;

  mutex_init(&raw_indio.mlock);
  sensor.raw_indio_dev = &raw_indio;
  raw_indio.currentmode = INDIO_BUFFER_SOFTWARE;
  raw_indio.active_scan_mask = &mask;
  for (mask = 1; mask < BIT(5); mask++) {
    size_t bytes = 0;
    for (bit = 0; bit < 5; bit++) {
      size_t width = bit == 3 ? 4 : 2;
      if ((mask & BIT(bit)) != 0)
        bytes = ALIGN(bytes, width) + width;
    }
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
  assert(ssf_mems_iio_publish_raw(&sensor, &raw_sample) == -EINVAL);
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
int ssf_mems_protocol_get_features(struct serdev_device *s, struct ssf_mems_sensor_data *out) { abort(); }
int ssf_mems_acquisition_get_baudrate(struct ssf_mems_xyzs_data *d, enum ssf_mems_baudrate *b) { abort(); }
int ssf_mems_acquisition_set_baudrate(struct ssf_mems_xyzs_data *d, enum ssf_mems_baudrate b) { abort(); }
int ssf_mems_acquisition_read_setting(struct ssf_mems_xyzs_data *d,
                                    enum ssf_mems_sensor_setting s, u16 *v) {
  assert(d == &sensor);
  last_setting = s;
  *v = setting_value;
  return setting_error;
}
int ssf_mems_acquisition_write_setting(struct ssf_mems_xyzs_data *d,
                                     enum ssf_mems_sensor_setting s, u16 v) {
  assert(d == &sensor);
  last_setting = s;
  setting_value = v;
  return setting_error;
}
void ssf_mems_acquisition_get_status(struct ssf_mems_xyzs_data *d, struct ssf_mems_acquisition_status *s) { abort(); }
int ssf_mems_baudrate_to_value(enum ssf_mems_baudrate b) { abort(); }
int sysfs_emit(char *buf, const char *fmt, ...) {
  va_list args;
  int count;
  va_start(args, fmt);
  count = vsnprintf(buf, 4096, fmt, args);
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
  char buf[4096];
  struct device_attribute *attr;

  indio.private = &state;
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
  setting_error = -EIO;
  assert(attr->store(&indio.dev, attr, "6\n", 2) == -EIO);
  setting_error = 0;
  indio.private = NULL;
}

/* 新统计属性均只读；有效采集、排空和业务模式分别输出，不能串错成员。 */
static void test_raw_status_attributes(void) {
  struct ssf_mems_iio_state state = {.data = &sensor};
  struct device_attribute *attrs[] = {
      &iio_dev_attr_raw_packets.dev_attr,
      &iio_dev_attr_raw_sequence_gaps.dev_attr,
      &iio_dev_attr_raw_crc_errors.dev_attr,
      &iio_dev_attr_raw_buffer_drops.dev_attr,
      &iio_dev_attr_raw_drain_packets.dev_attr,
      &iio_dev_attr_raw_drain_sequence_gaps.dev_attr,
      &iio_dev_attr_raw_drain_crc_errors.dev_attr,
      &iio_dev_attr_raw_start_wait_ms.dev_attr,
      &iio_dev_attr_raw_stop_attempts.dev_attr,
      &iio_dev_attr_raw_link_mode.dev_attr,
      &iio_dev_attr_raw_publishing.dev_attr,
  };
  unsigned int expected[] = {417, 1, 2, 4, 85, 5, 6, 6000, 3, 3, 0};
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
  sensor.raw.stop_attempts = 3;
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

int main(void) {
  pthread_t producer, controller;
  size_t i;

  test_configuration_attributes();
  test_raw_status_attributes();
  memset(&sample, 0, sizeof(sample));
  for (i = 0; i < SSF_MEMS_IIO_FEATURE_MAX; i++) {
    const struct ssf_mems_iio_feature_desc *desc = &ssf_mems_iio_feature_descs[i];
    if (desc->width == 2) {
      u16 value = 0x1234;
      memcpy((u8 *)&sample + desc->offset, &value, sizeof(value));
    }
  }
  sample.temperature_x100 = -321;
  sample.startup_flags = 7;
  sensor.indio_dev = &indio;
  mutex_init(&sensor.protocol.lock);
  mutex_init(&indio.mlock);
  configure_buffer(4);
  assert(ssf_mems_iio_publish_features(&sensor, &sample, timestamp) == 0);
  assert(pushes == 1);
  assert(pthread_barrier_init(&start, NULL, 2) == 0);
  assert(pthread_create(&producer, NULL, produce, NULL) == 0);
  assert(pthread_create(&controller, NULL, reconfigure, NULL) == 0);
  assert(pthread_join(producer, NULL) == 0);
  assert(pthread_join(controller, NULL) == 0);
  assert(sensor.protocol.valid && memcmp(&sensor.protocol.features, &sample, sizeof(sample)) == 0);
  free(indio.active_scan_mask);
  pthread_barrier_destroy(&start);
  pthread_mutex_destroy(&indio.mlock.native);
  pthread_mutex_destroy(&sensor.protocol.lock.native);
  test_raw_buffer();
  pthread_mutex_destroy(&raw_indio.mlock.native);
  printf("PASS IIO config attributes and publisher: 100000 publications / 20000 buffer reconfigurations, %u pushes\n", pushes);
  return 0;
}
