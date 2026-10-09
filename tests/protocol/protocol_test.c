#include "core.h"
#include "modbus.h"
#include "modbus_table.h"
#include "modbus_request.h"
#include "modbus_receive.h"
#include "protocol.h"
#include <linux/jiffies.h>

bool test_allocation_failure;
long test_wait_result;
unsigned int test_queue_count, test_cancel_count, test_wake_count;
unsigned int test_delayed_queue_count, test_delayed_cancel_count;
unsigned long jiffies;
static void (*wait_hook)(unsigned long timeout);
static bool wait_returns_early; /* 新的时序用例允许唤醒发生在 timeout 之前。 */

long test_wait_timeout(unsigned long timeout) {
  unsigned long end = jiffies + timeout;
  if (wait_hook)
    wait_hook(timeout);
  if (wait_returns_early == true && test_wait_result > 0)
    return test_wait_result;
  if (time_before(jiffies, end))
    jiffies = end;
  return test_wait_result;
}

static struct ssf_mems_xyzs_data sensor;
static struct serdev_device serial;
static unsigned int checks, sends;
static u8 last_tx[256];
static size_t last_tx_len;
static u16 read_starts[64], read_counts[64];
static unsigned int read_runs;
static ssize_t forced_send_result = -1;
static int response_mode;
static unsigned int fail_on_send;
static bool check_busy_handoff;
static bool saw_block_description;
static bool sensor_initialized;
static bool enforce_sensor_baudrate;
static bool invalid_baudrate_value;
static bool wrong_baudrate_value;
static enum ssf_mems_baudrate sensor_baudrate;
static enum ssf_mems_baudrate staged_baudrate;
static unsigned long sensor_reboot_until;
static unsigned int sensor_extra_boot_ms;
static bool verification_never_replies;
static unsigned int reboots, sleep_count, slept_ms;
static void (*sleep_hook)(void);
static unsigned int special_reply_send;
static int special_reply_mode;
static bool emulate_settings, ignore_setting_writes;
static u16 setting_registers[121];
struct tx_event {
  u8 function;
  u16 address, value;
  unsigned int baudrate;
  unsigned long sent_at;
};
static struct tx_event tx_events[256];
static unsigned int raw_published;
static struct ssf_mems_raw_sample raw_last;
static int raw_publish_error;
static bool emulate_raw_mode;
static unsigned int raw_wait_action;
static unsigned int ignored_raw_stops; /* 模拟停止命令在链路/固件中丢失。 */
static bool raw_sensor_running; /* 模拟私有流尚未停止，而非普通配置改变。 */
static enum ssf_mems_baudrate raw_restore_baudrate; /* 停流后返回的普通配置。 */
static unsigned long late_raw_packet_at; /* 首包前卸载时，固件稍后才开始输出。 */
static u32 drain_sequence; /* 排空包使用独立序号，验证统计不会污染有效段。 */
static unsigned int raw_gap_waits; /* 记录停止前等待包间空档的次数。 */
static bool raw_gap_partial; /* 模拟唤醒时下一包已经收到一个字节。 */

enum { REPLY_NORMAL, REPLY_EXCEPTION, REPLY_BAD_CRC, REPLY_WRONG_COUNT, REPLY_NONE, REPLY_MISMATCH_THEN_NORMAL };
#define CHECK(expr) do { checks++; if (!(expr)) { fprintf(stderr, "FAIL line %d: %s\n", __LINE__, #expr); abort(); } } while (0)
#define NO_SEND(expr, error) do { unsigned int before = sends; CHECK((expr) == (error)); CHECK(sends == before); } while (0)

/* 记录生产解析器交给 IIO 的样本，而不复制解码实现。 */
int ssf_mems_iio_publish_raw(struct ssf_mems_xyzs_data *data,
                            const struct ssf_mems_raw_sample *sample) {
  CHECK(data == &sensor);
  raw_published++;
  raw_last = *sample;
  return raw_publish_error;
}

void test_msleep(unsigned int ms) {
  CHECK(!sensor.modbus_req.pending && !sensor.modbus_req.busy);
  sleep_count++;
  slept_ms += ms;
  if (sleep_hook)
    sleep_hook();
  jiffies += ms;
}

static void apply_sensor_write(u16 address, u16 value) {
  if (emulate_settings && !ignore_setting_writes)
    setting_registers[address] = value;
  if (address == 101 && value < SSF_MEMS_BAUDRATE_MAX) {
    staged_baudrate = value;
  } else if (address == 109 && value == 1) {
    CHECK(staged_baudrate < SSF_MEMS_BAUDRATE_MAX);
    sensor_baudrate = staged_baudrate;
    sensor_reboot_until = jiffies + SSF_MEMS_SENSOR_REBOOT_DELAY_MS +
                          sensor_extra_boot_ms;
    reboots++;
  }
}

static u16 wire_value(u16 address) {
  if (emulate_settings)
    return setting_registers[address];
  return address == 28 ? 0xfffd : 0x2200 + address;
}

static void finish_crc(u8 *buf, size_t len) {
  u16 crc = ssf_mems_modbus_crc16(buf, len - 2);
  buf[len - 2] = crc;
  buf[len - 1] = crc >> 8;
}

/* 验证完整帧处理后所有候选槽位均已释放。 */
static void check_released_slots(struct ssf_mems_xyzs_data *data) {
  size_t i;
  for (i = 0; i < ARRAY_SIZE(data->modbus_rx.frame); i++) {
    const struct ssf_mems_frame_slot *slot = &data->modbus_rx.frame[i];
    CHECK(!slot->data && !slot->frame_desc && !slot->data_pos);
    CHECK(atomic_read(&slot->in_use) == SSF_MEMS_FRAME_SLOT_FREE);
  }
}

/* 验证未认领帧不改变请求、输出、缓存或唤醒计数，并释放完整帧缓冲区。 */
static void check_discard_preserves_state(struct ssf_mems_xyzs_data *data,
                                         const u8 *frame, size_t len) {
  u8 request_before[sizeof(data->modbus_req)];
  struct ssf_mems_sensor_data features_before = data->protocol.features;
  u16 values_before[126];
  u16 *values = data->modbus_req.values;
  size_t values_count = data->modbus_req.values_count;
  unsigned int wakes_before = test_wake_count;
  bool valid_before = data->protocol.valid;

  CHECK(values_count <= ARRAY_SIZE(values_before));
  memcpy(request_before, &data->modbus_req, sizeof(request_before));
  if (values)
    memcpy(values_before, values, values_count * sizeof(*values));
  CHECK(ssf_mems_protocol_handle_frame(data, frame, len) == 0);
  CHECK(ssf_mems_rx_push(data->serdev, frame, len) == (int)len);
  data->modbus_rx.work.fn(&data->modbus_rx.work);
  CHECK(memcmp(request_before, &data->modbus_req, sizeof(request_before)) == 0);
  CHECK(memcmp(&features_before, &data->protocol.features, sizeof(features_before)) == 0);
  CHECK(data->protocol.valid == valid_before && test_wake_count == wakes_before);
  if (values)
    CHECK(memcmp(values_before, values, values_count * sizeof(*values)) == 0);
  check_released_slots(data);
}

ssize_t serdev_device_write(struct serdev_device *s, const u8 *buf,
                           size_t size, unsigned long timeout) {
  u8 response[256] = {0};
  struct ssf_mems_xyzs_data *data = s->drvdata;
  size_t len, offset;
  u16 address, count, i;
  int mode;

  CHECK(size <= sizeof(last_tx));
  s->write_timeout = timeout;
  memcpy(last_tx, buf, size);
  last_tx_len = size;
  sends++;
  CHECK(ssf_mems_modbus_check_crc(buf, size) == 0);
  address = ((u16)buf[2] << 8) | buf[3];
  count = ((u16)buf[4] << 8) | buf[5];
  CHECK(sends <= ARRAY_SIZE(tx_events));
  tx_events[sends - 1] = (struct tx_event){buf[1], address, count,
                                         s->baudrate, jiffies};
  mode = sends == special_reply_send ? special_reply_mode : response_mode;
  if (buf[1] == 3) {
    CHECK(read_runs < ARRAY_SIZE(read_starts));
    read_starts[read_runs] = address;
    read_counts[read_runs++] = count;
    saw_block_description = data->modbus_req.transfer.block != NULL;
  }

  if (fail_on_send == sends)
    return -EIO;
  if (forced_send_result != -1)
    return forced_send_result;
  if (emulate_raw_mode && buf[1] == 3 && address == 57) {
    CHECK(count == 1 && s->baudrate == 9600);
    sensor_baudrate = SSF_MEMS_BAUDRATE_1000000;
    raw_sensor_running = true;
    return size; /* 启动命令没有普通 Modbus 应答。 */
  }
  if (emulate_raw_mode && buf[1] == 6 && address == 58) {
    CHECK(count == 1 && s->baudrate == 1000000);
    if (ignored_raw_stops != 0) {
      ignored_raw_stops--;
      return size;
    }
    sensor_baudrate = raw_restore_baudrate;
    raw_sensor_running = false;
    return size; /* 不依赖停止应答，恢复后再读寄存器确认。 */
  }
  if (enforce_sensor_baudrate &&
      (time_before(jiffies, sensor_reboot_until) ||
       s->baudrate != (unsigned int)ssf_mems_baudrate_to_value(sensor_baudrate)))
    return size;
  if (verification_never_replies && reboots && buf[1] == 3 && address == 101)
    return size;
  if (mode == REPLY_NONE) {
    if (enforce_sensor_baudrate && buf[1] == SSF_MEMS_MODBUS_FUNC_WRITE_SINGLE)
      apply_sensor_write(address, count);
    return size;
  }

  if (mode == REPLY_MISMATCH_THEN_NORMAL) {
    u8 unmatched[] = {1, 6, 0, 100, 0x12, 0x34, 0, 0};
    CHECK(data->modbus_req.pending && data->modbus_req.busy);
    finish_crc(unmatched, sizeof(unmatched));
    check_discard_preserves_state(data, unmatched, sizeof(unmatched));
  }

  response[0] = buf[0];
  response[1] = buf[1];
  if (mode == REPLY_EXCEPTION) {
    response[1] |= 0x80;
    response[2] = 2;
    len = 5;
  } else if (buf[1] == 3) {
    if (mode == REPLY_WRONG_COUNT)
      count--;
    response[2] = count * 2;
    len = 5 + count * 2;
    for (i = 0; i < count; i++) {
      u16 value;

      if (enforce_sensor_baudrate && address + i == 101)
        value = invalid_baudrate_value ? SSF_MEMS_BAUDRATE_MAX
                : wrong_baudrate_value ? SSF_MEMS_BAUDRATE_57600
                : staged_baudrate < SSF_MEMS_BAUDRATE_MAX ? staged_baudrate
                                                        : sensor_baudrate;
      else
        value = wire_value(address + i);
      response[3 + i * 2] = value >> 8;
      response[4 + i * 2] = value;
    }
  } else {
    memcpy(response + 2, buf + 2, 4);
    len = 8;
  }
  finish_crc(response, len);
  if (mode == REPLY_BAD_CRC)
    response[len - 1] ^= 0x40;

  /* Feed real FIFO/candidate-parser code in chunks that split headers and CRC. */
  for (offset = 0; offset < len;) {
    size_t chunk = offset % 4 + 1;
    if (chunk > len - offset) chunk = len - offset;
    CHECK(ssf_mems_rx_push(s, response + offset, chunk) == (int)chunk);
    data->modbus_rx.work.fn(&data->modbus_rx.work);
    offset += chunk;
  }
  if (check_busy_handoff && !data->modbus_req.pending) {
    u16 ignored;
    CHECK(data->modbus_req.busy);
    CHECK(ssf_mems_modbus_read_reg(s, 40121, &ignored, 0) == -EBUSY);
  }
  if ((enforce_sensor_baudrate || emulate_settings) && mode != REPLY_EXCEPTION &&
      buf[1] == SSF_MEMS_MODBUS_FUNC_WRITE_SINGLE)
    apply_sensor_write(address, count);
  return size;
}

static void cleanup_sensor(void) {
  if (!sensor_initialized)
    return;
  ssf_mems_modbus_receive_remove(&sensor);
  ssf_mems_modbus_request_remove(&sensor);
  sensor_initialized = false;
}

static void reset_sensor(void) {
  cleanup_sensor();
  jiffies = 0;
  wait_hook = NULL;
  wait_returns_early = false;
  memset(&sensor, 0, sizeof(sensor));
  memset(&serial, 0, sizeof(serial));
  serial.drvdata = &sensor;
  sensor.serdev = &serial;
  sensor.slave_id = SSF_MEMS_MODBUS_DEFAULT_SLAVE_ID;
  serial.baudrate = 9600;
  test_allocation_failure = false;
  CHECK(ssf_mems_protocol_init(&sensor) == 0);
  CHECK(ssf_mems_modbus_request_init(&sensor) == 0);
  CHECK(ssf_mems_modbus_receive_init(&sensor, ssf_mems_protocol_handle_frame, &sensor) == 0);
  sensor_initialized = true;
  sends = read_runs = 0;
  forced_send_result = -1;
  response_mode = REPLY_NORMAL;
  fail_on_send = 0;
  check_busy_handoff = saw_block_description = test_allocation_failure = false;
  enforce_sensor_baudrate = invalid_baudrate_value = false;
  wrong_baudrate_value = false;
  emulate_settings = ignore_setting_writes = false;
  memset(setting_registers, 0, sizeof(setting_registers));
  sensor_baudrate = SSF_MEMS_BAUDRATE_DEFAULT;
  staged_baudrate = SSF_MEMS_BAUDRATE_MAX;
  sensor_reboot_until = 0;
  sensor_extra_boot_ms = 0;
  verification_never_replies = false;
  reboots = sleep_count = slept_ms = special_reply_send = 0;
  sleep_hook = NULL;
  test_wait_result = 0;
  raw_published = 0;
  raw_publish_error = 0;
  emulate_raw_mode = false;
  raw_wait_action = 0;
  ignored_raw_stops = 0;
  raw_sensor_running = false;
  raw_restore_baudrate = SSF_MEMS_BAUDRATE_9600;
  late_raw_packet_at = 0;
  drain_sequence = 1000;
  raw_gap_waits = 0;
  raw_gap_partial = false;
  test_queue_count = test_cancel_count = test_wake_count = 0;
  test_delayed_queue_count = test_delayed_cancel_count = 0;
}

static void test_initialization_and_cleanup(void) {
  struct ssf_mems_xyzs_data local;
  struct ssf_mems_sensor_data zero = {0};
  const u8 partial_frame[] = {1, 3, 2, 0x12};
  u8 fifo_data[SSF_MEMS_RX_FIFO_SIZE] = {0};
  u16 output;
  size_t i;
  unsigned int queued;

  CHECK(ssf_mems_protocol_init(NULL) == -EINVAL);
  CHECK(ssf_mems_modbus_request_init(NULL) == -EINVAL);
  CHECK(ssf_mems_modbus_receive_init(NULL, ssf_mems_protocol_handle_frame, &sensor) == -EINVAL);
  CHECK(ssf_mems_modbus_receive_init(&local, NULL, &local) == -EINVAL);
  ssf_mems_modbus_request_remove(NULL);
  ssf_mems_modbus_receive_remove(NULL);

  /* Initialization must not depend on core having zero-filled each module's state. */
  memset(&local, 0xa5, sizeof(local));
  local.serdev = &serial;
  CHECK(ssf_mems_protocol_init(&local) == 0);
  CHECK(!local.protocol.valid && local.protocol.lock.unused == 0 &&
        local.protocol.bus_lock.unused == 0);
  CHECK(local.protocol.baudrate == SSF_MEMS_BAUDRATE_DEFAULT);
  CHECK(local.protocol.host_baudrate == 9600 &&
        local.protocol.parity == SSF_MEMS_PARITY_NONE);
  CHECK(memcmp(&local.protocol.features, &zero, sizeof(zero)) == 0);
  local.protocol.features.x.acc_rms_x100 = 0x4321;
  local.protocol.valid = true;
  CHECK(ssf_mems_modbus_request_init(&local) == 0);
  CHECK(local.protocol.valid && local.protocol.features.x.acc_rms_x100 == 0x4321);
  CHECK(!local.modbus_req.busy && !local.modbus_req.pending && !local.modbus_req.shutting_down);
  CHECK(!local.modbus_req.values && !local.modbus_req.transfer.frame);
  CHECK(!local.modbus_req.slave_id && local.modbus_req.status == 0 &&
        local.modbus_req.lock.unused == 0);

  test_allocation_failure = true;
  CHECK(ssf_mems_modbus_receive_init(&local, ssf_mems_protocol_handle_frame, &local) == -ENOMEM);
  CHECK(local.modbus_rx.shutting_down && !local.modbus_rx.fifo.data);
  CHECK(local.protocol.valid && local.modbus_req.status == 0);
  test_allocation_failure = false;
  CHECK(ssf_mems_modbus_receive_init(&local, ssf_mems_protocol_handle_frame, &local) == 0);
  CHECK(!local.modbus_rx.shutting_down && local.modbus_rx.fifo_lock == 0);
  CHECK(local.modbus_rx.fifo.data && local.modbus_rx.fifo.capacity == SSF_MEMS_RX_FIFO_SIZE);
  CHECK(local.modbus_rx.fifo.count == 0 && local.modbus_rx.work.fn != NULL);
  CHECK(local.protocol.valid && local.modbus_req.status == 0);
  for (i = 0; i < ARRAY_SIZE(local.modbus_rx.frame); i++) {
    struct ssf_mems_frame_slot *slot = &local.modbus_rx.frame[i];
    CHECK(!slot->data && !slot->frame_desc && !slot->data_len && !slot->data_pos);
    CHECK(slot->state == SSF_MEMS_RX_IDLE && !slot->slave_id && !slot->function);
    CHECK(atomic_read(&slot->in_use) == SSF_MEMS_FRAME_SLOT_FREE);
  }
  ssf_mems_modbus_receive_remove(&local);
  ssf_mems_modbus_request_remove(&local);

  reset_sensor();
  CHECK(ssf_mems_rx_push(&serial, fifo_data,
                         SSF_MEMS_RX_FIFO_SIZE - 4) ==
        SSF_MEMS_RX_FIFO_SIZE - 4);
  CHECK(ssf_mems_rx_push(&serial, fifo_data, 8) == 4);
  CHECK(sensor.modbus_rx.fifo.count == SSF_MEMS_RX_FIFO_SIZE);

  reset_sensor();
  CHECK(ssf_mems_rx_push(&serial, partial_frame, sizeof(partial_frame)) == sizeof(partial_frame));
  CHECK(test_queue_count == 1);
  sensor.modbus_rx.work.fn(&sensor.modbus_rx.work);
  CHECK(sensor.modbus_rx.fifo.count == 0);
  CHECK(sensor.modbus_rx.frame[0].data != NULL);
  CHECK(sensor.modbus_rx.frame[0].data_pos == sizeof(partial_frame));

  /* Request teardown must no longer cancel/free the receive module's resources. */
  sensor.modbus_req.pending = true;
  sensor.modbus_req.values = &output;
  ssf_mems_modbus_request_remove(&sensor);
  CHECK(sensor.modbus_req.status == -ENODEV && !sensor.modbus_req.pending);
  CHECK(!sensor.modbus_req.values && test_cancel_count == 0);
  CHECK(sensor.modbus_rx.fifo.data && sensor.modbus_rx.frame[0].data);
  CHECK(!sensor.modbus_rx.shutting_down);

  ssf_mems_modbus_receive_remove(&sensor);
  CHECK(sensor.modbus_rx.shutting_down && test_cancel_count == 1);
  CHECK(!sensor.modbus_rx.fifo.data);
  for (i = 0; i < ARRAY_SIZE(sensor.modbus_rx.frame); i++) {
    CHECK(!sensor.modbus_rx.frame[i].data && !sensor.modbus_rx.frame[i].frame_desc);
    CHECK(atomic_read(&sensor.modbus_rx.frame[i].in_use) == SSF_MEMS_FRAME_SLOT_FREE);
  }
  queued = test_queue_count;
  CHECK(ssf_mems_rx_push(&serial, partial_frame, sizeof(partial_frame)) == -ENODEV);
  sensor.modbus_rx.work.fn(&sensor.modbus_rx.work);
  CHECK(sensor.modbus_rx.fifo.count == 0);
  ssf_mems_modbus_queue_parse(&serial);
  CHECK(test_queue_count == queued);
  sensor_initialized = false;
}

struct callback_capture {
  unsigned int calls;
  size_t len;
  u8 bytes[256];
};

/* 保存完整帧并返回测试错误；接收层不得将回调错误再解释成其他协议。 */
static int capture_frame(void *context, const u8 *buf, size_t len) {
  struct callback_capture *capture = context;
  CHECK(capture != NULL && len <= sizeof(capture->bytes));
  capture->calls++;
  capture->len = len;
  memcpy(capture->bytes, buf, len);
  return -EOPNOTSUPP;
}

static void test_receive_callback(void) {
  struct callback_capture capture = {0};
  u8 frame[] = {1, 6, 0, 100, 0x12, 0x34, 0, 0};

  reset_sensor();
  ssf_mems_modbus_receive_remove(&sensor);
  CHECK(ssf_mems_modbus_receive_init(&sensor, capture_frame, &capture) == 0);
  finish_crc(frame, sizeof(frame));
  CHECK(ssf_mems_rx_push(&serial, frame, 2) == 2);
  sensor.modbus_rx.work.fn(&sensor.modbus_rx.work);
  CHECK(capture.calls == 0);
  CHECK(ssf_mems_rx_push(&serial, frame + 2, sizeof(frame) - 2) == sizeof(frame) - 2);
  sensor.modbus_rx.work.fn(&sensor.modbus_rx.work);
  CHECK(capture.calls == 1 && capture.len == sizeof(frame));
  CHECK(memcmp(capture.bytes, frame, sizeof(frame)) == 0);
  check_released_slots(&sensor);

  frame[sizeof(frame) - 1] ^= 0x40;
  CHECK(ssf_mems_rx_push(&serial, frame, sizeof(frame)) == sizeof(frame));
  sensor.modbus_rx.work.fn(&sensor.modbus_rx.work);
  CHECK(capture.calls == 1);
  CHECK(!ssf_mems_modbus_claim_frame(NULL, frame, sizeof(frame)));
  CHECK(!ssf_mems_modbus_claim_frame(&sensor, NULL, sizeof(frame)));
  CHECK(!ssf_mems_modbus_claim_frame(&sensor, frame, 0));

  ssf_mems_modbus_receive_remove(&sensor);
  sensor.modbus_rx.work.fn(&sensor.modbus_rx.work);
  CHECK(capture.calls == 1);
}

static void test_unclaimed_frames(void) {
  struct ssf_mems_sensor_data cached;
  u8 frame[] = {1, 3, 2, 0x11, 0x22, 0, 0};
  u16 values[] = {0xabcd, 0x9876};
  unsigned int wakes_before;

  reset_sensor();
  memset(&sensor.protocol.features, 0xa5, sizeof(sensor.protocol.features));
  sensor.protocol.valid = true;
  cached = sensor.protocol.features;
  finish_crc(frame, sizeof(frame));
  CHECK(ssf_mems_protocol_handle_frame(NULL, frame, sizeof(frame)) == -EINVAL);
  CHECK(ssf_mems_protocol_handle_frame(&sensor, NULL, sizeof(frame)) == -EINVAL);
  CHECK(!sensor.modbus_req.pending && !sensor.modbus_req.busy);
  check_discard_preserves_state(&sensor, frame, sizeof(frame));

  response_mode = REPLY_MISMATCH_THEN_NORMAL;
  wakes_before = test_wake_count;
  CHECK(ssf_mems_modbus_read(&serial, 40101, 2, values, ARRAY_SIZE(values), 0) == 0);
  CHECK(values[0] == wire_value(100) && values[1] == wire_value(101));
  CHECK(!sensor.modbus_req.pending && !sensor.modbus_req.busy && sensor.modbus_req.status == 0);
  CHECK(test_wake_count == wakes_before + 1);
  CHECK(memcmp(&cached, &sensor.protocol.features, sizeof(cached)) == 0 && sensor.protocol.valid);
  check_released_slots(&sensor);

  response_mode = REPLY_WRONG_COUNT;
  values[0] = 0xabcd;
  values[1] = 0x9876;
  wakes_before = test_wake_count;
  CHECK(ssf_mems_modbus_read(&serial, 40101, 2, values, ARRAY_SIZE(values), 0) == -ETIMEDOUT);
  CHECK(values[0] == 0xabcd && values[1] == 0x9876);
  CHECK(!sensor.modbus_req.pending && !sensor.modbus_req.busy);
  CHECK(test_wake_count == wakes_before);
  CHECK(memcmp(&cached, &sensor.protocol.features, sizeof(cached)) == 0 && sensor.protocol.valid);
  check_released_slots(&sensor);
}

struct expected_field { u16 display_reg; size_t offset, width; u16 mask; };
#define FIELD(index, member) {40001 + index, offsetof(struct ssf_mems_sensor_data, member), sizeof(((struct ssf_mems_sensor_data *)0)->member), 0xffff}
static const struct expected_field feature_fields[] = {
  FIELD(0, x.high_freq_acc_rms_x100), FIELD(1, x.low_freq_velocity_rms_x100),
  FIELD(2, y.high_freq_acc_rms_x100), FIELD(3, y.low_freq_velocity_rms_x100),
  FIELD(4, z.high_freq_acc_rms_x100), FIELD(5, z.low_freq_velocity_rms_x100),
  FIELD(6, temperature_x100), FIELD(7, x.acc_peak_to_peak_x100),
  FIELD(8, y.acc_peak_to_peak_x100), FIELD(9, z.acc_peak_to_peak_x100),
  FIELD(10, x.acc_peak_x100), FIELD(11, y.acc_peak_x100), FIELD(12, z.acc_peak_x100),
  FIELD(13, x.acc_rms_x100), FIELD(14, y.acc_rms_x100), FIELD(15, z.acc_rms_x100),
  FIELD(16, x.kurtosis_x100), FIELD(17, y.kurtosis_x100), FIELD(18, z.kurtosis_x100),
  FIELD(19, x.velocity_rms_x100), FIELD(20, y.velocity_rms_x100), FIELD(21, z.velocity_rms_x100),
  FIELD(22, sound.rms_db_x100), FIELD(23, sound.peak_db_x100), FIELD(24, sound.peak_to_peak_db_x100),
  FIELD(25, zero_crossing_rate_percent), FIELD(26, spectral_centroid_hz_x10),
  FIELD(27, spectral_flux_x100),
  {40029, offsetof(struct ssf_mems_sensor_data, startup_flags), sizeof(u8), 7},
};

static void test_sparse_features(void) {
  struct ssf_mems_sensor_data expected = {0}, actual, decoded;
  u16 full_values[29];
  unsigned int wanted_runs = 0, wanted_count = 0;
  u16 expected_starts[29], expected_counts[29];
  bool previous_enabled = false;
  size_t i;

  reset_sensor();
  CHECK(ssf_mems_protocol_get_features(&serial, &actual) == -ENODATA);
  memset(&sensor.protocol.features, 0xa5, sizeof(sensor.protocol.features));
  sensor.protocol.valid = true;

  for (i = 0; i < ARRAY_SIZE(feature_fields); i++) {
    const struct expected_field *field = &feature_fields[i];
    const struct ssf_reg_desc *reg = ssf_mems_modbus_find_reg(field->display_reg);
    u16 value = 0;
    bool enabled = reg != NULL && (reg->access & SSF_REG_READ);

    full_values[i] = wire_value(i);
    if (enabled) {
      value = full_values[i] & field->mask;
      wanted_count++;
      if (!previous_enabled) {
        expected_starts[wanted_runs] = i;
        expected_counts[wanted_runs++] = 0;
      }
      expected_counts[wanted_runs - 1]++;
    }
    if (field->width == sizeof(u16))
      memcpy((u8 *)&expected + field->offset, &value, sizeof(value));
    else
      *((u8 *)&expected + field->offset) = value;
    previous_enabled = enabled;
  }

  check_busy_handoff = true;
  CHECK(ssf_mems_protocol_read_features(&serial, 0) == 0);
  CHECK(ssf_mems_protocol_get_features(&serial, &actual) == 0);
  CHECK(memcmp(&expected, &actual, sizeof(expected)) == 0);
  CHECK(read_runs == wanted_runs);
  for (i = 0; i < wanted_runs; i++) {
    CHECK(read_starts[i] == expected_starts[i]);
    CHECK(read_counts[i] == expected_counts[i]);
  }
  CHECK((wanted_count == 0) || saw_block_description);
  CHECK(!sensor.modbus_req.busy && !sensor.modbus_req.pending);

  CHECK(ssf_mems_protocol_decode_features(full_values, 29, &decoded) == 0);
  CHECK(memcmp(&expected, &decoded, sizeof(expected)) == 0);
  CHECK(ssf_mems_protocol_decode_features(full_values, 28, &decoded) == -EINVAL);

  if (wanted_runs) {
    fail_on_send = sends + (wanted_runs > 1 ? 2 : 1);
    CHECK(ssf_mems_protocol_read_features(&serial, 0) == -EIO);
    CHECK(memcmp(&sensor.protocol.features, &actual, sizeof(actual)) == 0);
    CHECK(!sensor.modbus_req.busy && !sensor.modbus_req.pending);
  }

  ssf_mems_protocol_invalidate_features(&sensor);
  CHECK(!sensor.protocol.valid);
  CHECK(memcmp(&sensor.protocol.features, &actual, sizeof(actual)) == 0);
  CHECK(ssf_mems_protocol_get_features(&serial, &decoded) == -ENODATA);
  fail_on_send = 0;
  read_runs = 0;
  CHECK(ssf_mems_protocol_read_features(&serial, 0) == 0);
  CHECK(ssf_mems_protocol_get_features(&serial, &decoded) == 0);
  CHECK(memcmp(&decoded, &expected, sizeof(expected)) == 0);
}

static void test_codecs(void) {
  struct ssf_modbus_transfer transfer = {0};
  u8 buf[256];
  u16 values[125] = {0x1122, 0xabcd, 0x8001};
  const u8 known_read[] = {1, 3, 0, 0, 0, 10, 0xc5, 0xcd};
  const struct ssf_modbus_frame_desc *exception = ssf_mems_modbus_find_rx_frame(0x83);

  transfer.frame = ssf_mems_modbus_find_frame(3);
  transfer.reg_count = 10;
  CHECK(ssf_mems_modbus_build_request(&transfer, 1, NULL, 0, buf, sizeof(buf)) == 8);
  CHECK(memcmp(buf, known_read, sizeof(known_read)) == 0);
  CHECK(ssf_mems_modbus_tx_frame_len(transfer.frame, 0) == -EINVAL);
  CHECK(ssf_mems_modbus_tx_frame_len(transfer.frame, 126) == -EINVAL);
  CHECK(ssf_mems_modbus_rx_frame_len(transfer.frame, 0) == -EMSGSIZE);
  CHECK(ssf_mems_modbus_rx_frame_len(transfer.frame, 3) == -EMSGSIZE);
  CHECK(ssf_mems_modbus_rx_frame_len(transfer.frame, 250) == 255);
  CHECK(ssf_mems_modbus_rx_frame_len(transfer.frame, 252) == -EMSGSIZE);
  CHECK(exception && ssf_mems_modbus_rx_frame_len(exception, 0) == 5);
  CHECK(ssf_mems_modbus_tx_frame_len(exception, 1) == -EOPNOTSUPP);
  CHECK(!ssf_mems_modbus_find_rx_frame(0x84));
  CHECK(ssf_mems_modbus_build_request(&transfer, 1, NULL, 0, buf, 7) == -ENOSPC);
  CHECK(ssf_mems_modbus_check_crc(NULL, 8) == -EINVAL);

  transfer.frame = ssf_mems_modbus_find_frame(0x10);
  transfer.protocol_addr = 0x003f;
  transfer.reg_count = 3;
  CHECK(ssf_mems_modbus_build_request(&transfer, 1, values, 125, buf, sizeof(buf)) == 15);
  CHECK(buf[2] == 0 && buf[3] == 0x3f && buf[4] == 0 && buf[5] == 3 && buf[6] == 6);
  CHECK(buf[7] == 0x11 && buf[8] == 0x22 && buf[9] == 0xab && buf[10] == 0xcd);
  CHECK(ssf_mems_modbus_check_crc(buf, 15) == 0);
  buf[8] ^= 1;
  CHECK(ssf_mems_modbus_check_crc(buf, 15) == -EBADMSG);
  transfer.reg_count = 123;
  CHECK(ssf_mems_modbus_build_request(&transfer, 1, values, 125, buf, sizeof(buf)) == 255);
  CHECK(buf[6] == 246);
  transfer.reg_count = 124;
  CHECK(ssf_mems_modbus_build_request(&transfer, 1, values, 125, buf, sizeof(buf)) == -EINVAL);
}

static void test_response_matching(void) {
  struct ssf_modbus_transfer transfer = {
    .frame = ssf_mems_modbus_find_frame(3),
    .display_reg = 40101, .protocol_addr = 100, .reg_count = 2,
  };
  u16 values[2] = {0};
  u8 read[] = {1, 3, 4, 0x11, 0x22, 0x33, 0x44, 0, 0};
  u8 single[] = {1, 6, 0, 100, 0x12, 0x34, 0, 0};
  u8 multi[] = {1, 0x10, 0, 100, 0, 2, 0, 0};

  finish_crc(read, sizeof(read));
  CHECK(ssf_mems_modbus_parse_response(&transfer, 1, 0, read, sizeof(read), values, 2) == 0);
  CHECK(values[0] == 0x1122 && values[1] == 0x3344);
  CHECK(ssf_mems_modbus_parse_response(&transfer, 2, 0, read, sizeof(read), values, 2) == -ENOMSG);
  CHECK(ssf_mems_modbus_parse_response(&transfer, 1, 0, read, sizeof(read) - 1, values, 2) == -EMSGSIZE);
  CHECK(ssf_mems_modbus_parse_response(&transfer, 1, 0, read, sizeof(read), values, 1) == -EFAULT);
  transfer.reg_count = 1;
  CHECK(ssf_mems_modbus_parse_response(&transfer, 1, 0, read, sizeof(read), values, 2) == -ENOMSG);

  transfer.frame = ssf_mems_modbus_find_frame(6);
  finish_crc(single, sizeof(single));
  CHECK(ssf_mems_modbus_parse_response(&transfer, 1, 0x1234, single, sizeof(single), NULL, 0) == 0);
  CHECK(ssf_mems_modbus_parse_response(&transfer, 1, 0x1235, single, sizeof(single), NULL, 0) == -ENOMSG);
  single[3]++;
  finish_crc(single, sizeof(single));
  CHECK(ssf_mems_modbus_parse_response(&transfer, 1, 0x1234, single, sizeof(single), NULL, 0) == -ENOMSG);

  transfer.frame = ssf_mems_modbus_find_frame(0x10);
  transfer.reg_count = 2;
  finish_crc(multi, sizeof(multi));
  CHECK(ssf_mems_modbus_parse_response(&transfer, 1, 0, multi, sizeof(multi), NULL, 0) == 0);
  multi[5]++;
  finish_crc(multi, sizeof(multi));
  CHECK(ssf_mems_modbus_parse_response(&transfer, 1, 0, multi, sizeof(multi), NULL, 0) == -ENOMSG);
}

static void test_requests(void) {
  u16 values[126] = {0x1122, 0xabcd, 0x8001};
  struct serdev_device no_driver = {0};
  const struct ssf_reg_desc *config = ssf_mems_modbus_find_reg(40050);
  const struct ssf_reg_desc *work = ssf_mems_modbus_find_reg(40064);
  struct ssf_modbus_transfer planned;
  const struct ssf_block_cmd_desc *block = ssf_mems_modbus_find_block_cmd(SSF_BLOCK_WRITE_WORK_PARAMETERS);

  reset_sensor();
  sensor.slave_id = 0x11;
  CHECK(ssf_mems_modbus_read(&serial, 40101, 3, values, 126, 0) == 0);
  CHECK(last_tx[0] == sensor.slave_id && sensor.modbus_req.slave_id == sensor.slave_id);
  CHECK(values[0] == wire_value(100) && values[2] == wire_value(102));
  CHECK(!saw_block_description);
  CHECK(ssf_mems_modbus_write_regs(&serial, 40101, 3, values, 126, 0) == 0);
  CHECK(last_tx_len == 15);
  CHECK(ssf_mems_modbus_write_reg(&serial, 40101, 0x1234, 0) == 0);
  CHECK(last_tx_len == 8 && last_tx[4] == 0x12 && last_tx[5] == 0x34);

  if (config)
    CHECK(ssf_mems_modbus_write_reg(&serial, 40050, 1, 0) == 0);
  else
    NO_SEND(ssf_mems_modbus_write_reg(&serial, 40050, 1, 0), -ENOENT);
  if (work) {
    sensor.protocol.host_baudrate = 2400;
    sensor.protocol.parity = SSF_MEMS_PARITY_NONE;
    CHECK(ssf_mems_protocol_write_work_parameters(&serial, values, 126, 0) == 0);
    CHECK(last_tx_len == 29 && last_tx[3] == 0x3c && last_tx[5] == 10 && last_tx[6] == 20);
    CHECK(serial.write_timeout == 221 && serial.wait_timeout == 221);
    sensor.protocol.parity = SSF_MEMS_PARITY_ODD;
    CHECK(ssf_mems_modbus_write_block(&serial, block, values, 126, 0) == 0);
    CHECK(serial.write_timeout == 233 && serial.wait_timeout == 233);
    NO_SEND(ssf_mems_modbus_plan_request(0x10, 40062, 2, block, &planned), -EINVAL);
  } else {
    NO_SEND(ssf_mems_protocol_write_work_parameters(&serial, values, 126, 0), -ENOENT);
  }

  NO_SEND(ssf_mems_modbus_read(&serial, 40059, 1, values, 126, 0),
          ssf_mems_modbus_find_reg(40059) == NULL ? -ENOENT : -EACCES);
  NO_SEND(ssf_mems_modbus_read(&serial, 40054, 1, values, 126, 0), -EOPNOTSUPP);
  NO_SEND(ssf_mems_modbus_read(&serial, 40103, 2, values, 126, 0), -ENOENT);
  NO_SEND(ssf_mems_modbus_write_regs(&serial, 40121, 1, values, 126, 0), -EACCES);
  NO_SEND(ssf_mems_modbus_write_regs(&serial, 40053, 2, values, 126, 0), -EACCES);
  NO_SEND(ssf_mems_modbus_write_reg(&serial, 40061, 1, 0), -EOPNOTSUPP);
  NO_SEND(ssf_mems_modbus_write_regs(&serial, 65535, 2, values, 126, 0), -EINVAL);
  NO_SEND(ssf_mems_modbus_read(&serial, 40101, 0, values, 126, 0), -EINVAL);
  NO_SEND(ssf_mems_modbus_read(&serial, 40101, 126, values, 126, 0), -EINVAL);
  NO_SEND(ssf_mems_modbus_write_regs(&serial, 40101, 124, values, 126, 0), -EINVAL);
  NO_SEND(ssf_mems_modbus_write_regs(&serial, 40101, 2, values, 1, 0), -EINVAL);
  NO_SEND(ssf_mems_modbus_read(NULL, 40101, 1, values, 126, 0), -EINVAL);
  NO_SEND(ssf_mems_modbus_read(&no_driver, 40101, 1, values, 126, 0), -ENODEV);
  NO_SEND(ssf_mems_protocol_write_work_parameters(&serial, values, 9, 0), -EINVAL);
  NO_SEND(ssf_mems_modbus_write_block(&serial, NULL, values, 126, 0), -EINVAL);
  NO_SEND(ssf_mems_modbus_write_block(&serial, block, values, 9, 0), -EINVAL);

  sensor.modbus_req.busy = true;
  NO_SEND(ssf_mems_modbus_read_reg(&serial, 40121, values, 0), -EBUSY);
  sensor.modbus_req.busy = false;
  sensor.modbus_req.shutting_down = true;
  NO_SEND(ssf_mems_modbus_read_reg(&serial, 40121, values, 0), -ENODEV);
  sensor.modbus_req.shutting_down = false;

  test_allocation_failure = true;
  NO_SEND(ssf_mems_modbus_read_reg(&serial, 40121, values, 0), -ENOMEM);
  CHECK(!sensor.modbus_req.busy);
  test_allocation_failure = false;
  forced_send_result = 3;
  CHECK(ssf_mems_modbus_read_reg(&serial, 40121, values, 0) == -EIO);
  CHECK(!sensor.modbus_req.busy && !sensor.modbus_req.pending);
  forced_send_result = -1;
  response_mode = REPLY_EXCEPTION;
  CHECK(ssf_mems_modbus_read_reg(&serial, 40121, values, 0) == -EREMOTEIO);
  CHECK(!sensor.modbus_req.busy && !sensor.modbus_req.pending);
  response_mode = REPLY_BAD_CRC;
  CHECK(ssf_mems_modbus_read_reg(&serial, 40121, values, 0) == -ETIMEDOUT);
  CHECK(!sensor.modbus_req.busy && !sensor.modbus_req.pending);
  response_mode = REPLY_WRONG_COUNT;
  CHECK(ssf_mems_modbus_read(&serial, 40101, 2, values, 126, 0) == -ETIMEDOUT);
  CHECK(!sensor.modbus_req.busy && !sensor.modbus_req.pending);
  response_mode = REPLY_NONE;
  test_wait_result = -1;
  CHECK(ssf_mems_modbus_read_reg(&serial, 40121, values, 0) == -ERESTARTSYS);
  CHECK(!sensor.modbus_req.busy && !sensor.modbus_req.pending);
}

/* 未投入使用的启动夹具：init 应自行设置默认值并解析属性。 */
static void reset_startup_configuration(void) {
  cleanup_sensor();
  memset(&sensor, 0, sizeof(sensor));
  memset(&serial, 0, sizeof(serial));
  serial.drvdata = &sensor;
  sensor.serdev = &serial;
  sends = 0;
}

static void test_serial_configuration(void) {
  static const struct {
    const char *name;
    enum ssf_mems_parity protocol_parity;
    enum serdev_parity host_parity;
  } parity_cases[] = {
      {"none", SSF_MEMS_PARITY_NONE, SERDEV_PARITY_NONE},
      {"odd", SSF_MEMS_PARITY_ODD, SERDEV_PARITY_ODD},
      {"even", SSF_MEMS_PARITY_EVEN, SERDEV_PARITY_EVEN},
  };
  struct ssf_mems_xyzs_data no_serial = {0};
  size_t i;

  CHECK(ssf_mems_protocol_init(NULL) == -EINVAL);
  CHECK(ssf_mems_protocol_init(&no_serial) == -EINVAL);
  CHECK(ssf_mems_protocol_configure_serial(NULL) == -EINVAL);
  CHECK(ssf_mems_protocol_configure_serial(&no_serial) == -EINVAL);

  reset_startup_configuration();
  CHECK(ssf_mems_protocol_init(&sensor) == 0);
  CHECK(sensor.slave_id == 1 &&
        sensor.protocol.baudrate == SSF_MEMS_BAUDRATE_DEFAULT &&
        sensor.protocol.parity == SSF_MEMS_PARITY_NONE);
  CHECK(sensor.protocol.host_baudrate == 9600 && serial.dev.info_count == 3);
  CHECK(strstr(serial.dev.last_info, "default parity none") != NULL);
  serial.flow_control = true;
  CHECK(ssf_mems_protocol_configure_serial(&sensor) == 0);
  CHECK(serial.baudrate == 9600 && sensor.protocol.host_baudrate == 9600);
  CHECK(serial.parity == SERDEV_PARITY_NONE && !serial.flow_control);
  CHECK(sends == 0);

  for (i = 0; i < ARRAY_SIZE(parity_cases); i++) {
    reset_startup_configuration();
    serial.dev.has_slave_id = serial.dev.has_current_speed =
        serial.dev.has_parity = true;
    serial.dev.slave_id = 247;
    serial.dev.current_speed = 115200;
    serial.dev.parity = parity_cases[i].name;
    CHECK(ssf_mems_protocol_init(&sensor) == 0);
    CHECK(sensor.slave_id == 247 &&
          sensor.protocol.baudrate == SSF_MEMS_BAUDRATE_115200 &&
          sensor.protocol.host_baudrate == 115200 &&
          sensor.protocol.parity == parity_cases[i].protocol_parity);
    CHECK(serial.baudrate_set_count == 0 && serial.parity_set_count == 0);
    CHECK(serial.dev.info_count == 0);
    CHECK(ssf_mems_protocol_configure_serial(&sensor) == 0);
    CHECK(serial.baudrate == 115200 &&
          serial.parity == parity_cases[i].host_parity);
    CHECK(serial.baudrate_set_count == 1 && serial.parity_set_count == 1);
    CHECK(sends == 0);
  }

  reset_startup_configuration();
  serial.dev.has_slave_id = true;
  serial.dev.slave_id = 0;
  CHECK(ssf_mems_protocol_init(&sensor) == -EINVAL);
  serial.dev.slave_id = 248;
  CHECK(ssf_mems_protocol_init(&sensor) == -EINVAL);

  reset_startup_configuration();
  serial.dev.has_current_speed = true;
  serial.dev.current_speed = 12345;
  CHECK(ssf_mems_protocol_init(&sensor) == 0);
  CHECK(sensor.protocol.baudrate == SSF_MEMS_BAUDRATE_DEFAULT &&
        sensor.protocol.host_baudrate == 9600);
  CHECK(ssf_mems_protocol_configure_serial(&sensor) == 0 && serial.baudrate == 9600);
  CHECK(sends == 0);
  serial.dev.current_speed = 0;
  CHECK(ssf_mems_protocol_init(&sensor) == 0);
  CHECK(sensor.protocol.host_baudrate == 9600);

  reset_startup_configuration();
  serial.dev.has_parity = true;
  serial.dev.parity = "invalid";
  CHECK(ssf_mems_protocol_init(&sensor) == -EINVAL);
  CHECK(sensor.protocol.parity == SSF_MEMS_PARITY_NONE);

  /* 单个属性读取失败不影响其他合法属性，也必须记录对应默认值。 */
  for (i = 0; i < 3; i++) {
    reset_startup_configuration();
    serial.dev.has_slave_id = serial.dev.has_current_speed =
        serial.dev.has_parity = true;
    serial.dev.slave_id = 42;
    serial.dev.current_speed = 115200;
    serial.dev.parity = "odd";
    if (i == 0)
      serial.dev.slave_id_read_error = -EIO;
    else if (i == 1)
      serial.dev.current_speed_read_error = -EINVAL;
    else
      serial.dev.parity_read_error = -ENODATA;
    CHECK(ssf_mems_protocol_init(&sensor) == 0);
    CHECK(sensor.slave_id == (i == 0 ? 1 : 42));
    CHECK(sensor.protocol.baudrate == (i == 1 ? SSF_MEMS_BAUDRATE_DEFAULT
                                             : SSF_MEMS_BAUDRATE_115200));
    CHECK(sensor.protocol.host_baudrate == (i == 1 ? 9600 : 115200));
    CHECK(sensor.protocol.parity == (i == 2 ? SSF_MEMS_PARITY_NONE
                                           : SSF_MEMS_PARITY_ODD));
    CHECK(serial.dev.info_count == 1);
    CHECK(strstr(serial.dev.last_info, i == 0 ? "default slave ID 1"
                                      : i == 1 ? "default baud rate 9600"
                                               : "default parity none") != NULL);
    CHECK(serial.baudrate_set_count == 0 && serial.parity_set_count == 0);
    CHECK(ssf_mems_protocol_configure_serial(&sensor) == 0);
    CHECK(serial.baudrate == sensor.protocol.host_baudrate);
    CHECK(serial.parity == (i == 2 ? SERDEV_PARITY_NONE : SERDEV_PARITY_ODD));
    CHECK(sends == 0);
  }

  reset_startup_configuration();
  serial.dev.has_slave_id = serial.dev.has_current_speed =
      serial.dev.has_parity = true;
  serial.dev.slave_id_read_error = serial.dev.current_speed_read_error =
      serial.dev.parity_read_error = -EIO;
  CHECK(ssf_mems_protocol_init(&sensor) == 0);
  CHECK(sensor.slave_id == 1 && sensor.protocol.host_baudrate == 9600 &&
        sensor.protocol.parity == SSF_MEMS_PARITY_NONE);
  CHECK(serial.dev.info_count == 3 && sends == 0);

  reset_startup_configuration();
  serial.dev.has_current_speed = true;
  serial.dev.current_speed = 57600;
  CHECK(ssf_mems_protocol_init(&sensor) == 0);
  CHECK(sensor.slave_id == 1 && sensor.protocol.host_baudrate == 57600 &&
        sensor.protocol.parity == SSF_MEMS_PARITY_NONE);
  CHECK(serial.dev.info_count == 2 && sends == 0);

  reset_sensor();
  sensor.protocol.parity = SSF_MEMS_PARITY_MAX;
  CHECK(ssf_mems_protocol_configure_serial(&sensor) == -EINVAL);
  CHECK(serial.baudrate_set_count == 0 && serial.parity_set_count == 0);
  sensor.protocol.parity = SSF_MEMS_PARITY_NONE;
  sensor.protocol.baudrate = SSF_MEMS_BAUDRATE_MAX;
  CHECK(ssf_mems_protocol_configure_serial(&sensor) == -EINVAL);
  CHECK(serial.baudrate_set_count == 0 && serial.parity_set_count == 0);

  reset_sensor();
  serial.baudrate_set_failure = true;
  CHECK(ssf_mems_protocol_configure_serial(&sensor) == -EIO);
  CHECK(serial.parity_set_count == 0 && sensor.protocol.host_baudrate == 9600);

  reset_sensor();
  sensor.protocol.baudrate = SSF_MEMS_BAUDRATE_115200;
  serial.actual_baudrate = 115107;
  CHECK(ssf_mems_protocol_configure_serial(&sensor) == 0);
  CHECK(sensor.protocol.host_baudrate == 115107);

  reset_sensor();
  sensor.protocol.baudrate = SSF_MEMS_BAUDRATE_57600;
  serial.rejected_baudrate = 57600;
  CHECK(ssf_mems_protocol_configure_serial(&sensor) == 0);
  CHECK(sensor.protocol.baudrate == SSF_MEMS_BAUDRATE_DEFAULT &&
        sensor.protocol.host_baudrate == 9600 && serial.baudrate == 9600);
  CHECK(serial.baudrate_set_count == 2 && sends == 0);
  CHECK(strstr(serial.dev.last_info, "using default baud rate 9600") != NULL);

  reset_sensor();
  serial.parity_set_error = -EOPNOTSUPP;
  CHECK(ssf_mems_protocol_configure_serial(&sensor) == -EOPNOTSUPP);
  CHECK(serial.baudrate_set_count == 1 && serial.parity_set_count == 1);
  CHECK(sends == 0);
}

static void test_baudrate_management(void) {
  enum ssf_mems_baudrate baudrate = SSF_MEMS_BAUDRATE_MAX;
  unsigned int sends_before;

  reset_sensor();
  enforce_sensor_baudrate = true;
  sensor_baudrate = SSF_MEMS_BAUDRATE_9600;
  CHECK(ssf_mems_baudrate_to_value(SSF_MEMS_BAUDRATE_DEFAULT) == 9600);
  CHECK(ssf_mems_baudrate_to_value(SSF_MEMS_BAUDRATE_38400) == 38400);
  CHECK(ssf_mems_baudrate_to_value(SSF_MEMS_BAUDRATE_MAX) == -EINVAL);
  CHECK(ssf_mems_baudrate_from_value(9600, &baudrate) == 0 &&
        baudrate == SSF_MEMS_BAUDRATE_9600);
  CHECK(ssf_mems_baudrate_from_value(12345, &baudrate) == -EINVAL);
  CHECK(ssf_mems_baudrate_from_value(9600, NULL) == -EINVAL);
  CHECK(ssf_mems_protocol_get_baudrate(&serial, &baudrate, 0) == 0);
  CHECK(baudrate == SSF_MEMS_BAUDRATE_9600);
  CHECK(sensor.protocol.baudrate == SSF_MEMS_BAUDRATE_9600);
  CHECK(serial.baudrate == 9600 && serial.baudrate_set_count == 0);
  CHECK(sends == 1);

  if (!ssf_mems_modbus_find_reg(40110)) {
    NO_SEND(ssf_mems_protocol_set_baudrate(&serial, SSF_MEMS_BAUDRATE_115200, 0),
            -ENOENT);
    CHECK(staged_baudrate == SSF_MEMS_BAUDRATE_MAX && !reboots);
    return;
  }

  reset_sensor();
  enforce_sensor_baudrate = true;
  sensor_baudrate = SSF_MEMS_BAUDRATE_38400;
  baudrate = SSF_MEMS_BAUDRATE_MAX;
  CHECK(ssf_mems_protocol_get_baudrate(&serial, &baudrate, 0) == 0);
  CHECK(baudrate == SSF_MEMS_BAUDRATE_38400);
  CHECK(sensor.protocol.baudrate == SSF_MEMS_BAUDRATE_38400);
  CHECK(serial.baudrate == 38400 && serial.baudrate_set_count == 4);
  /* 当前 9600 只试一次：DEFAULT 和显式 9600 不重复扫描。 */
  CHECK(sends == 5);

  sends_before = sends;
  CHECK(ssf_mems_protocol_set_baudrate(&serial, SSF_MEMS_BAUDRATE_115200,
                                       0) == 0);
  CHECK(sends == sends_before + 3);
  CHECK(sensor_baudrate == SSF_MEMS_BAUDRATE_115200);
  CHECK(sensor.protocol.baudrate == SSF_MEMS_BAUDRATE_115200);
  CHECK(serial.baudrate == 115200);
  CHECK(tx_events[sends_before].function == SSF_MEMS_MODBUS_FUNC_WRITE_SINGLE &&
        tx_events[sends_before].address == 101 &&
        tx_events[sends_before].value == SSF_MEMS_BAUDRATE_115200 &&
        tx_events[sends_before].baudrate == 38400);
  CHECK(tx_events[sends_before + 1].function == SSF_MEMS_MODBUS_FUNC_WRITE_SINGLE &&
        tx_events[sends_before + 1].address == 109 &&
        tx_events[sends_before + 1].value == 1 &&
        tx_events[sends_before + 1].baudrate == 38400);
  CHECK(tx_events[sends_before + 2].function == SSF_MEMS_MODBUS_FUNC_READ &&
        tx_events[sends_before + 2].baudrate == 115200);
  CHECK(tx_events[sends_before + 2].sent_at >=
        tx_events[sends_before + 1].sent_at + SSF_MEMS_SENSOR_REBOOT_DELAY_MS);
  CHECK(sleep_count == 1 && slept_ms == SSF_MEMS_SENSOR_REBOOT_DELAY_MS && reboots == 1);

  sends_before = sends;
  NO_SEND(ssf_mems_protocol_set_baudrate(
              &serial, (enum ssf_mems_baudrate)SSF_MEMS_BAUDRATE_MAX, 0),
          -EINVAL);
  CHECK(sends == sends_before);

  response_mode = REPLY_EXCEPTION;
  sends_before = sends;
  CHECK(ssf_mems_protocol_set_baudrate(&serial, SSF_MEMS_BAUDRATE_57600, 0) ==
        -EREMOTEIO);
  CHECK(sends == sends_before + 1);
  CHECK(sensor_baudrate == SSF_MEMS_BAUDRATE_115200);
  CHECK(sensor.protocol.baudrate == SSF_MEMS_BAUDRATE_115200);
  CHECK(serial.baudrate == 115200);

  reset_sensor();
  enforce_sensor_baudrate = true;
  sensor_baudrate = SSF_MEMS_BAUDRATE_9600;
  invalid_baudrate_value = true;
  baudrate = SSF_MEMS_BAUDRATE_MAX;
  CHECK(ssf_mems_protocol_get_baudrate(&serial, &baudrate, 0) == -EPROTO);
  CHECK(baudrate == SSF_MEMS_BAUDRATE_MAX);
  CHECK(sends == 1 && serial.baudrate == 9600 &&
        serial.baudrate_set_count == 0);

  reset_sensor();
  response_mode = REPLY_NONE;
  baudrate = SSF_MEMS_BAUDRATE_MAX;
  CHECK(ssf_mems_protocol_get_baudrate(&serial, &baudrate, 0) == -ETIMEDOUT);
  CHECK(baudrate == SSF_MEMS_BAUDRATE_MAX);
  CHECK(sensor.protocol.baudrate == SSF_MEMS_BAUDRATE_DEFAULT);
  CHECK(serial.baudrate == 9600);
  /* 17 个不同实际速率：入口试 1 次，其余 16 个各试 1 次。 */
  CHECK(sends == 17);
  CHECK(serial.baudrate_set_count == 17); /* 16 次扫描 + 1 次恢复 */

  NO_SEND(ssf_mems_protocol_get_baudrate(NULL, &baudrate, 0), -EINVAL);
  NO_SEND(ssf_mems_protocol_get_baudrate(&serial, NULL, 0), -EINVAL);
}

static void shutdown_during_reboot(void) {
  CHECK(serial.baudrate == 9600 && sends == 2);
  ssf_mems_modbus_request_remove(&sensor);
}

static void shutdown_during_verification_retry(void) {
  if (sends > 2)
    ssf_mems_modbus_request_remove(&sensor);
}

static void test_baudrate_reboot(void) {
  enum ssf_mems_baudrate found;
  unsigned long before;
  size_t i;
  const int uncertain_modes[] = {REPLY_NONE, REPLY_BAD_CRC};

  reset_sensor();
  enforce_sensor_baudrate = true;
  sensor_baudrate = SSF_MEMS_BAUDRATE_9600;
  if (!ssf_mems_modbus_find_reg(40110)) {
    NO_SEND(ssf_mems_protocol_set_baudrate(&serial, SSF_MEMS_BAUDRATE_115200, 50),
            -ENOENT);
    return;
  }

  /* 普通写 40102 只暂存；读取接口不得据此宣布速率已切换。 */
  CHECK(ssf_mems_modbus_write_reg(&serial, 40102, SSF_MEMS_BAUDRATE_115200, 50) == 0);
  CHECK(sensor_baudrate == SSF_MEMS_BAUDRATE_9600 && !reboots);
  CHECK(staged_baudrate == SSF_MEMS_BAUDRATE_115200 && serial.baudrate == 9600);
  CHECK(ssf_mems_protocol_get_baudrate(&serial, &found, 50) == 0);
  CHECK(ssf_mems_baudrate_to_value(found) == 9600);

  /* 40102 已暂存但它的应答丢失：不得继续保存或提前切换主机。 */
  reset_sensor();
  enforce_sensor_baudrate = true;
  special_reply_send = 1;
  special_reply_mode = REPLY_NONE;
  CHECK(ssf_mems_protocol_set_baudrate(&serial, SSF_MEMS_BAUDRATE_115200, 50) == -ETIMEDOUT);
  CHECK(sends == 1 && !reboots && !sleep_count && serial.baudrate == 9600);
  CHECK(staged_baudrate == SSF_MEMS_BAUDRATE_115200);
  special_reply_send = 0;
  CHECK(ssf_mems_protocol_get_baudrate(&serial, &found, 50) == 0);
  CHECK(ssf_mems_baudrate_to_value(found) == 9600);

  reset_sensor();
  enforce_sensor_baudrate = true;
  sensor.protocol.valid = true;
  before = jiffies;
  CHECK(ssf_mems_protocol_set_baudrate(&serial, SSF_MEMS_BAUDRATE_2400, 50) == 0);
  CHECK(!sensor.protocol.valid && jiffies >= before + SSF_MEMS_SENSOR_REBOOT_DELAY_MS);
  CHECK(sends == 3 && reboots == 1 && sensor_baudrate == SSF_MEMS_BAUDRATE_2400);
  CHECK(tx_events[0].baudrate == 9600 && tx_events[1].baudrate == 9600);
  CHECK(tx_events[1].address == 109 && tx_events[1].value == 1);
  CHECK(tx_events[2].baudrate == 2400 &&
        tx_events[2].sent_at >= sensor_reboot_until);

  reset_sensor();
  enforce_sensor_baudrate = true;
  special_reply_send = 2;
  special_reply_mode = REPLY_EXCEPTION;
  CHECK(ssf_mems_protocol_set_baudrate(&serial, SSF_MEMS_BAUDRATE_115200, 50) == -EREMOTEIO);
  CHECK(sends == 2 && !reboots && !sleep_count && serial.baudrate == 9600);
  CHECK(staged_baudrate == SSF_MEMS_BAUDRATE_115200);
  CHECK(ssf_mems_protocol_get_baudrate(&serial, &found, 50) == 0);
  CHECK(ssf_mems_baudrate_to_value(found) == 9600);

  /* 丢失/损坏保存回显：设备可能已重启，但接口必须报错并保留静默期。 */
  for (i = 0; i < ARRAY_SIZE(uncertain_modes); i++) {
    reset_sensor();
    enforce_sensor_baudrate = true;
    special_reply_send = 2;
    special_reply_mode = uncertain_modes[i];
    CHECK(ssf_mems_protocol_set_baudrate(&serial, SSF_MEMS_BAUDRATE_115200, 50) == -ETIMEDOUT);
    CHECK(sends == 2 && reboots == 1 && sleep_count == 1);
    CHECK(jiffies >= sensor_reboot_until && serial.baudrate == 9600);
    CHECK(sensor.protocol.baudrate == SSF_MEMS_BAUDRATE_DEFAULT);
    special_reply_send = 0;
    CHECK(ssf_mems_protocol_get_baudrate(&serial, &found, 50) == 0);
    CHECK(found == SSF_MEMS_BAUDRATE_115200 && serial.baudrate == 115200);
    CHECK(reboots == 1); /* 恢复只读，不自动重发保存命令。 */
  }

  reset_sensor();
  enforce_sensor_baudrate = true;
  special_reply_send = 3;
  special_reply_mode = REPLY_NONE;
  CHECK(ssf_mems_protocol_set_baudrate(&serial, SSF_MEMS_BAUDRATE_115200, 50) == 0);
  CHECK(sends == 4 && reboots == 1 && sleep_count == 2);
  CHECK(serial.baudrate == 115200 && sensor.protocol.baudrate == SSF_MEMS_BAUDRATE_115200);
  CHECK(tx_events[3].function == 3 && tx_events[3].address == 101);

  /* 实际启动时间比最小静默期长：只读重试到可通信，不再保存/重启。 */
  reset_sensor();
  enforce_sensor_baudrate = true;
  sensor_baudrate = sensor.protocol.baudrate = SSF_MEMS_BAUDRATE_115200;
  sensor.protocol.host_baudrate = serial.baudrate = 115200;
  sensor_extra_boot_ms = 2500;
  CHECK(ssf_mems_protocol_set_baudrate(&serial, SSF_MEMS_BAUDRATE_9600, 200) == 0);
  CHECK(sensor_baudrate == SSF_MEMS_BAUDRATE_9600 && serial.baudrate == 9600);
  CHECK(reboots == 1 && sends > 3 && sends <= 2 + SSF_MEMS_BAUDRATE_VERIFY_ATTEMPTS);
  CHECK(tx_events[sends - 1].sent_at >= sensor_reboot_until);
  CHECK(tx_events[2].sent_at < sensor_reboot_until);
  for (i = 2; i < sends; i++)
    CHECK(tx_events[i].function == 3 && tx_events[i].address == 101 &&
          tx_events[i].baudrate == 9600);

  /* 一直无应答必须有限结束，不能将超时或目标 UART 设置当成成功。 */
  reset_sensor();
  enforce_sensor_baudrate = true;
  verification_never_replies = true;
  CHECK(ssf_mems_protocol_set_baudrate(&serial, SSF_MEMS_BAUDRATE_115200, 50) == -ETIMEDOUT);
  CHECK(sends == 2 + SSF_MEMS_BAUDRATE_VERIFY_ATTEMPTS && reboots == 1);
  CHECK(sleep_count == SSF_MEMS_BAUDRATE_VERIFY_ATTEMPTS && !sensor.protocol.valid);
  CHECK(serial.baudrate == 115200 && sensor.protocol.baudrate == SSF_MEMS_BAUDRATE_115200);
  for (i = 2; i < sends; i++)
    CHECK(tx_events[i].function == 3 && tx_events[i].address == 101);

  /* 合法异常应答不代表启动尚未完成，不进行重试。 */
  reset_sensor();
  enforce_sensor_baudrate = true;
  special_reply_send = 3;
  special_reply_mode = REPLY_EXCEPTION;
  CHECK(ssf_mems_protocol_set_baudrate(&serial, SSF_MEMS_BAUDRATE_115200, 50) == -EREMOTEIO);
  CHECK(sends == 3 && reboots == 1 && sleep_count == 1);

  reset_sensor();
  enforce_sensor_baudrate = true;
  verification_never_replies = true;
  sleep_hook = shutdown_during_verification_retry;
  CHECK(ssf_mems_protocol_set_baudrate(&serial, SSF_MEMS_BAUDRATE_115200, 50) == -ENODEV);
  CHECK(sends == 3 && reboots == 1 && sleep_count == 2);

  reset_sensor();
  enforce_sensor_baudrate = true;
  invalid_baudrate_value = true;
  CHECK(ssf_mems_protocol_set_baudrate(&serial, SSF_MEMS_BAUDRATE_115200, 50) == -EPROTO);
  CHECK(sends == 3 && serial.baudrate == 115200 && reboots == 1);

  reset_sensor();
  enforce_sensor_baudrate = true;
  wrong_baudrate_value = true;
  CHECK(ssf_mems_protocol_set_baudrate(&serial, SSF_MEMS_BAUDRATE_115200, 50) == -EPROTO);
  CHECK(sends == 3 && serial.baudrate == 115200 && reboots == 1);

  reset_sensor();
  enforce_sensor_baudrate = true;
  serial.baudrate_set_failure = true;
  CHECK(ssf_mems_protocol_set_baudrate(&serial, SSF_MEMS_BAUDRATE_115200, 50) == -EIO);
  CHECK(sends == 2 && sleep_count == 1 && sensor_baudrate == SSF_MEMS_BAUDRATE_115200);
  CHECK(serial.baudrate == 9600);

  reset_sensor();
  enforce_sensor_baudrate = true;
  sleep_hook = shutdown_during_reboot;
  CHECK(ssf_mems_protocol_set_baudrate(&serial, SSF_MEMS_BAUDRATE_115200, 50) == -ENODEV);
  CHECK(sends == 2 && serial.baudrate_set_count == 0 && sleep_count == 1);
}

static void feed_bytes(const u8 *bytes, size_t len) {
  CHECK(ssf_mems_rx_push(&serial, bytes, len) == (ssize_t)len);
  sensor.modbus_rx.work.fn(&sensor.modbus_rx.work);
}

/* 旧请求 40102 和新请求 40121 的应答形状完全相同。旧值不得写入新输出。 */
static void deliver_late_response(unsigned long timeout) {
  u8 late[] = {1, 3, 2, 0, 3, 0, 0};
  unsigned int before = sends;

  CHECK(sensor.modbus_req.busy && !sensor.modbus_req.pending);
  CHECK(timeout > 10);
  jiffies += timeout - 1;
  finish_crc(late, sizeof(late));
  feed_bytes(late, sizeof(late));
  CHECK(sends == before && !sensor.modbus_req.pending);
  wait_hook = NULL;
}

static void deliver_noise(unsigned long timeout) {
  const u8 noise = 0xff;
  CHECK(!sensor.modbus_req.pending);
  jiffies += timeout;
  feed_bytes(&noise, 1);
}

static void remove_during_guard(unsigned long timeout) {
  (void)timeout;
  ssf_mems_modbus_request_remove(&sensor);
  wait_hook = NULL;
}

static void test_request_isolation(void) {
  u16 value = 0xbeef;
  unsigned long failed_at, before;
  const u8 leftovers[] = {1, 3, 250, 1, 3, 250, 1, 3, 250, 1, 3, 250};

  reset_sensor();
  response_mode = REPLY_NONE;
  CHECK(ssf_mems_modbus_read_reg(&serial, 40102, &value, 40) == -ETIMEDOUT);
  CHECK(value == 0xbeef);
  failed_at = jiffies;
  response_mode = REPLY_NORMAL;
  wait_hook = deliver_late_response;
  CHECK(ssf_mems_modbus_read_reg(&serial, 40121, &value, 40) == 0);
  CHECK(value == wire_value(120) && sends == 2);
  CHECK(jiffies >= failed_at + SSF_MEMS_MODBUS_RECOVERY_GUARD_MS + 3);
  CHECK(!sensor.modbus_req.busy && !sensor.modbus_req.pending);

  reset_sensor();
  forced_send_result = 3;
  CHECK(ssf_mems_modbus_read_reg(&serial, 40102, &value, 1500) == -EIO);
  before = jiffies;
  forced_send_result = -1;
  CHECK(ssf_mems_modbus_read_reg(&serial, 40121, &value, 40) == 0);
  CHECK(jiffies >= before + 1500 && value == wire_value(120));

  reset_sensor();
  feed_bytes(leftovers, sizeof(leftovers));
  CHECK(ssf_mems_modbus_read_reg(&serial, 40121, &value, 40) == 0);
  CHECK(value == wire_value(120));
  check_released_slots(&sensor);

  reset_sensor();
  wait_hook = deliver_noise;
  NO_SEND(ssf_mems_modbus_read_reg(&serial, 40121, &value, 40), -ETIMEDOUT);
  CHECK(jiffies == SSF_MEMS_MODBUS_IDLE_WAIT_MS);
  CHECK(!sensor.modbus_req.busy && !sensor.modbus_req.pending);

  reset_sensor();
  sensor.modbus_req.quarantine_until = jiffies + 1000;
  wait_hook = remove_during_guard;
  NO_SEND(ssf_mems_modbus_read_reg(&serial, 40121, &value, 40), -ENODEV);
  CHECK(!sensor.modbus_req.busy && !sensor.modbus_req.pending);

  reset_sensor();
  test_wait_result = -ERESTARTSYS;
  NO_SEND(ssf_mems_modbus_read_reg(&serial, 40121, &value, 40), -ERESTARTSYS);
  CHECK(!sensor.modbus_req.busy);
}

static void test_receive_expiry(void) {
  const u8 leftovers[] = {1, 3, 250, 1, 3, 250, 1, 3, 250, 1, 3, 250};
  u8 response[] = {1, 3, 2, 0x12, 0x34, 0, 0};
  struct callback_capture capture = {0};
  unsigned long expiry;
  size_t i;

  reset_sensor();
  sensor.modbus_rx.handler = capture_frame;
  sensor.modbus_rx.handler_context = &capture;
  finish_crc(response, sizeof(response));
  feed_bytes(leftovers, sizeof(leftovers));
  for (i = 0; i < ARRAY_SIZE(sensor.modbus_rx.frame); i++)
    CHECK(atomic_read(&sensor.modbus_rx.frame[i].in_use) == SSF_MEMS_FRAME_SLOT_USED);
  expiry = sensor.modbus_rx.expiry_work.delay;
  jiffies = expiry - 1;
  sensor.modbus_rx.expiry_work.work.fn(&sensor.modbus_rx.expiry_work.work);
  CHECK(sensor.modbus_rx.frame[0].data != NULL);
  jiffies = expiry;
  sensor.modbus_rx.expiry_work.work.fn(&sensor.modbus_rx.expiry_work.work);
  check_released_slots(&sensor);
  feed_bytes(response, sizeof(response));
  CHECK(capture.calls == 1 && capture.len == sizeof(response));

  /* 即使定时任务尚未执行，新字节也会主动回收超过绝对寿命的候选帧。 */
  ssf_mems_modbus_receive_flush(&sensor);
  feed_bytes(leftovers, sizeof(leftovers));
  jiffies += expiry;
  feed_bytes(response, sizeof(response));
  CHECK(capture.calls == 2);

  /* 较低波特率按实际最长帧传输时间预留预算，分批交付的正常帧保留。 */
  ssf_mems_modbus_receive_flush(&sensor);
  sensor.protocol.host_baudrate = 2400;
  feed_bytes(response, 3);
  expiry = sensor.modbus_rx.expiry_work.delay;
  CHECK(expiry >= 1067 + SSF_MEMS_RX_EXPIRY_MARGIN_MS);
  jiffies += expiry - 1;
  sensor.modbus_rx.expiry_work.work.fn(&sensor.modbus_rx.expiry_work.work);
  feed_bytes(response + 3, sizeof(response) - 3);
  CHECK(capture.calls == 3);
  CHECK(memcmp(capture.bytes, response, sizeof(response)) == 0);
}

static void test_feature_settings(void) {
  u16 value = 999;
  unsigned int before;

  reset_sensor();
  emulate_settings = true;
  setting_registers[51] = 0;
  setting_registers[52] = 2;
  setting_registers[69] = 0x03a5;
  setting_registers[120] = 5100;
  CHECK(ssf_mems_protocol_read_setting(&serial, SSF_MEMS_SETTING_FIRMWARE_VERSION,
                                       &value, 0) == 0 && value == 5100);
  CHECK(ssf_mems_protocol_read_setting(&serial, SSF_MEMS_SETTING_SAMPLING_LENGTH,
                                       &value, 0) == 0 && value == 2);
  CHECK(ssf_mems_protocol_write_setting(&serial, SSF_MEMS_SETTING_SAMPLING_RATE,
                                        6, 0) == 0);
  CHECK(setting_registers[51] == 6 && setting_registers[52] == 2);
  CHECK(tx_events[sends - 2].function == 6 &&
        tx_events[sends - 2].address == 51 && tx_events[sends - 2].value == 6);
  before = sends;
  CHECK(ssf_mems_protocol_write_setting(&serial, SSF_MEMS_SETTING_SAMPLING_RATE,
                                        6, 0) == 0);
  CHECK(sends == before + 1); /* 相同采样率只读一次，避免重复触发固件 Flash 保存。 */
  CHECK(ssf_mems_protocol_write_setting(&serial, SSF_MEMS_SETTING_SAMPLING_RATE,
                                        SSF_MEMS_SAMPLING_RATE_MAX_INDEX, 0) == 0);
  CHECK(setting_registers[51] == 9 && setting_registers[52] == 2);
  CHECK(tx_events[sends - 2].function == 6 &&
        tx_events[sends - 2].address == 51 && tx_events[sends - 2].value == 9);
  before = sends;
  CHECK(ssf_mems_protocol_write_setting(&serial, SSF_MEMS_SETTING_SAMPLING_RATE,
                                        SSF_MEMS_SAMPLING_RATE_MAX_INDEX, 0) == 0);
  CHECK(sends == before + 1);
  CHECK(ssf_mems_protocol_write_setting(&serial, SSF_MEMS_SETTING_FEATURE_ENABLE,
                                        63, 0) == 0);
  CHECK(setting_registers[69] == 0xffa5); /* 全开特征，但保留报警/关系/保留位。 */
  CHECK(ssf_mems_protocol_read_setting(&serial, SSF_MEMS_SETTING_FEATURE_ENABLE,
                                       &value, 0) == 0 && value == 63);
  CHECK(ssf_mems_protocol_write_setting(&serial, SSF_MEMS_SETTING_FEATURE_ENABLE,
                                        0, 0) == 0);
  CHECK(setting_registers[69] == 0x03a5);
  NO_SEND(ssf_mems_protocol_write_setting(&serial, SSF_MEMS_SETTING_SAMPLING_RATE, 10, 0), -EINVAL);
  NO_SEND(ssf_mems_protocol_write_setting(&serial, SSF_MEMS_SETTING_FEATURE_ENABLE, 64, 0), -EINVAL);
  NO_SEND(ssf_mems_protocol_write_setting(&serial, SSF_MEMS_SETTING_PARAMETER_SWITCH, 0, 0), -EINVAL);
  NO_SEND(ssf_mems_protocol_read_setting(&serial, SSF_MEMS_SETTING_MAX, &value, 0), -EINVAL);
  CHECK(!reboots && !sleep_count);
  ignore_setting_writes = true;
  CHECK(ssf_mems_protocol_write_setting(&serial, SSF_MEMS_SETTING_SAMPLING_RATE,
                                        6, 0) == -EIO);
  response_mode = REPLY_EXCEPTION;
  value = 999;
  CHECK(ssf_mems_protocol_read_setting(&serial, SSF_MEMS_SETTING_SAMPLING_RATE,
                                       &value, 0) == -EREMOTEIO && value == 999);
}

/* 构造固件私有流测试包，包含负数和数据区内的帧头/尾标记。 */
static void make_raw_packet(u8 *packet, u32 sequence) {
  unsigned int i;
  memset(packet, 0, SSF_MEMS_RAW_PACKET_BYTES);
  packet[0] = 0x15;
  packet[1] = sequence >> 24;
  packet[2] = sequence >> 16;
  packet[3] = sequence >> 8;
  packet[4] = sequence;
  for (i = 0; i < 64; i++) {
    packet[5 + i * 6] = 0xff;
    packet[6 + i * 6] = 0xfe; /* X = -2 */
    packet[7 + i * 6] = 0x80; /* Y = -32768 */
    packet[9 + i * 6] = 0x15;
    packet[10 + i * 6] = 0x17; /* Z = 5399，不应误判成边界。 */
  }
  finish_crc(packet, 391);
  packet[391] = 0x17;
}

/* 使用实际 FIFO/工作项分批交付流，不直接调用解码器。 */
static void feed_raw_packet(u32 sequence) {
  u8 packet[SSF_MEMS_RAW_PACKET_BYTES];
  size_t offset;
  make_raw_packet(packet, sequence);
  for (offset = 0; offset < sizeof(packet); offset++) {
    CHECK(ssf_mems_rx_push(&serial, packet + offset, 1) == 1);
    sensor.modbus_rx.work.fn(&sensor.modbus_rx.work);
  }
}

/* 验证拆包、粘包、CRC、尾标记、丢字节恢复和序号回绕。 */
static void test_raw_parser(void) {
  u8 packet[SSF_MEMS_RAW_PACKET_BYTES];
  size_t i;

  reset_sensor();
  sensor.raw.active = true;
  sensor.raw.publishing = true;
  feed_raw_packet(0xffffffffU);
  feed_raw_packet(0);
  CHECK(raw_published == 128 && sensor.raw.discontinuities == 0);
  CHECK(raw_last.xyz[0] == -2 && raw_last.xyz[1] == -32768 && raw_last.xyz[2] == 5399);
  CHECK(raw_last.packet_sequence == 0 && raw_last.sample_index == 63);
  make_raw_packet(packet, 1);
  packet[389] ^= 1;
  CHECK(ssf_mems_rx_push(&serial, packet, sizeof(packet)) == sizeof(packet));
  sensor.modbus_rx.work.fn(&sensor.modbus_rx.work);
  CHECK(raw_published == 128 && sensor.raw.crc_errors > 0);
  feed_raw_packet(2);
  CHECK(raw_published == 192 && sensor.raw.discontinuities == 1);
  make_raw_packet(packet, 3);
  packet[391] = 0;
  for (i = 0; i < sizeof(packet); i++)
    ssf_mems_raw_receive_byte(&sensor, packet[i]);
  feed_raw_packet(4);
  CHECK(raw_published == 256 && sensor.raw.discontinuities == 2);
  raw_publish_error = -ENOSPC;
  feed_raw_packet(5);
  CHECK(sensor.raw.buffer_errors == 64 && sensor.raw.packets == 5);
  sensor.raw.publishing = false;
  feed_raw_packet(7);
  CHECK(sensor.raw.packets == 5 && sensor.raw.discontinuities == 2);
  CHECK(sensor.raw.drain_packets == 1 && sensor.raw.drain_discontinuities == 1);
  make_raw_packet(packet, 8);
  packet[389] ^= 1;
  CHECK(ssf_mems_rx_push(&serial, packet, sizeof(packet)) == sizeof(packet));
  sensor.modbus_rx.work.fn(&sensor.modbus_rx.work);
  CHECK(sensor.raw.drain_crc_errors > 0 && raw_published == 320);
  sensor.raw.active = false;
}

/* 首次等待交付一个真实数据包；第二次等待模拟正常采集时段结束。 */
static void raw_wait_hook(unsigned long timeout) {
  if (sensor.raw.active == false) {
    test_wait_result = 0; /* 普通请求发送前的帧间空闲等待。 */
    return;
  }
  CHECK(sensor.raw.active && serial.baudrate == 1000000);
  CHECK(!sensor.modbus_req.pending && !sensor.modbus_req.busy);
  if (sensor.raw.publishing == false) {
    raw_gap_waits++;
    CHECK(timeout <= SSF_MEMS_RAW_STOP_GAP_WAIT_MS);
    if (raw_sensor_running == true) {
      feed_raw_packet(drain_sequence++);
      if (raw_gap_partial == true && raw_gap_waits == 1)
        ssf_mems_raw_receive_byte(&sensor, 0x15);
      test_wait_result = 1;
    } else {
      test_wait_result = 0;
    }
    return;
  }
  if (sensor.raw.packets == 0) {
    CHECK(timeout == SSF_MEMS_RAW_START_TIMEOUT_MS);
    if (raw_wait_action == 4)
      jiffies += 6000; /* 实测可能超过旧的 5 秒上限。 */
    if (raw_wait_action == 5) {
      sensor.acquisition.stopping = true;
      ssf_mems_modbus_request_remove(&sensor);
      late_raw_packet_at = jiffies + 2000;
      test_wait_result = 1;
      return;
    }
    feed_raw_packet(0);
    test_wait_result = 1;
  } else {
    if (raw_wait_action == 1) {
      CHECK(timeout == 1000); /* 连续一秒无包，应提前退出。 */
    } else if (raw_wait_action == 2) {
      sensor.acquisition.stopping = true;
      sensor.modbus_req.shutting_down = true;
    } else if (raw_wait_action == 3) {
      test_wait_result = -ERESTARTSYS;
      return;
    } else {
      CHECK(timeout == 100);
      feed_raw_packet(1);
    }
    test_wait_result = 0;
  }
}

/* 保持未停止的私有流，或在首包前卸载后交付迟到包。 */
static void raw_recovery_sleep_hook(void) {
  if (sensor.raw.active == false || serial.baudrate != 1000000)
    return;
  if (late_raw_packet_at != 0) {
    if (time_before(jiffies, late_raw_packet_at) == true)
      return;
    late_raw_packet_at = 0;
    feed_raw_packet(drain_sequence++);
  } else if (raw_sensor_running == true && sensor.raw.publishing == false) {
    feed_raw_packet(drain_sequence++);
  }
}

/* 回归覆盖迟到启动、停止丢失、恢复门禁、卸载确认及临时 1 Mbaud 识别。 */
static void test_raw_recovery(void) {
  enum ssf_mems_baudrate found = SSF_MEMS_BAUDRATE_MAX;
  unsigned int before, i;
  u16 value;

  reset_sensor();
  if (ssf_mems_modbus_find_reg(40058) == NULL ||
      ssf_mems_modbus_find_reg(40059) == NULL)
    return;
  emulate_settings = enforce_sensor_baudrate = emulate_raw_mode = true;
  setting_registers[51] = 6;
  wait_hook = raw_wait_hook;
  wait_returns_early = true;
  raw_wait_action = 4;
  raw_gap_partial = true;
  CHECK(ssf_mems_protocol_collect_raw(&sensor, 100) == 0);
  CHECK(sensor.raw.start_wait_ms == 6000 && sensor.raw.packets == 2);
  CHECK(raw_gap_waits == 2 && raw_published == 128);
  CHECK(sensor.protocol.mode == SSF_MEMS_LINK_MODBUS);

  reset_sensor();
  emulate_settings = enforce_sensor_baudrate = emulate_raw_mode = true;
  setting_registers[51] = 6;
  wait_hook = raw_wait_hook;
  sleep_hook = raw_recovery_sleep_hook;
  wait_returns_early = true;
  raw_wait_action = 5;
  CHECK(ssf_mems_protocol_collect_raw(&sensor, 100) == -ENODEV);
  CHECK(sensor.raw.packets == 0 && sensor.raw.drain_packets >= 1);
  CHECK(raw_published == 0 && sensor.protocol.mode == SSF_MEMS_LINK_MODBUS);
  CHECK(sends == 4 && tx_events[2].sent_at >= 2000);
  CHECK(tx_events[3].address == 101 && sensor.modbus_req.shutting_down);
  NO_SEND(ssf_mems_modbus_read_reg(&serial, 40102, &value, 50), -ENODEV);

  reset_sensor();
  emulate_settings = enforce_sensor_baudrate = emulate_raw_mode = true;
  setting_registers[51] = 6;
  wait_hook = raw_wait_hook;
  sleep_hook = raw_recovery_sleep_hook;
  ignored_raw_stops = SSF_MEMS_RAW_STOP_ATTEMPTS;
  CHECK(ssf_mems_protocol_collect_raw(&sensor, 100) == -ETIMEDOUT);
  CHECK(sensor.protocol.mode == SSF_MEMS_LINK_RAW_RECOVERING);
  CHECK(sensor.protocol.baudrate == SSF_MEMS_BAUDRATE_DEFAULT);
  CHECK(sensor.raw.active && !sensor.raw.publishing && sensor.raw.drain_packets > 0);
  CHECK(sensor.raw.packets == 2 && sensor.raw.discontinuities == 0);
  CHECK(sensor.raw.stop_attempts == SSF_MEMS_RAW_STOP_ATTEMPTS && !reboots);
  NO_SEND(ssf_mems_modbus_read_reg(&serial, 40102, &value, 50), -EAGAIN);
  NO_SEND(ssf_mems_modbus_write_reg(&serial, 40102, 6, 50), -EAGAIN);
  if (ssf_mems_modbus_find_reg(40110) != NULL)
    NO_SEND(ssf_mems_protocol_set_baudrate(&serial, SSF_MEMS_BAUDRATE_57600, 50), -EAGAIN);
  before = sends;
  CHECK(ssf_mems_protocol_get_baudrate(&serial, &found, 50) == 0);
  CHECK(ssf_mems_baudrate_to_value(found) == 9600 && serial.baudrate == 9600);
  CHECK(sensor.protocol.mode == SSF_MEMS_LINK_MODBUS && !reboots);
  CHECK(tx_events[before].address == 58); /* 先停止，不扫描或写 40102/40110。 */
  for (i = before; i < sends; i++)
    CHECK(tx_events[i].function == 3 || tx_events[i].address == 58);

  before = sends;
  CHECK(ssf_mems_protocol_recover_raw(&sensor) == 0 && sends == before);
  CHECK(ssf_mems_protocol_recover_raw(NULL) == -EINVAL);
  /* 模拟线程已离开收流函数、在恢复退避中卸载。 */
  sensor.protocol.mode = SSF_MEMS_LINK_RAW_RECOVERING;
  sensor_baudrate = SSF_MEMS_BAUDRATE_1000000;
  raw_sensor_running = true;
  ssf_mems_modbus_request_remove(&sensor);
  NO_SEND(ssf_mems_modbus_read_reg(&serial, 40102, &value, 50), -ENODEV);
  CHECK(ssf_mems_protocol_recover_raw(&sensor) == 0);
  CHECK(sensor.protocol.mode == SSF_MEMS_LINK_MODBUS && !reboots);
  CHECK(sensor.modbus_req.shutting_down && serial.baudrate == 9600);
  CHECK(tx_events[before].address == 58 && tx_events[before + 1].address == 101);

  reset_sensor();
  enforce_sensor_baudrate = emulate_raw_mode = true;
  sensor_baudrate = sensor.protocol.baudrate = SSF_MEMS_BAUDRATE_1000000;
  staged_baudrate = SSF_MEMS_BAUDRATE_9600;
  sensor.protocol.host_baudrate = serial.baudrate = 1000000;
  CHECK(ssf_mems_protocol_get_baudrate(&serial, &found, 50) == 0);
  CHECK(found == SSF_MEMS_BAUDRATE_9600 && serial.baudrate == 9600);
  CHECK(sensor.protocol.mode == SSF_MEMS_LINK_MODBUS && !reboots);
  CHECK(sends == 3 && tx_events[1].address == 58);

  reset_sensor();
  enforce_sensor_baudrate = emulate_raw_mode = true;
  sensor_baudrate = SSF_MEMS_BAUDRATE_1000000;
  staged_baudrate = raw_restore_baudrate = SSF_MEMS_BAUDRATE_57600;
  raw_sensor_running = true;
  sleep_hook = raw_recovery_sleep_hook;
  /* 启动目标为 9600，但遗留流退出后实际是 57600；仅只读发现，不保存。 */
  CHECK(ssf_mems_protocol_get_baudrate(&serial, &found, 50) == 0);
  CHECK(found == SSF_MEMS_BAUDRATE_57600 && serial.baudrate == 57600);
  CHECK(sensor.raw.drain_packets > 0 && sensor.raw.packets == 0);
  CHECK(sensor.protocol.mode == SSF_MEMS_LINK_MODBUS && !reboots);
  CHECK(!sensor.protocol.raw_baudrate_unknown);
  for (i = 0; i < sends; i++)
    CHECK(tx_events[i].function == 3 || tx_events[i].address == 58);
}

/* 验证启动无 ACK、临时切速、停止和恢复验证，以及失败/卸载清理。 */
static void test_raw_session(void) {
  reset_sensor();
  sensor.protocol.parity = SSF_MEMS_PARITY_EVEN;
  NO_SEND(ssf_mems_protocol_collect_raw(&sensor, 100), -EOPNOTSUPP);
  CHECK(ssf_mems_protocol_collect_raw(NULL, 100) == -EINVAL);
  reset_sensor();
  if (ssf_mems_modbus_find_reg(40058) == NULL ||
      ssf_mems_modbus_find_reg(40059) == NULL) {
    NO_SEND(ssf_mems_protocol_collect_raw(&sensor, 100), -ENOENT);
    return;
  }
  emulate_settings = true;
  setting_registers[51] = 9;
  CHECK(ssf_mems_protocol_collect_raw(&sensor, 100) == -EOPNOTSUPP);
  CHECK(sends == 1 && !sensor.raw.active);
  reset_sensor();
  emulate_settings = true;
  setting_registers[51] = 6;
  enforce_sensor_baudrate = emulate_raw_mode = true;
  wait_hook = raw_wait_hook;
  CHECK(ssf_mems_protocol_collect_raw(&sensor, 100) == 0);
  CHECK(raw_published == 128 && sensor.raw.packets == 2 && !sensor.raw.active);
  CHECK(serial.baudrate == 9600 && sensor.protocol.host_baudrate == 9600);
  CHECK(sends == 4 && tx_events[0].address == 51 && tx_events[1].address == 57);
  CHECK(tx_events[2].address == 58 && tx_events[2].baudrate == 1000000);
  CHECK(tx_events[3].address == 101 && tx_events[3].baudrate == 9600);
  CHECK(!sensor.protocol.valid);
  reset_sensor();
  emulate_settings = true;
  setting_registers[51] = 6;
  enforce_sensor_baudrate = emulate_raw_mode = true;
  /* 不交付首包，应超时退出，但仍停止并恢复。 */
  CHECK(ssf_mems_protocol_collect_raw(&sensor, 100) == -ETIMEDOUT);
  CHECK(serial.baudrate == 9600 && !sensor.raw.active && sends == 4);
  reset_sensor();
  emulate_settings = true;
  setting_registers[51] = 6;
  enforce_sensor_baudrate = emulate_raw_mode = true;
  wait_hook = raw_wait_hook;
  raw_wait_action = 1;
  CHECK(ssf_mems_protocol_collect_raw(&sensor, 5000) == -ETIMEDOUT);
  CHECK(serial.baudrate == 9600 && !sensor.raw.active && sends == 4);
  reset_sensor();
  emulate_settings = true;
  setting_registers[51] = 6;
  enforce_sensor_baudrate = emulate_raw_mode = true;
  wait_hook = raw_wait_hook;
  raw_wait_action = 2;
  CHECK(ssf_mems_protocol_collect_raw(&sensor, 100) == -ENODEV);
  CHECK(serial.baudrate == 9600 && !sensor.raw.active && sends == 4);
  CHECK(tx_events[2].address == 58); /* 请求模块关闭后仍执行流停止。 */
  CHECK(tx_events[3].address == 101); /* 卸载时也确认普通通信，而非仅发送成功。 */
  reset_sensor();
  emulate_settings = true;
  setting_registers[51] = 6;
  enforce_sensor_baudrate = emulate_raw_mode = true;
  wait_hook = raw_wait_hook;
  raw_wait_action = 3;
  CHECK(ssf_mems_protocol_collect_raw(&sensor, 100) == -ERESTARTSYS);
  CHECK(serial.baudrate == 9600 && !sensor.raw.active && sends == 4);
  reset_sensor();
  emulate_settings = true;
  setting_registers[51] = 6;
  enforce_sensor_baudrate = emulate_raw_mode = true;
  wait_hook = raw_wait_hook;
  fail_on_send = 3; /* 第一次停止发送失败，必须重试停止而非误发普通读。 */
  CHECK(ssf_mems_protocol_collect_raw(&sensor, 100) == 0);
  CHECK(sends == 5 && tx_events[2].address == 58 && tx_events[3].address == 58);
  CHECK(serial.baudrate == 9600 && !sensor.raw.active);
}

/* 联合验证真实属性解析、首帧速率、枚举扫描及保存重启，不只检查目标变量。 */
static void test_startup_baudrate_matching(void) {
  static const struct {
    bool has_speed;
    u32 speed;
    enum ssf_mems_baudrate sensor_rate;
    unsigned int expected_target;
  } cases[] = {
      {false, 0, SSF_MEMS_BAUDRATE_9600, 9600},
      {true, 0, SSF_MEMS_BAUDRATE_9600, 9600},
      {true, 12345, SSF_MEMS_BAUDRATE_57600, 9600},
      {true, 57600, SSF_MEMS_BAUDRATE_9600, 57600},
      {true, 115200, SSF_MEMS_BAUDRATE_115200, 115200},
      {true, 115200, SSF_MEMS_BAUDRATE_57600, 115200},
  };
  unsigned int i;

  for (i = 0; i < ARRAY_SIZE(cases); i++) {
    enum ssf_mems_baudrate found, target;
    unsigned int before;

    reset_sensor();
    serial.dev.has_current_speed = cases[i].has_speed;
    serial.dev.current_speed = cases[i].speed;
    CHECK(ssf_mems_protocol_init(&sensor) == 0);
    CHECK(ssf_mems_protocol_configure_serial(&sensor) == 0);
    target = sensor.protocol.baudrate;
    CHECK((unsigned int)ssf_mems_baudrate_to_value(target) == cases[i].expected_target);
    CHECK(sends == 0); /* 启动属性和主机 UART 配置不会发送 Modbus。 */
    sensor_baudrate = cases[i].sensor_rate;
    enforce_sensor_baudrate = true;
    CHECK(ssf_mems_protocol_get_baudrate(&serial, &found, 200) == 0);
    CHECK(tx_events[0].baudrate == cases[i].expected_target);
    CHECK(ssf_mems_baudrate_to_value(found) == ssf_mems_baudrate_to_value(cases[i].sensor_rate));
    before = sends;
    if ((unsigned int)ssf_mems_baudrate_to_value(found) != cases[i].expected_target) {
      if (ssf_mems_modbus_find_reg(40110) == NULL) {
        NO_SEND(ssf_mems_protocol_set_baudrate(&serial, target, 200), -ENOENT);
        continue;
      }
      CHECK(ssf_mems_protocol_set_baudrate(&serial, target, 200) == 0);
      CHECK(sends == before + 3 && reboots == 1);
    } else {
      CHECK(reboots == 0 && sends == 1);
    }
    CHECK(serial.baudrate == cases[i].expected_target);
    CHECK((unsigned int)ssf_mems_baudrate_to_value(sensor_baudrate) == cases[i].expected_target);
  }
}

int main(int argc, char **argv) {
  CHECK(argc == 2);
  test_initialization_and_cleanup();
  test_receive_callback();
  test_unclaimed_frames();
  test_codecs();
  test_response_matching();
  test_requests();
  test_serial_configuration();
  test_baudrate_management();
  test_baudrate_reboot();
  test_request_isolation();
  test_receive_expiry();
  test_sparse_features();
  test_feature_settings();
  test_raw_parser();
  test_raw_session();
  test_raw_recovery();
  test_startup_baudrate_matching();
  cleanup_sensor();
  printf("PASS %-23s %u checks\n", argv[1], checks);
  return 0;
}
