#include "core.h"
#include "modbus.h"
#include "modbus_table.h"
#include "modbus_request.h"
#include "modbus_receive.h"
#include "protocol.h"

bool test_allocation_failure;
long test_wait_result;
unsigned int test_queue_count, test_cancel_count, test_wake_count;

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
static enum ssf_mems_baudrate sensor_baudrate;

enum { REPLY_NORMAL, REPLY_EXCEPTION, REPLY_BAD_CRC, REPLY_WRONG_COUNT, REPLY_NONE, REPLY_MISMATCH_THEN_NORMAL };
#define CHECK(expr) do { checks++; if (!(expr)) { fprintf(stderr, "FAIL line %d: %s\n", __LINE__, #expr); abort(); } } while (0)
#define NO_SEND(expr, error) do { unsigned int before = sends; CHECK((expr) == (error)); CHECK(sends == before); } while (0)

static u16 wire_value(u16 address) {
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

  CHECK(size <= sizeof(last_tx));
  s->write_timeout = timeout;
  memcpy(last_tx, buf, size);
  last_tx_len = size;
  sends++;
  CHECK(ssf_mems_modbus_check_crc(buf, size) == 0);
  address = ((u16)buf[2] << 8) | buf[3];
  count = ((u16)buf[4] << 8) | buf[5];
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
  if (enforce_sensor_baudrate &&
      s->baudrate != (unsigned int)ssf_mems_baudrate_to_value(sensor_baudrate))
    return size;
  if (response_mode == REPLY_NONE)
    return size;

  if (response_mode == REPLY_MISMATCH_THEN_NORMAL) {
    u8 unmatched[] = {1, 6, 0, 100, 0x12, 0x34, 0, 0};
    CHECK(data->modbus_req.pending && data->modbus_req.busy);
    finish_crc(unmatched, sizeof(unmatched));
    check_discard_preserves_state(data, unmatched, sizeof(unmatched));
  }

  response[0] = buf[0];
  response[1] = buf[1];
  if (response_mode == REPLY_EXCEPTION) {
    response[1] |= 0x80;
    response[2] = 2;
    len = 5;
  } else if (buf[1] == 3) {
    if (response_mode == REPLY_WRONG_COUNT)
      count--;
    response[2] = count * 2;
    len = 5 + count * 2;
    for (i = 0; i < count; i++) {
      u16 value;

      if (enforce_sensor_baudrate && address + i == 101)
        value = invalid_baudrate_value ? SSF_MEMS_BAUDRATE_MAX
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
  if (response_mode == REPLY_BAD_CRC)
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
  if (enforce_sensor_baudrate && response_mode != REPLY_EXCEPTION &&
      buf[1] == SSF_MEMS_MODBUS_FUNC_WRITE_SINGLE && address == 101 &&
      count < SSF_MEMS_BAUDRATE_MAX)
    sensor_baudrate = count;
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
  sensor_baudrate = SSF_MEMS_BAUDRATE_DEFAULT;
  test_wait_result = 0;
  test_queue_count = test_cancel_count = test_wake_count = 0;
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

  NO_SEND(ssf_mems_modbus_read(&serial, 40059, 1, values, 126, 0), -EACCES);
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
  CHECK(ssf_mems_protocol_init(&sensor) == -EINVAL);

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
  CHECK(sends == sends_before + 1);
  CHECK(sensor_baudrate == SSF_MEMS_BAUDRATE_115200);
  CHECK(sensor.protocol.baudrate == SSF_MEMS_BAUDRATE_115200);
  CHECK(serial.baudrate == 115200);
  CHECK(last_tx[1] == SSF_MEMS_MODBUS_FUNC_WRITE_SINGLE && last_tx[3] == 101);
  CHECK(last_tx[4] == 0 && last_tx[5] == SSF_MEMS_BAUDRATE_115200);

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
  test_sparse_features();
  cleanup_sensor();
  printf("PASS %-23s %u checks\n", argv[1], checks);
  return 0;
}
