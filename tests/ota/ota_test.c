/* Production code is linked separately; the model implements the sensor's
 * nextPack/Flash semantics rather than returning unconditional success. */
#include "core.h"
#include "modbus.h"
#include <linux/firmware.h>
#include <linux/jiffies.h>
#include <assert.h>
#include <errno.h>

bool test_allocation_failure;
long test_wait_result;
unsigned int test_queue_count, test_cancel_count, test_wake_count;
unsigned int test_delayed_queue_count, test_delayed_cancel_count;
unsigned long jiffies;

static struct ssf_mems_xyzs_data sensor;
static struct serdev_device serial;
static struct firmware firmware;
static u8 image[SSF_MEMS_OTA_MAX_SIZE + 1];
static u8 flash[SSF_MEMS_OTA_MAX_SIZE];
static u8 response[32];
static size_t response_len;
static _Atomic unsigned int checks; /* Assertions may run on both lifecycle-test threads. */
static unsigned int sends, queries, finals, writes, releases;
static unsigned int first_attempts, version_reads, sleeps;
static u32 next_packet;
static unsigned long reboot_until;
static bool initialized, stop_on_load, signal_wait;
static int load_error;
static bool acquisition_held_by_test; /* Main thread emulates an in-flight raw window. */
static pthread_mutex_t loader_lock = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t loader_event = PTHREAD_COND_INITIALIZER;
static bool loader_entered; /* Protected by loader_lock. */
static int worker_result; /* Read only after joining the updating thread. */
enum fault {
  NORMAL, LOST_FIRST_ACK, LOST_FINAL_ACK, LOST_QUERY_ACK, BAD_QUERY_COUNTER,
  WRONG_PACKET_COUNTER, REJECT_PACKET, SHORT_FIRST, SHORT_FINAL, NO_BOOT_REPLY,
  STOP_FIRST, STOP_FINAL, SIGNAL_FIRST, SIGNAL_FINAL, OLD_ACK_FIRST, WRONG_ADDRESS,
};
static enum fault fault;
#define CHECK(expr) do { checks++; if (!(expr)) { \
  fprintf(stderr, "FAIL line %d: %s\n", __LINE__, #expr); abort(); \
} } while (0)

/* Exercise the actual byte FIFO/worker, including one-byte serdev fragments. */
long test_wait_timeout(unsigned long timeout) {
  size_t i;

  if (signal_wait == true) {
    signal_wait = false;
    return -ERESTARTSYS;
  }
  if (response_len != 0) {
    size_t length = response_len;

    response_len = 0;
    for (i = 0; i < length; i++) {
      CHECK(ssf_mems_rx_push(&serial, response + i, 1) == 1);
      sensor.modbus_rx.work.fn(&sensor.modbus_rx.work);
    }
    return 1;
  }
  jiffies += timeout;
  return 0;
}

void test_msleep(unsigned int ms) {
  sleeps += ms;
  jiffies += ms;
}

int ssf_mems_iio_publish_raw(struct ssf_mems_xyzs_data *data,
                            const struct ssf_mems_raw_sample *sample) {
  (void)data;
  (void)sample;
  CHECK(false); /* OTA bytes must never become IIO raw samples. */
  return -EINVAL;
}

int request_firmware_direct(const struct firmware **fw, const char *name,
                            struct device *dev) {
  CHECK(dev == &serial.dev && strcmp(name, "update.bin") == 0);
  if (acquisition_held_by_test == true) {
    CHECK(pthread_mutex_trylock(&sensor.acquisition.lock.native) == EBUSY);
    pthread_mutex_lock(&loader_lock);
    loader_entered = true;
    pthread_cond_signal(&loader_event);
    pthread_mutex_unlock(&loader_lock);
  } else {
    CHECK(mutex_trylock(&sensor.acquisition.lock) != 0);
    mutex_unlock(&sensor.acquisition.lock);
  }
  if (stop_on_load == true)
    ssf_mems_ota_shutdown(&sensor);
  if (load_error != 0)
    return load_error;
  *fw = &firmware;
  return 0;
}

void release_firmware(const struct firmware *fw) {
  CHECK(fw == &firmware);
  releases++;
}

/* This models Modbus.c's unusual 0x41 branch: query packets do not write,
 * repeated normal sequence numbers do not write, FINAL always writes at nextPack. */
ssize_t serdev_device_write(struct serdev_device *s, const u8 *buf,
                           size_t len, unsigned long timeout) {
  u32 sequence;
  u32 ack;
  unsigned int sum = 0;
  size_t i;

  CHECK(s == &serial);
  CHECK(pthread_mutex_trylock(&sensor.protocol.bus_lock.native) == EBUSY);
  CHECK(pthread_mutex_trylock(&sensor.acquisition.lock.native) == EBUSY);
  CHECK(pthread_mutex_trylock(&sensor.ota.update_lock.native) == EBUSY);
  CHECK(response_len == 0);
  sends++;
  if (buf[1] == 3) {
    u16 crc;

    CHECK(len == 8 && buf[2] == 0 && buf[3] == 120 && buf[5] == 1);
    CHECK(ssf_mems_modbus_check_crc(buf, len) == 0);
    CHECK(sensor.protocol.mode == SSF_MEMS_LINK_MODBUS);
    version_reads++;
    if (finals != 0) {
      CHECK(jiffies >= reboot_until && sleeps >= 15000);
      if (fault == NO_BOOT_REPLY)
        return len;
    }
    response[0] = sensor.slave_id;
    response[1] = 3;
    response[2] = 2;
    response[3] = 0x13;
    response[4] = finals != 0 ? 0xed : 0xec; /* 5100 -> 5101 */
    crc = ssf_mems_modbus_crc16(response, 5);
    response[5] = crc;
    response[6] = crc >> 8;
    response_len = 7;
    return len;
  }
  CHECK(buf[1] == 0x41 && len == 263);
  CHECK(sensor.protocol.mode == SSF_MEMS_LINK_OTA);
  CHECK(timeout >= DIV_ROUND_UP(263U * 10000U, serial.baudrate));
  for (i = 0; i < len - 1; i++)
    sum += buf[i];
  CHECK((sum & 0xff) == buf[len - 1]);
  sequence = (u32)buf[2] << 24 | (u32)buf[3] << 16 |
              (u32)buf[4] << 8 | buf[5];
  if (sequence == 0xfffffffeU) {
    struct ssf_mems_ota_status status;

    queries++;
    ssf_mems_ota_get_status(&sensor, &status);
    CHECK(status.stage == SSF_MEMS_OTA_CHECKING);
    CHECK(ssf_mems_acquisition_update_firmware(&sensor, "update.bin") == -EBUSY);
    CHECK(writes == 0 && next_packet == 0);
    if (fault == LOST_QUERY_ACK)
      return len;
  } else if (sequence == 0xffffffffU) {
    finals++;
    CHECK(finals == 1); /* A retry here would irreversibly corrupt the staged image. */
    reboot_until = jiffies + 15000;
    if (fault == SHORT_FINAL)
      return len - 1;
  } else {
    if (sequence == 0)
      first_attempts++;
    if (fault == SHORT_FIRST)
      return len - 1;
  }
  if ((sequence == next_packet || sequence == 0xffffffffU) &&
      fault != REJECT_PACKET) {
    CHECK(next_packet < 1024);
    memcpy(flash + 256 * next_packet, buf + 6, 256);
    writes++;
    next_packet++;
  }
  if ((fault == STOP_FIRST && sequence == 0) ||
      (fault == STOP_FINAL && sequence == 0xffffffffU)) {
    ssf_mems_ota_shutdown(&sensor);
    return len;
  }
  if ((fault == SIGNAL_FIRST && sequence == 0) ||
      (fault == SIGNAL_FINAL && sequence == 0xffffffffU)) {
    signal_wait = true;
    return len;
  }
  if ((fault == LOST_FIRST_ACK && sequence == 0 && first_attempts == 1) ||
      (fault == LOST_FINAL_ACK && sequence == 0xffffffffU))
    return len;
  ack = next_packet;
  if (fault == BAD_QUERY_COUNTER)
    ack = 1; /* A previous abandoned session, including the dangerous one-packet case. */
  if (fault == WRONG_PACKET_COUNTER && sequence == 0)
    ack = 99;
  if (fault == OLD_ACK_FIRST && sequence == 0 && first_attempts == 1)
    ack = 0;
  /* Deliberate leading junk and repeated address exercise stream resynchronization. */
  response[0] = 0x88;
  response[1] = sensor.slave_id;
  response[2] = fault == WRONG_ADDRESS ? sensor.slave_id + 1U : sensor.slave_id;
  response[3] = 0x41;
  response[4] = ack >> 24;
  response[5] = ack >> 16;
  response[6] = ack >> 8;
  response[7] = ack;
  response_len = 8;
  return len;
}

/* Release every native mutex between fixtures, in addition to the production FIFO. */
static void cleanup(void) {
  if (initialized == false)
    return;
  ssf_mems_modbus_receive_remove(&sensor);
  pthread_mutex_destroy(&sensor.modbus_rx.parse_lock.native);
  pthread_mutex_destroy(&sensor.modbus_req.lock.native);
  pthread_mutex_destroy(&sensor.protocol.bus_lock.native);
  pthread_mutex_destroy(&sensor.protocol.lock.native);
  pthread_mutex_destroy(&sensor.acquisition.lock.native);
  pthread_mutex_destroy(&sensor.acquisition.status_lock.native);
  pthread_mutex_destroy(&sensor.ota.lock.native);
  pthread_mutex_destroy(&sensor.ota.update_lock.native);
  initialized = false;
}

static void reset(size_t size, enum fault selected) {
  size_t i;

  cleanup();
  memset(&sensor, 0, sizeof(sensor));
  memset(&serial, 0, sizeof(serial));
  sensor.serdev = &serial;
  serial.drvdata = &sensor;
  serial.dev.has_current_speed = true;
  serial.dev.current_speed = 128000;
  CHECK(ssf_mems_protocol_init(&sensor) == 0);
  CHECK(ssf_mems_modbus_request_init(&sensor) == 0);
  CHECK(ssf_mems_modbus_receive_init(&sensor, ssf_mems_protocol_handle_frame, &sensor) == 0);
  mutex_init(&sensor.acquisition.lock);
  mutex_init(&sensor.acquisition.status_lock);
  sensor.acquisition.online = true;
  sensor.acquisition.mode = SSF_MEMS_ACQ_MODE_ALTERNATING;
  sensor.acquisition.raw_duration_ms = 5000;
  sensor.acquisition.feature_duration_ms = 5000;
  serial.baudrate = 128000;
  for (i = 0; i < sizeof(image); i++)
    image[i] = (i * 59U + i / 17U) & 0xff;
  image[0] = 0x00; image[1] = 0x00; image[2] = 0x02; image[3] = 0x20;
  image[4] = 0x41; image[5] = 0x00; image[6] = 0x02; image[7] = 0x08;
  firmware.data = image;
  firmware.size = size;
  memset(flash, 0, sizeof(flash));
  fault = selected;
  sends = queries = finals = writes = releases = next_packet = 0;
  first_attempts = version_reads = sleeps = 0;
  response_len = 0;
  reboot_until = 0;
  jiffies = 100;
  load_error = 0;
  stop_on_load = signal_wait = false;
  initialized = true;
}

static void check_image(void) {
  size_t padded = DIV_ROUND_UP(firmware.size, 256U) * 256U;
  size_t i;

  CHECK(memcmp(flash, image, firmware.size) == 0);
  CHECK(writes == DIV_ROUND_UP(firmware.size, 256U));
  CHECK(next_packet == writes);
  for (i = firmware.size; i < padded; i++)
    CHECK(flash[i] == 0xff);
}

static void test_success(void) {
  const size_t sizes[] = {80, 255, 256, 257, 511, 512, 513, 262144};
  struct ssf_mems_ota_status status;
  size_t i;

  for (i = 0; i < ARRAY_SIZE(sizes); i++) {
    reset(sizes[i], NORMAL);
    if (i == 0) {
      sensor.protocol.host_baudrate = serial.baudrate = 2400;
      sensor.protocol.parity = SSF_MEMS_PARITY_EVEN;
    }
    CHECK(ssf_mems_acquisition_update_firmware(&sensor, "update.bin") == 0);
    check_image();
    ssf_mems_ota_get_status(&sensor, &status);
    CHECK(status.stage == SSF_MEMS_OTA_COMPLETE && status.error == 0);
    CHECK(status.total == sizes[i] && status.acknowledged == sizes[i]);
    CHECK(status.version == 5101 && finals == 1 && queries == 1);
    CHECK(version_reads == 2 && releases == 1);
    CHECK(serial.baudrate_set_count == 0 && serial.parity_set_count == 0);
    CHECK(sensor.acquisition.reconnect == true && sensor.acquisition.firmware_blocked == false);
    CHECK(sensor.protocol.mode == SSF_MEMS_LINK_MODBUS && sensor.ota.pending == false);
    CHECK(sensor.acquisition.mode == SSF_MEMS_ACQ_MODE_ALTERNATING);
    CHECK(sensor.acquisition.raw_duration_ms == 5000 && sensor.acquisition.feature_duration_ms == 5000);
  }
}

static void test_failures(void) {
  const struct {
    enum fault fault;
    int error;
    bool committed;
  } cases[] = {
    {LOST_FIRST_ACK, 0, true}, {OLD_ACK_FIRST, 0, true},
    {LOST_FINAL_ACK, -ETIMEDOUT, true}, {LOST_QUERY_ACK, -ETIMEDOUT, false},
    {BAD_QUERY_COUNTER, -EPROTO, false}, {WRONG_PACKET_COUNTER, -EPROTO, false},
    {REJECT_PACKET, -ETIMEDOUT, false}, {SHORT_FIRST, -EIO, false},
    {SHORT_FINAL, -EIO, true}, {NO_BOOT_REPLY, -ETIMEDOUT, true},
    {STOP_FIRST, -ENODEV, false}, {STOP_FINAL, -ENODEV, true},
    {SIGNAL_FIRST, -EINTR, false}, {SIGNAL_FINAL, -EINTR, true},
    {WRONG_ADDRESS, -ETIMEDOUT, false},
  };
  struct ssf_mems_ota_status status;
  size_t i;

  for (i = 0; i < ARRAY_SIZE(cases); i++) {
    reset(513, cases[i].fault);
    CHECK(ssf_mems_acquisition_update_firmware(&sensor, "update.bin") == cases[i].error);
    ssf_mems_ota_get_status(&sensor, &status);
    CHECK(finals == (cases[i].committed ? 1U : 0U));
    CHECK(status.error == cases[i].error && releases == 1);
    CHECK(sensor.protocol.mode == SSF_MEMS_LINK_MODBUS && sensor.ota.pending == false);
    if (cases[i].error == 0) {
      check_image();
      CHECK(first_attempts == 2 && status.stage == SSF_MEMS_OTA_COMPLETE);
    } else if (cases[i].committed == true) {
      CHECK(status.stage == SSF_MEMS_OTA_UNCERTAIN);
      CHECK(sensor.acquisition.firmware_blocked == true && sleeps >= 15000);
      if (cases[i].fault == LOST_FINAL_ACK) {
        check_image();
        CHECK(status.version == 5101); /* Reachability does not turn lost final ACK into success. */
      }
    } else {
      CHECK(status.stage == SSF_MEMS_OTA_FAILED);
      CHECK(sensor.acquisition.firmware_blocked == false);
    }
  }
}

static void test_validation(const char *existing_bin) {
  struct ssf_mems_ota_status status;
  char name[SSF_MEMS_OTA_NAME_SIZE];
  u8 frame[263];
  u16 value;
  FILE *file;

  reset(513, NORMAL);
  sensor.protocol.mode = SSF_MEMS_LINK_OTA;
  mutex_lock(&sensor.protocol.bus_lock);
  CHECK(ssf_mems_modbus_read_reg_locked(&serial, 40121, &value, 1000) == -EAGAIN);
  CHECK(ssf_mems_modbus_read_reg_recovery_locked(&serial, 40121, &value, 1000) == -EAGAIN);
  mutex_unlock(&sensor.protocol.bus_lock);
  CHECK(sends == 0);
  sensor.protocol.mode = SSF_MEMS_LINK_MODBUS;
  CHECK(ssf_mems_ota_validate_image(NULL, 513) == -EINVAL);
  CHECK(ssf_mems_ota_validate_image(image, 7) == -EINVAL);
  CHECK(ssf_mems_ota_validate_image(image, sizeof(image)) == -EFBIG);
  CHECK(ssf_mems_ota_parse_name("update.bin\n", 11, name) == 0);
  CHECK(strcmp(name, "update.bin") == 0);
  CHECK(ssf_mems_ota_parse_name("../x.bin", 8, name) == -EINVAL);
  CHECK(ssf_mems_ota_parse_name("/tmp/x.bin", 10, name) == -EINVAL);
  CHECK(ssf_mems_ota_parse_name("x\0bad.bin", 9, name) == -EINVAL);
  CHECK(ssf_mems_ota_parse_name("x.bin\n\n", 7, name) == -EINVAL);
  CHECK(ssf_mems_ota_parse_name("x.hex", 5, name) == -EINVAL);
  CHECK(ssf_mems_ota_build_packet(0, 0, image, 256, frame) == -EINVAL);
  CHECK(ssf_mems_ota_build_packet(1, 0, image, 257, frame) == -EINVAL);
  image[6] = 0; /* Wrong link address: 0x08000041 rather than 0x08020041. */
  CHECK(ssf_mems_acquisition_update_firmware(&sensor, "update.bin") == -ENOEXEC);
  CHECK(sends == 0 && releases == 1);
  image[6] = 2;
  image[4] &= ~1U;
  CHECK(ssf_mems_ota_validate_image(image, 513) == -ENOEXEC);
  image[4] |= 1U;
  image[0] = 1;
  CHECK(ssf_mems_ota_validate_image(image, 513) == -ENOEXEC);
  reset(513, NORMAL);
  load_error = -ENOENT;
  CHECK(ssf_mems_acquisition_update_firmware(&sensor, "update.bin") == -ENOENT);
  CHECK(sends == 0 && releases == 0 && sensor.acquisition.online == true);
  ssf_mems_ota_get_status(&sensor, &status);
  CHECK(status.stage == SSF_MEMS_OTA_FAILED && status.error == -ENOENT);
  reset(513, NORMAL);
  stop_on_load = true;
  CHECK(ssf_mems_acquisition_update_firmware(&sensor, "update.bin") == -ENODEV);
  CHECK(sends == 0 && releases == 1);
  reset(513, NORMAL);
  sensor.protocol.mode = SSF_MEMS_LINK_RAW_RECOVERING;
  CHECK(ssf_mems_acquisition_update_firmware(&sensor, "update.bin") == -EBUSY);
  CHECK(sends == 0);
  reset(513, NORMAL);
  sensor.acquisition.online = false;
  CHECK(ssf_mems_acquisition_update_firmware(&sensor, "update.bin") == -ENODATA);
  CHECK(sends == 0);
  mutex_lock(&sensor.ota.update_lock);
  CHECK(ssf_mems_acquisition_update_firmware(&sensor, "update.bin") == -EBUSY);
  mutex_unlock(&sensor.ota.update_lock);

  /* The checked-in-source directory's optional build artifact is only read, never sent. */
  file = fopen(existing_bin, "rb");
  if (file != NULL) {
    size_t size = fread(image, 1, sizeof(image), file);
    u32 entry = size >= 8 ? (u32)image[4] | (u32)image[5] << 8 |
                            (u32)image[6] << 16 | (u32)image[7] << 24 : 0;

    fclose(file);
    if (size >= 8 && entry < SSF_MEMS_OTA_APP_ADDRESS) {
      CHECK(ssf_mems_ota_validate_image(image, size) == -ENOEXEC);
      printf("PASS existing MEMS2.0 bin rejected: wrong application link address\n");
    }
  }
}

/* Run the real entry on a second thread while the current acquisition window owns its lock. */
static void *update_worker(void *unused) {
  (void)unused;
  worker_result = ssf_mems_acquisition_update_firmware(&sensor, "update.bin");
  return NULL;
}

static void test_waiting_and_shutdown(void) {
  struct ssf_mems_ota_status status;
  pthread_t worker;
  unsigned int stop;

  for (stop = 0; stop < 2; stop++) {
    reset(513, NORMAL);
    acquisition_held_by_test = true;
    loader_entered = false;
    mutex_lock(&sensor.acquisition.lock);
    CHECK(pthread_create(&worker, NULL, update_worker, NULL) == 0);
    pthread_mutex_lock(&loader_lock);
    while (loader_entered == false)
      pthread_cond_wait(&loader_event, &loader_lock);
    pthread_mutex_unlock(&loader_lock);
    /* The file has loaded, yet not even a query may overtake the active raw window. */
    CHECK(sends == 0);
    ssf_mems_ota_get_status(&sensor, &status);
    CHECK(status.stage == SSF_MEMS_OTA_LOADING || status.stage == SSF_MEMS_OTA_WAITING);
    CHECK(ssf_mems_acquisition_update_firmware(&sensor, "update.bin") == -EBUSY);
    if (stop != 0)
      ssf_mems_ota_shutdown(&sensor);
    mutex_unlock(&sensor.acquisition.lock);
    CHECK(pthread_join(worker, NULL) == 0);
    CHECK(worker_result == (stop != 0 ? -ENODEV : 0));
    if (stop != 0)
      CHECK(sends == 0 && finals == 0);
    else
      check_image();
    acquisition_held_by_test = false;
  }
}

int main(int argc, char **argv) {
  CHECK(argc == 2);
  test_success();
  test_failures();
  test_validation(argv[1]);
  test_waiting_and_shutdown();
  cleanup();
  printf("PASS OTA framing, staged Flash, ACK faults, loader and lifecycle: %u checks\n", checks);
  return 0;
}
