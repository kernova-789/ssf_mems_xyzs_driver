// SPDX-License-Identifier: GPL-2.0-only
/* 独立实机实验：不链接正式驱动、不发布 IIO、不自动保存或重启传感器。 */
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/jiffies.h>
#include <linux/ktime.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/serdev.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/vmalloc.h>
#include <linux/workqueue.h>

static unsigned int seconds = 60;
module_param(seconds, uint, 0444);
static unsigned int rounds = 3;
module_param(rounds, uint, 0444);
static unsigned int first_timeout = 45;
module_param(first_timeout, uint, 0444);
static char *port = "";
module_param(port, charp, 0444);
/* 仅人工指定故障恢复轮：确认持久配置仍为128000后保存/重启一次。 */
static bool recover_only;
module_param(recover_only, bool, 0444);

#define BLOB_SIZE (16U * 1024U * 1024U)
struct lab {
  struct serdev_device *serial;
  struct work_struct work;
  spinlock_t lock;
  bool stopping, active, done;
  struct dentry *debug;
  u8 *blob;
  u32 blob_used, blob_lost;
  u8 response[512], packet[392];
  u32 response_used, overflow, partial, packets, bad, gaps, ignored, next;
  u64 first_ns, last_packet_ns, last_byte_ns, max_gap_ns;
};

static u16 crc16(const u8 *p, unsigned int size)
{
  u16 crc = 0xffff;
  unsigned int i, bit;
  for (i = 0; i < size; i++) {
    crc ^= p[i];
    for (bit = 0; bit < 8; bit++)
      crc = (crc & 1) ? (crc >> 1) ^ 0xa001 : crc >> 1;
  }
  return crc;
}

static void byte_in(struct lab *lab, u8 byte, u64 now)
{
  u32 seq;
  if (!lab->partial && byte != 0x15) {
    lab->ignored++;
    return;
  }
  lab->packet[lab->partial++] = byte;
  if (lab->partial != sizeof(lab->packet))
    return;
  if (lab->packet[391] == 0x17 && !crc16(lab->packet, 391)) {
    seq = (u32)lab->packet[1] << 24 | (u32)lab->packet[2] << 16 |
          (u32)lab->packet[3] << 8 | lab->packet[4];
    if (seq != lab->next)
      lab->gaps++;
    lab->next = seq + 1;
    if (!lab->first_ns)
      lab->first_ns = now;
    else
      lab->max_gap_ns = max(lab->max_gap_ns, now - lab->last_packet_ns);
    lab->last_packet_ns = now;
    lab->packets++;
    lab->partial = 0;
    return;
  }
  lab->bad++;
  do {
    lab->partial--;
    memmove(lab->packet, lab->packet + 1, lab->partial);
  } while (lab->partial && lab->packet[0] != 0x15);
}

static int receive(struct serdev_device *serial, const unsigned char *buf, size_t count)
{
  struct lab *lab = serdev_device_get_drvdata(serial);
  unsigned long flags;
  size_t n, i;
  u64 now = ktime_get_ns();
  spin_lock_irqsave(&lab->lock, flags);
  lab->last_byte_ns = now;
  if (lab->active) {
    n = min_t(size_t, count, BLOB_SIZE - lab->blob_used);
    memcpy(lab->blob + lab->blob_used, buf, n);
    lab->blob_used += n;
    lab->blob_lost += count - n;
    for (i = 0; i < count; i++)
      byte_in(lab, buf[i], now);
  } else {
    n = min_t(size_t, count, sizeof(lab->response) - lab->response_used);
    memcpy(lab->response + lab->response_used, buf, n);
    lab->response_used += n;
    lab->overflow += count - n;
  }
  spin_unlock_irqrestore(&lab->lock, flags);
  return count;
}

static void wakeup(struct serdev_device *serial)
{
  serdev_device_write_wakeup(serial);
}
static const struct serdev_device_ops ops = {.receive_buf = receive, .write_wakeup = wakeup};

static int send(struct lab *lab, u8 fn, u16 addr, u16 value)
{
  u8 frame[8] = {1, fn, addr >> 8, addr, value >> 8, value};
  u16 crc = crc16(frame, 6);
  int sent;
  frame[6] = crc;
  frame[7] = crc >> 8;
  sent = serdev_device_write(lab->serial, frame, sizeof(frame), HZ);
  serdev_device_wait_until_sent(lab->serial, HZ);
  dev_info(&lab->serial->dev, "CL TX fn=%u addr=%u value=%u sent=%d\n", fn, addr, value, sent);
  return sent == sizeof(frame) ? 0 : -EIO;
}

static bool exchange(struct lab *lab, u8 fn, u16 addr, u16 value, u16 *out)
{
  u8 response[512];
  unsigned int used, overflow;
  unsigned long flags;
  int ret;
  spin_lock_irqsave(&lab->lock, flags);
  lab->active = false;
  lab->response_used = lab->overflow = 0;
  spin_unlock_irqrestore(&lab->lock, flags);
  ret = send(lab, fn, addr, value);
  msleep(250);
  spin_lock_irqsave(&lab->lock, flags);
  used = lab->response_used;
  overflow = lab->overflow;
  memcpy(response, lab->response, used);
  spin_unlock_irqrestore(&lab->lock, flags);
  dev_info(&lab->serial->dev, "CL RX fn=%u addr=%u value=%u size=%u overflow=%u crc=%04x data=%*ph\n",
           fn, addr, value, used, overflow, crc16(response, used), min(used, 80U), response);
  if (ret || overflow || crc16(response, used))
    return false;
  if (fn == 3 && used == value * 2U + 5U && response[0] == 1 &&
      response[1] == 3 && response[2] == value * 2U) {
    if (out)
      *out = response[3] * 256U + response[4];
    return true;
  }
  return fn == 6 && used == 8 && response[0] == 1 && response[1] == 6 &&
         response[2] == addr >> 8 && response[3] == (u8)addr &&
         response[4] == value >> 8 && response[5] == (u8)value;
}

static bool ordinary(struct lab *lab)
{
  unsigned int i;
  u16 value;
  if (serdev_device_set_baudrate(lab->serial, 128000) != 128000)
    return false;
  for (i = 0; i < 5; i++) {
    if (exchange(lab, 3, 101, 1, &value) && value == 8)
      return true;
    msleep(300);
  }
  return false;
}

/* 不期待40059写回显；以接收安静和普通速率读回共同确认停流。 */
static bool stop_stream(struct lab *lab)
{
  unsigned int attempt;
  unsigned long deadline;
  bool quiet = false;
  for (attempt = 1; attempt <= 3; attempt++) {
    deadline = jiffies + msecs_to_jiffies(2000);
    while (time_before(jiffies, deadline)) {
      if (!READ_ONCE(lab->partial) &&
          ktime_get_ns() - READ_ONCE(lab->last_byte_ns) > 200000)
        break;
      usleep_range(300, 600);
    }
    dev_info(&lab->serial->dev, "CL STOP attempt=%u packets=%u partial=%u\n",
             attempt, lab->packets, lab->partial);
    send(lab, 6, 58, 1);
    deadline = jiffies + msecs_to_jiffies(5000);
    while (time_before(jiffies, deadline)) {
      if (ktime_get_ns() - READ_ONCE(lab->last_byte_ns) > 300000000ULL) {
        quiet = true;
        break;
      }
      msleep(20);
    }
    if (quiet)
      break;
  }
  if (quiet && ordinary(lab)) {
    dev_info(&lab->serial->dev, "CL STOP VERIFIED attempts=%u ordinary=128000\n", attempt);
    return true;
  }
  /* 保留故障现场，只做诊断，不自动保存、复位或再启动。 */
  serdev_device_set_baudrate(lab->serial, 1000000);
  exchange(lab, 3, 101, 1, NULL);
  exchange(lab, 3, 52, 7, NULL);
  dev_err(&lab->serial->dev, "CL STOP UNCONFIRMED; no automatic reset/save\n");
  return false;
}

static void work(struct work_struct *item)
{
  struct lab *lab = container_of(item, struct lab, work);
  unsigned long flags, deadline;
  unsigned int round;
  u32 begin, packets, bad, gaps, ignored, partial;
  u16 value;
  u64 started, first, end;
  bool restored, full;
  msleep(500);
  if (recover_only) {
    if (exchange(lab, 3, 101, 1, &value) && value == 8) {
      dev_warn(&lab->serial->dev, "CL RECOVERY ONLY: ONE save/reboot; not a test pass\n");
      exchange(lab, 6, 109, 1, NULL);
      msleep(15000);
      dev_info(&lab->serial->dev, "CL RECOVERY ordinary_verified=%u\n", ordinary(lab));
    } else {
      dev_err(&lab->serial->dev, "CL RECOVERY refused: stored baud unverified\n");
    }
    goto done;
  }
  if (!ordinary(lab) || !exchange(lab, 3, 51, 1, &value) || value != 6) {
    dev_err(&lab->serial->dev, "CL ABORT initial 128000/sampling6 unverified\n");
    goto done;
  }
  for (round = 1; round <= rounds && !READ_ONCE(lab->stopping); round++) {
    spin_lock_irqsave(&lab->lock, flags);
    lab->partial = lab->packets = lab->bad = lab->gaps = lab->ignored = lab->next = 0;
    lab->first_ns = lab->last_packet_ns = lab->max_gap_ns = 0;
    lab->active = true;
    begin = lab->blob_used;
    spin_unlock_irqrestore(&lab->lock, flags);
    started = ktime_get_ns();
    send(lab, 3, 57, 1);
    if (serdev_device_set_baudrate(lab->serial, 1000000) != 1000000) {
      dev_err(&lab->serial->dev, "CL host failed to select 1 Mbaud\n");
      break;
    }
    dev_info(&lab->serial->dev, "CL START round=%u first_timeout=%u seconds=%u offset=%u\n",
             round, first_timeout, seconds, begin);
    deadline = jiffies + first_timeout * HZ;
    while (!READ_ONCE(lab->first_ns) && time_before(jiffies, deadline) &&
           !READ_ONCE(lab->stopping))
      msleep(10);
    first = READ_ONCE(lab->first_ns);
    if (first)
      dev_info(&lab->serial->dev, "CL FIRST round=%u delay_us=%llu\n", round, (first-started)/1000);
    while (first && ktime_get_ns() - first < (u64)seconds * NSEC_PER_SEC &&
           !READ_ONCE(lab->stopping)) {
      if (ktime_get_ns() - READ_ONCE(lab->last_packet_ns) > 10000000000ULL)
        break;
      msleep(20);
    }
    end = ktime_get_ns();
    spin_lock_irqsave(&lab->lock, flags);
    packets = lab->packets; bad = lab->bad; gaps = lab->gaps;
    ignored = lab->ignored; partial = lab->partial;
    spin_unlock_irqrestore(&lab->lock, flags);
    full = first && end - first >= (u64)seconds * NSEC_PER_SEC;
    dev_info(&lab->serial->dev,
             "CL STREAM round=%u elapsed_us=%llu packets=%u bad=%u gaps=%u ignored=%u partial=%u max_gap_us=%llu offset=%u\n",
             round, first ? (end-first)/1000 : 0, packets, bad, gaps, ignored, partial,
             READ_ONCE(lab->max_gap_ns)/1000, READ_ONCE(lab->blob_used));
    restored = stop_stream(lab);
    dev_info(&lab->serial->dev,
             "CL FINISH round=%u full=%u restored=%u stream_clean=%u offset=%u blob_lost=%u\n",
             round, full, restored, full && packets && !bad && !gaps && !ignored,
             READ_ONCE(lab->blob_used), READ_ONCE(lab->blob_lost));
    if (!restored || !full)
      break;
    msleep(1000);
  }
done:
  WRITE_ONCE(lab->done, true);
  dev_info(&lab->serial->dev, "CL DONE\n");
}

static ssize_t blob_read(struct file *file, char __user *buf, size_t size, loff_t *pos)
{
  struct lab *lab = file->private_data;
  return simple_read_from_buffer(buf, size, pos, lab->blob, READ_ONCE(lab->blob_used));
}
static const struct file_operations blob_ops = {
  .owner = THIS_MODULE, .open = simple_open, .read = blob_read, .llseek = default_llseek,
};

static int status_show(struct seq_file *seq, void *unused)
{
  struct lab *lab = seq->private;
  seq_printf(seq, "done=%u bytes=%u lost=%u packets=%u bad=%u gaps=%u ignored=%u partial=%u\n",
             READ_ONCE(lab->done), READ_ONCE(lab->blob_used), READ_ONCE(lab->blob_lost),
             READ_ONCE(lab->packets), READ_ONCE(lab->bad), READ_ONCE(lab->gaps),
             READ_ONCE(lab->ignored), READ_ONCE(lab->partial));
  return 0;
}
DEFINE_SHOW_ATTRIBUTE(status);

static int probe(struct serdev_device *serial)
{
  struct lab *lab;
  char name[80];
  int ret;
  if (*port && strcmp(port, dev_name(&serial->dev)))
    return -ENODEV;
  if (!seconds || seconds > 120 || !rounds || rounds > 5 || first_timeout < 15 || first_timeout > 90)
    return -EINVAL;
  lab = devm_kzalloc(&serial->dev, sizeof(*lab), GFP_KERNEL);
  if (!lab)
    return -ENOMEM;
  lab->blob = vzalloc(BLOB_SIZE);
  if (!lab->blob)
    return -ENOMEM;
  lab->serial = serial;
  spin_lock_init(&lab->lock);
  INIT_WORK(&lab->work, work);
  serdev_device_set_drvdata(serial, lab);
  serdev_device_set_client_ops(serial, &ops);
  ret = serdev_device_open(serial);
  if (ret)
    goto free;
  serdev_device_set_flow_control(serial, false);
  ret = serdev_device_set_parity(serial, SERDEV_PARITY_NONE);
  if (ret || serdev_device_set_baudrate(serial, recover_only ? 1000000 : 128000) !=
             (recover_only ? 1000000 : 128000)) {
    ret = ret ?: -EIO;
    serdev_device_close(serial);
    goto free;
  }
  snprintf(name, sizeof(name), "mems-continuous-%s", dev_name(&serial->dev));
  lab->debug = debugfs_create_dir(name, NULL);
  debugfs_create_file("raw.bin", 0400, lab->debug, lab, &blob_ops);
  debugfs_create_file("status", 0400, lab->debug, lab, &status_fops);
  schedule_work(&lab->work);
  return 0;
free:
  vfree(lab->blob);
  return ret;
}

static void remove(struct serdev_device *serial)
{
  struct lab *lab = serdev_device_get_drvdata(serial);
  WRITE_ONCE(lab->stopping, true);
  cancel_work_sync(&lab->work);
  debugfs_remove_recursive(lab->debug);
  serdev_device_close(serial);
  vfree(lab->blob);
}
static const struct of_device_id matches[] = {{.compatible = "sange-cbm,ssf-mems-xyzs"}, {}};
static struct serdev_device_driver continuous_driver = {
  .driver = {.name = "mems_continuous_lab", .of_match_table = matches},
  .probe = probe, .remove = remove,
};
module_serdev_device_driver(continuous_driver);
MODULE_LICENSE("GPL");
