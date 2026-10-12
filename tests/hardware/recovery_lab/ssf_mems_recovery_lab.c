// SPDX-License-Identifier: GPL-2.0-only
/* 独立、有界的故障诊断；默认只读，不属于正式驱动。 */
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/serdev.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>

static unsigned int baud = 1000000; /* 本次已知故障速率；普通只读确认可用 9600。 */
module_param(baud, uint, 0444);
static bool recover_once; /* 显式允许一次采样率切换和恢复，会自动写两次 Flash。 */
module_param(recover_once, bool, 0444);
MODULE_PARM_DESC(recover_once, "Opt in to ONE sampling restart (two automatic Flash saves)");

/* 只保存 UART 交付字节，不复用正式驱动的解析器或 IIO 状态。 */
struct recovery_lab {
  struct serdev_device *serial; /* 本次独占的串口设备。 */
  struct work_struct work; /* 延后执行的有界诊断，不在 probe 中发帧。 */
  spinlock_t lock; /* 保护接收数组和计数。 */
  u8 bytes[512]; /* 单次请求的原始接收字节，过量字节只丢弃不越界。 */
  size_t used; /* 当前已保存的接收字节数。 */
};

/* 标准 Modbus RTU CRC；完整帧的返回值应为零。 */
static u16 lab_crc(const u8 *buf, size_t size)
{
  u16 crc = 0xffff;
  size_t i;
  unsigned int bit;

  for (i = 0; i < size; i++) {
    crc ^= buf[i];
    for (bit = 0; bit < 8; bit++)
      crc = (crc & 1U) != 0 ? (crc >> 1) ^ 0xa001 : crc >> 1;
  }
  return crc;
}

/* 回调只收集字节；完整性检查和日志留在工作线程中。 */
static int lab_receive(struct serdev_device *serial, const unsigned char *buf,
                       size_t count)
{
  struct recovery_lab *lab = serdev_device_get_drvdata(serial);
  unsigned long flags;
  size_t size;

  spin_lock_irqsave(&lab->lock, flags);
  size = min(count, sizeof(lab->bytes) - lab->used);
  memcpy(lab->bytes + lab->used, buf, size);
  lab->used += size;
  spin_unlock_irqrestore(&lab->lock, flags);
  return count;
}

/* 唤醒 serdev 同步发送接口，不从接收回调发送新请求。 */
static void lab_wakeup(struct serdev_device *serial)
{
  serdev_device_write_wakeup(serial);
}

static const struct serdev_device_ops lab_ops = {
  .receive_buf = lab_receive,
  .write_wakeup = lab_wakeup,
};

/* 仅在工作线程调用：发送一次八字节帧，收集 200 ms，不自动重发写命令。 */
static size_t lab_exchange(struct recovery_lab *lab, u8 function, u16 address,
                           u16 value, u8 *received)
{
  u8 frame[8] = {1, function, address >> 8, address, value >> 8, value};
  unsigned long flags;
  size_t size;
  u16 crc = lab_crc(frame, 6);
  int ret;

  frame[6] = crc;
  frame[7] = crc >> 8;
  spin_lock_irqsave(&lab->lock, flags);
  lab->used = 0;
  spin_unlock_irqrestore(&lab->lock, flags);
  ret = serdev_device_write(lab->serial, frame, sizeof(frame), HZ);
  serdev_device_wait_until_sent(lab->serial, HZ);
  msleep(200);
  spin_lock_irqsave(&lab->lock, flags);
  size = lab->used;
  memcpy(received, lab->bytes, size);
  spin_unlock_irqrestore(&lab->lock, flags);
  dev_info(&lab->serial->dev,
           "lab f=%u addr=%u value=%u sent=%d rx=%zu crc=%04x data=%*ph\n",
           function, address, value, ret, size, lab_crc(received, size),
           (int)min(size, (size_t)48), received);
  return ret == sizeof(frame) ? size : 0;
}

/* 只认地址、功能码、数量和 CRC 均正确的普通读应答。 */
static bool lab_valid_read(const u8 *buf, size_t size, u16 count)
{
  return size == 5U + count * 2U && buf[0] == 1 && buf[1] == 3 &&
         buf[2] == count * 2U && lab_crc(buf, size) == 0;
}

/* 地址 51 起的普通两寄存器读取，避开起始地址 53..59 的采集副作用。 */
static bool lab_read_sampling(struct recovery_lab *lab, u16 *rate, u16 *length)
{
  u8 received[512];
  size_t size;
  unsigned int i;

  for (i = 0; i < 10; i++) {
    size = lab_exchange(lab, 3, 51, 2, received);
    if (lab_valid_read(received, size, 2) == true) {
      *rate = received[3] * 256U + received[4];
      *length = received[5] * 256U + received[6];
      return true;
    }
    msleep(100);
  }
  return false;
}

/* 仅针对本板已核实的故障：索引 6/长度 0，临时改为 5，随后恢复 6。
 * 每次写最多发送一遍；回显丢失时只读定位，不能因此重复擦写 Flash。 */
static void lab_restart_sampling(struct recovery_lab *lab)
{
  u8 received[512];
  u16 rate, length;
  bool found;

  if (lab_read_sampling(lab, &rate, &length) == false ||
      rate != 6 || length != 0) {
    dev_err(&lab->serial->dev, "recovery aborted: expected sampling 6/0 not verified\n");
    return;
  }
  dev_warn(&lab->serial->dev, "ONE sampling restart: 6 -> 5 -> 6; firmware auto-saves Flash\n");
  lab_exchange(lab, 6, 51, 5, received);
  msleep(500);
  found = false;
  if (serdev_device_set_baudrate(lab->serial, 9600) == 9600)
    found = lab_read_sampling(lab, &rate, &length);
  if (found == false &&
      serdev_device_set_baudrate(lab->serial, 1000000) == 1000000)
    found = lab_read_sampling(lab, &rate, &length);
  if (found == false || length != 0 || (rate != 5 && rate != 6)) {
    dev_err(&lab->serial->dev, "recovery unresolved: sampling 6/0 MUST be checked; no blind writes\n");
    return;
  }
  if (rate == 5) {
    lab_exchange(lab, 6, 51, 6, received);
    msleep(2000); /* 给固件重配、保存原采样参数留出时间。 */
  }
  if (serdev_device_set_baudrate(lab->serial, 9600) == 9600 &&
      lab_read_sampling(lab, &rate, &length) == true && rate == 6 && length == 0)
    dev_info(&lab->serial->dev, "recovery verified at 9600; sampling 6/0 restored\n");
  else
    dev_err(&lab->serial->dev, "recovery NOT verified; no automatic repeat\n");
}

/* 默认只读十次；恢复需显式开启，且现场串口配置和停止标志都已核实。 */
static void lab_work(struct work_struct *work)
{
  static const u8 expected[] = {0, 1, 0, 3, 0, 0, 0, 255, 0, 1};
  struct recovery_lab *lab = container_of(work, struct recovery_lab, work);
  u8 received[512];
  bool verified_config = false, verified_stop = false;
  u16 rate, length;
  unsigned int i;
  size_t size;

  msleep(500);
  for (i = 0; i < 10; i++) {
    if (i % 2U == 0) {
      size = lab_exchange(lab, 3, 100, 5, received);
      if (lab_valid_read(received, size, 5) == true) {
        if (memcmp(received + 3, expected, sizeof(expected)) != 0) {
          dev_err(&lab->serial->dev, "configuration differs from tested board; recovery forbidden\n");
          return;
        }
        verified_config = true;
      }
    } else {
      size = lab_exchange(lab, 3, 52, 7, received);
      if (lab_valid_read(received, size, 7) == true)
        verified_stop = received[15] == 0 && received[16] == 1;
    }
    msleep(300);
  }
  if (recover_once == true) {
    if (baud != 1000000 || verified_config == false || verified_stop == false) {
      dev_err(&lab->serial->dev, "recovery forbidden: stalled 1 Mbaud/stop=1 not verified\n");
      return;
    }
    lab_restart_sampling(lab);
  } else if (lab_read_sampling(lab, &rate, &length) == true) {
    dev_info(&lab->serial->dev, "readonly sampling index=%u length=%u\n", rate, length);
  }
  dev_info(&lab->serial->dev, "bounded diagnostic finished; no loop or 40110 command\n");
}

/* probe 仅打开并设置主机 UART；总线帧由延后工作执行。 */
static int lab_probe(struct serdev_device *serial)
{
  struct recovery_lab *lab;
  int ret;

  if ((baud != 9600 && baud != 1000000) ||
      (recover_once == true && baud != 1000000))
    return -EINVAL;
  lab = devm_kzalloc(&serial->dev, sizeof(*lab), GFP_KERNEL);
  if (lab == NULL)
    return -ENOMEM;
  lab->serial = serial;
  spin_lock_init(&lab->lock);
  INIT_WORK(&lab->work, lab_work);
  serdev_device_set_drvdata(serial, lab);
  serdev_device_set_client_ops(serial, &lab_ops);
  ret = serdev_device_open(serial);
  if (ret != 0)
    return ret;
  serdev_device_set_flow_control(serial, false);
  ret = serdev_device_set_parity(serial, SERDEV_PARITY_NONE);
  if (ret != 0)
    goto close;
  if (serdev_device_set_baudrate(serial, baud) != baud) {
    ret = -EIO;
    goto close;
  }
  schedule_work(&lab->work);
  return 0;
close:
  serdev_device_close(serial);
  return ret;
}

/* 等待有界诊断退出；接收状态必须保留到工作结束和串口关闭之后。 */
static void lab_remove(struct serdev_device *serial)
{
  struct recovery_lab *lab = serdev_device_get_drvdata(serial);

  cancel_work_sync(&lab->work);
  serdev_device_close(serial);
}

static const struct of_device_id lab_match[] = {
  {.compatible = "sange-cbm,ssf-mems-xyzs"}, {}
};
static struct serdev_device_driver lab_driver = {
  .driver = {.name = "ssf_mems_recovery_lab", .of_match_table = lab_match},
  .probe = lab_probe,
  .remove = lab_remove,
};
module_serdev_device_driver(lab_driver);
MODULE_LICENSE("GPL");
