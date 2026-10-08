#ifndef SSF_PROTOCOL_TEST_COMMON_H
#define SSF_PROTOCOL_TEST_COMMON_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <sys/types.h>
#include <asm-generic/errno.h>

typedef uint8_t u8;
typedef int16_t s16;
typedef uint16_t u16;
typedef uint32_t u32;
#define BIT(n) (1UL << (n))
#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
#define GFP_KERNEL 0
#define ERESTARTSYS 512
#define READ_ONCE(v) (v)
#define WRITE_ONCE(v, value) ((v) = (value))
#define container_of(ptr, type, member) ((type *)((char *)(ptr) - offsetof(type, member)))

struct mutex { int unused; };
typedef struct { int unused; } wait_queue_head_t;
typedef int spinlock_t;
typedef struct { int counter; } atomic_t;
struct work_struct { void (*fn)(struct work_struct *); };
struct device {
  bool has_slave_id, has_current_speed, has_parity;
  u32 slave_id, current_speed;
  const char *parity;
  int slave_id_read_error, current_speed_read_error, parity_read_error;
  unsigned int info_count;
  char last_info[256];
};
enum serdev_parity {
  SERDEV_PARITY_NONE,
  SERDEV_PARITY_EVEN,
  SERDEV_PARITY_ODD,
};
struct serdev_device {
  void *drvdata;
  struct device dev;
  unsigned int baudrate;
  unsigned int baudrate_set_count;
  unsigned int actual_baudrate;
  bool baudrate_set_failure;
  enum serdev_parity parity;
  unsigned int parity_set_count;
  int parity_set_error;
  bool flow_control;
  unsigned long write_timeout;
  unsigned long wait_timeout;
};
struct kfifo { u8 *data; size_t capacity, head, tail, count; };
struct ssf_mems_xyzs_data;

extern bool test_allocation_failure;
extern long test_wait_result;
extern unsigned int test_queue_count, test_cancel_count, test_wake_count;
#define mutex_init(p) ((p)->unused = 0)
#define mutex_lock(p) ((void)(p))
#define mutex_unlock(p) ((void)(p))
#define spin_lock_init(p) (*(p) = 0)
#define spin_lock_irqsave(p, flags) do { (void)(p); (flags) = 0; } while (0)
#define spin_unlock_irqrestore(p, flags) ((void)(p), (void)(flags))
#define init_waitqueue_head(p) ((void)(p))
#define wake_up_interruptible(p) ((void)(p), test_wake_count++)
#define INIT_WORK(p, f) ((p)->fn = (f))
#define cancel_work_sync(p) ((void)(p), test_cancel_count++)
#define queue_work(wq, work) ((void)(wq), (void)(work), test_queue_count++)
#define system_wq NULL
#define msecs_to_jiffies(ms) (ms)
#define wait_event_interruptible_timeout(q, condition, timeout) ((void)(q), (void)(timeout), (condition) ? 1L : test_wait_result)
#define dev_err(dev, ...) ((void)(dev))
#define dev_warn(dev, ...) ((void)(dev))
#define dev_dbg(dev, ...) ((void)(dev))
#define dev_info(dev, ...) ((dev)->info_count++, \
                           (void)snprintf((dev)->last_info, \
                                         sizeof((dev)->last_info), __VA_ARGS__))

static inline int atomic_read(const atomic_t *p) { return p->counter; }
static inline void atomic_set(atomic_t *p, int value) { p->counter = value; }
static inline int atomic_cmpxchg(atomic_t *p, int old, int value) {
  int previous = p->counter;
  if (previous == old) p->counter = value;
  return previous;
}
static inline void *kmalloc(size_t size, int flags) {
  (void)flags;
  return test_allocation_failure ? NULL : malloc(size);
}
static inline void *kzalloc(size_t size, int flags) {
  (void)flags;
  return test_allocation_failure ? NULL : calloc(1, size);
}
static inline void kfree(void *p) { free(p); }
static inline void *serdev_device_get_drvdata(struct serdev_device *s) { return s->drvdata; }
static inline void serdev_device_wait_until_sent(struct serdev_device *s,
                                                  unsigned long t) {
  s->wait_timeout = t;
}
static inline int kfifo_alloc(struct kfifo *fifo, size_t size, int flags) {
  memset(fifo, 0, sizeof(*fifo));
  fifo->data = kmalloc(size, flags);
  if (!fifo->data) return -ENOMEM;
  fifo->capacity = size;
  return 0;
}
static inline void kfifo_free(struct kfifo *fifo) {
  kfree(fifo->data);
  memset(fifo, 0, sizeof(*fifo));
}
static inline void kfifo_reset(struct kfifo *fifo) {
  fifo->head = fifo->tail = fifo->count = 0;
}
static inline unsigned int kfifo_in(struct kfifo *fifo, const u8 *buf, size_t size) {
  size_t i;
  for (i = 0; i < size && fifo->count < fifo->capacity; i++) {
    fifo->data[fifo->tail++ % fifo->capacity] = buf[i];
    fifo->count++;
  }
  return i;
}
static inline unsigned int kfifo_out_spinlocked(struct kfifo *fifo, u8 *buf, size_t size, spinlock_t *lock) {
  size_t i;
  (void)lock;
  for (i = 0; i < size && fifo->count; i++) {
    buf[i] = fifo->data[fifo->head++ % fifo->capacity];
    fifo->count--;
  }
  return i;
}

static inline unsigned int
serdev_device_set_baudrate(struct serdev_device *s, unsigned int baudrate) {
  s->baudrate_set_count++;
  if (s->baudrate_set_failure)
    return 0;
  s->baudrate = s->actual_baudrate ? s->actual_baudrate : baudrate;
  return s->baudrate;
}

static inline int serdev_device_set_parity(struct serdev_device *s,
                                           enum serdev_parity parity) {
  s->parity_set_count++;
  if (s->parity_set_error)
    return s->parity_set_error;
  s->parity = parity;
  return 0;
}

static inline void serdev_device_set_flow_control(struct serdev_device *s,
                                                 bool enabled) {
  s->flow_control = enabled;
}

static inline bool device_property_present(struct device *dev,
                                           const char *name) {
  if (!strcmp(name, "sange-cbm,slave-id"))
    return dev->has_slave_id;
  if (!strcmp(name, "current-speed"))
    return dev->has_current_speed;
  if (!strcmp(name, "sange-cbm,parity"))
    return dev->has_parity;
  return false;
}

static inline int device_property_read_u32(struct device *dev,
                                           const char *name, u32 *value) {
  if (!strcmp(name, "sange-cbm,slave-id") && dev->has_slave_id) {
    if (dev->slave_id_read_error)
      return dev->slave_id_read_error;
    *value = dev->slave_id;
    return 0;
  }
  if (!strcmp(name, "current-speed") && dev->has_current_speed) {
    if (dev->current_speed_read_error)
      return dev->current_speed_read_error;
    *value = dev->current_speed;
    return 0;
  }
  return -EINVAL;
}

static inline int device_property_read_string(struct device *dev,
                                              const char *name,
                                              const char **value) {
  if (strcmp(name, "sange-cbm,parity") || !dev->has_parity)
    return -EINVAL;
  if (dev->parity_read_error)
    return dev->parity_read_error;
  *value = dev->parity;
  return 0;
}

ssize_t serdev_device_write(struct serdev_device *s, const u8 *buf, size_t size, unsigned long timeout);
#endif
