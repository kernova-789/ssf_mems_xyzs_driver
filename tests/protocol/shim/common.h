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
struct serdev_device {
  void *drvdata;
  int dev;
  unsigned int baudrate;
  unsigned int baudrate_set_count;
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
static inline void serdev_device_wait_until_sent(struct serdev_device *s, unsigned long t) { (void)s; (void)t; }
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
  s->baudrate = baudrate;
  s->baudrate_set_count++;
  return baudrate;
}

ssize_t serdev_device_write(struct serdev_device *s, const u8 *buf, size_t size, unsigned long timeout);
#endif
