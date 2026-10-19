#pragma once
#include <linux/iio/iio.h>
struct attribute { const char *name; umode_t mode; };
struct device_attribute {
  struct attribute attr;
  ssize_t (*show)(struct device *, struct device_attribute *, char *);
  ssize_t (*store)(struct device *, struct device_attribute *, const char *, size_t);
};
struct iio_dev_attr { struct device_attribute dev_attr; unsigned long address; };
struct attribute_group { struct attribute **attrs;
 umode_t (*is_visible)(struct kobject *, struct attribute *, int); };
#define IIO_DEVICE_ATTR(n, permissions, show_fn, store_fn, addr) \
  struct iio_dev_attr iio_dev_attr_##n = { \
    .dev_attr = { .attr = { .name = #n, .mode = permissions }, .show = show_fn, .store = store_fn }, .address = addr }
#define IIO_CONST_ATTR(n, value) \
  struct iio_dev_attr iio_const_attr_##n = { .dev_attr.attr = { .name = #n, .mode = 0444 } }
#define to_iio_dev_attr(attr) container_of(attr, struct iio_dev_attr, dev_attr)
int sysfs_emit(char *, const char *, ...);
int kstrtouint(const char *, unsigned int, unsigned int *);

int sysfs_emit_at(char *buf, int at, const char *fmt, ...);
