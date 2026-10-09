#pragma once
#include <linux/types.h>
enum { IIO_ACCEL, IIO_VELOCITY, IIO_TEMP, IIO_INDEX, IIO_COUNT, IIO_TIMESTAMP };
enum { IIO_MOD_X, IIO_MOD_Y, IIO_MOD_Z, IIO_CPU };
enum { IIO_CHAN_INFO_RAW, IIO_CHAN_INFO_SCALE };
enum { IIO_VAL_INT = 1, IIO_VAL_INT_PLUS_MICRO, IIO_VAL_INT_PLUS_NANO };
#define INDIO_DIRECT_MODE BIT(0)
#define INDIO_BUFFER_SOFTWARE BIT(1)
struct iio_chan_spec {
  int type, modified, indexed, channel, channel2, scan_index;
  const char *extend_name;
  unsigned long address, info_mask_separate;
  struct { char sign; int realbits, storagebits, endianness; } scan_type;
};
struct iio_dev;
struct iio_info {
  const struct attribute_group *attrs;
  int (*read_raw)(struct iio_dev *, const struct iio_chan_spec *, int *, int *, long);
};
struct iio_buffer_setup_ops { int unused; };
struct iio_buffer { int unused; };
struct iio_dev {
  struct mutex mlock;
  struct device dev;
  struct iio_buffer *buffer;
  unsigned long *active_scan_mask;
  size_t scan_bytes;
  bool scan_timestamp;
  unsigned int currentmode, modes;
  void *private;
  const char *name;
  const struct iio_info *info;
  const struct iio_chan_spec *channels;
  size_t num_channels;
  const struct iio_buffer_setup_ops *setup_ops;
};
#define IIO_CHAN_SOFT_TIMESTAMP(index) { .type = IIO_TIMESTAMP, .scan_index = (index) }
static inline void *iio_priv(struct iio_dev *dev) { return dev->private; }
static inline struct iio_dev *dev_to_iio_dev(struct device *dev) {
  return container_of(dev, struct iio_dev, dev);
}
static inline bool iio_buffer_enabled(struct iio_dev *dev) {
  return dev->currentmode == INDIO_BUFFER_SOFTWARE;
}
s64 iio_get_time_ns(struct iio_dev *dev);
struct iio_dev *devm_iio_device_alloc(struct device *, int);
void iio_device_attach_buffer(struct iio_dev *, struct iio_buffer *);
int iio_device_register(struct iio_dev *);
void iio_device_unregister(struct iio_dev *);
