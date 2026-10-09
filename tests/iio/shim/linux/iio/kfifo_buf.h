#pragma once
#include <linux/iio/iio.h>
struct iio_buffer *iio_kfifo_allocate(void);
void iio_kfifo_free(struct iio_buffer *);
