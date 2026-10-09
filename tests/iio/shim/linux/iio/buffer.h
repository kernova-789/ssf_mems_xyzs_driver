#pragma once
#include <linux/iio/iio.h>
int iio_push_to_buffers_with_timestamp(struct iio_dev *, void *, s64);
