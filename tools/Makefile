CROSS_COMPILE ?=
ifeq ($(origin CC),default)
CC := $(CROSS_COMPILE)gcc
endif
CPPFLAGS ?=
CFLAGS ?= -O2 -Wall -Wextra -Wpedantic
# 默认静态链接，避免交叉工具链与开发板的 glibc 版本不匹配。
LDFLAGS ?= -static
LDLIBS ?=

.PHONY: all clean
all: iio_read

iio_read: iio_read.c Makefile
	$(CC) $(CPPFLAGS) $(CFLAGS) $(LDFLAGS) -o $@ $< $(LDLIBS)

clean:
	rm -f iio_read
