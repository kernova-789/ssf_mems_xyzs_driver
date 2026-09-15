# Build this directory as an external kernel module.
obj-m += ssf_mems.o
ssf_mems-y += core/core.o
# ssf_mems-y += data/temperature.o
# ssf_mems-y += data/vibration.o
# ssf_mems-y += protocol/modbus.o
# ssf_mems-y += protocol/protocol.o
# ssf_mems-y += sensor/sensor.o
# ssf_mems-y += serdev/serdev.o

ccflags-y += -I$(src)/core
ccflags-y += -I$(src)/data
ccflags-y += -I$(src)/protocol
ccflags-y += -I$(src)/sensor
ccflags-y += -I$(src)/serdev

KDIR ?= /home/u-yoimiya/rk3588/kernel
ARCH ?= arm64
CROSS_COMPILE ?= aarch64-linux-gnu-

.PHONY: all prepare clean

all: prepare
	$(MAKE) -C $(KDIR) M=$(CURDIR) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) modules

prepare:
	$(MAKE) -C $(KDIR) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) modules_prepare

clean:
	$(MAKE) -C $(KDIR) M=$(CURDIR) clean
