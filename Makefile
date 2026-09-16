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

# .ko 自动复制到这个目录
INSTALL_DIR ?= /home/u-yoimiya/nfs/

.PHONY: all prepare install clean

all: prepare
	$(MAKE) -C $(KDIR) M=$(CURDIR) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) modules
	@mkdir -p $(INSTALL_DIR)
	@cp -f $(CURDIR)/ssf_mems.ko $(INSTALL_DIR)/
	@echo "Installed: $(INSTALL_DIR)/ssf_mems.ko"

prepare:
	$(MAKE) -C $(KDIR) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) modules_prepare

install:
	@mkdir -p $(INSTALL_DIR)
	@cp -f $(CURDIR)/ssf_mems.ko $(INSTALL_DIR)/
	@echo "Installed: $(INSTALL_DIR)/ssf_mems.ko"

clean:
	$(MAKE) -C $(KDIR) M=$(CURDIR) clean