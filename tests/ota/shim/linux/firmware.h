#pragma once
#include "common.h"
struct firmware {
  size_t size;
  const u8 *data;
};
int request_firmware_direct(const struct firmware **fw, const char *name,
                            struct device *dev);
void release_firmware(const struct firmware *fw);
