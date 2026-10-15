#pragma once
#include "common.h"
static inline u16 get_unaligned_be16(const void *address) {
  const u8 *p = address;
  return (u16)p[0] << 8 | p[1];
}
static inline u32 get_unaligned_be32(const void *address) {
  const u8 *p = address;
  return (u32)p[0] << 24 | (u32)p[1] << 16 | (u32)p[2] << 8 | p[3];
}
