#pragma once
#include "common.h"
#define ALIGN(value, alignment) (((value) + (alignment) - 1) & ~((alignment) - 1))
#define __aligned(n) __attribute__((aligned(n)))
#define for_each_set_bit(bit, mask, size) \
  for ((bit) = 0; (bit) < (size); (bit)++) if (*(mask) & BIT(bit))
