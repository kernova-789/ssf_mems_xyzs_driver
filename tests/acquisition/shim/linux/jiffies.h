#pragma once
#include "common.h"
extern unsigned long jiffies;
#define time_after_eq(a, b) ((long)((a) - (b)) >= 0)
static inline unsigned int jiffies_to_msecs(unsigned long ticks) {
  return ticks;
}
