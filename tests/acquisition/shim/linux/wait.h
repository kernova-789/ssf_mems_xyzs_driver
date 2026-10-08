#pragma once
#include "common.h"
void test_acquisition_wait(unsigned long timeout);
#undef wait_event_interruptible_timeout
#define wait_event_interruptible_timeout(q, condition, timeout) \
  ({ long result; (void)(q); \
     if (condition) result = 1; \
     else { test_acquisition_wait(timeout); result = (condition) ? 1 : 0; } \
     result; })
