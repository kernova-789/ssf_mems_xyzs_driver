#pragma once
#include "common.h"
#include <stdatomic.h>
struct completion { atomic_bool done; };
void test_completion_notify(void);
int test_completion_wait(struct completion *done, bool interruptible);
static inline void init_completion(struct completion *done) { atomic_init(&done->done, false); }
static inline void complete(struct completion *done) {
  atomic_store(&done->done, true);
  test_completion_notify();
}
static inline int wait_for_completion_interruptible(struct completion *done) {
  return test_completion_wait(done, true);
}
static inline void wait_for_completion(struct completion *done) {
  (void)test_completion_wait(done, false);
}
