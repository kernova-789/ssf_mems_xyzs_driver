#pragma once
#include "common.h"
#ifdef SSF_TEST_THREADED_TASK
struct task_struct { pthread_t native; int (*fn)(void *); void *context; };
#else
struct task_struct { int unused; };
#endif
struct task_struct *test_kthread_run(int (*fn)(void *), void *data,
                                    const char *name, ...);
#define kthread_run(fn, data, ...) test_kthread_run(fn, data, __VA_ARGS__)
bool kthread_should_stop(void);
int kthread_stop(struct task_struct *thread);
