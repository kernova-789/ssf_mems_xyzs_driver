#pragma once
#include "common.h"
struct task_struct { int unused; };
struct task_struct *test_kthread_run(int (*fn)(void *), void *data,
                                    const char *name, ...);
#define kthread_run(fn, data, ...) test_kthread_run(fn, data, __VA_ARGS__)
bool kthread_should_stop(void);
int kthread_stop(struct task_struct *thread);
