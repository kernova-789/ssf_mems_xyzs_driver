#pragma once
#include "common.h"
#define min(a, b) ((a) < (b) ? (a) : (b))
#define max(a, b) ((a) > (b) ? (a) : (b))
#define DIV_ROUND_UP(n, d) (((n) + (d) - 1) / (d))
#define dev_warn_ratelimited(dev, ...) dev_warn(dev, __VA_ARGS__)
#define dev_name(dev) ((void)(dev), "mock-sensor")
