#pragma once
#include "common.h"
#define IS_ERR(ptr) ((uintptr_t)(ptr) >= (uintptr_t)-4095)
#define PTR_ERR(ptr) ((long)(ptr))
