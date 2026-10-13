#pragma once
#include "../common.h"

/* 模拟内核 sysfs 字符串比较：允许输入末尾有一个换行。 */
static inline bool sysfs_streq(const char *a, const char *b) {
  while (*a != '\0' && *a == *b) {
    a++;
    b++;
  }
  return (*a == '\0' && *b == '\0') ||
         (*a == '\n' && a[1] == '\0' && *b == '\0') ||
         (*b == '\n' && b[1] == '\0' && *a == '\0');
}
