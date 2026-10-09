#include "../common.h"
void test_msleep(unsigned int ms);
#define msleep(ms) test_msleep(ms)
