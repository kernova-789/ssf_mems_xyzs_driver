#include "../common.h"
extern unsigned long jiffies;
#define time_after(a, b) ((long)((b) - (a)) < 0)
#define time_before(a, b) time_after(b, a)
#define time_after_eq(a, b) ((long)((a) - (b)) >= 0)
#define jiffies_to_msecs(ticks) (ticks)
