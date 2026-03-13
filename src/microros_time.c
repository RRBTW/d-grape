#include <time.h>
#include <stm32f4xx_hal.h>
int clock_gettime(clockid_t id, struct timespec *tp) { uint32_t ms=HAL_GetTick(); tp->tv_sec=ms/1000; tp->tv_nsec=(ms%1000)*1000000; return 0; }
