#ifndef SCHEDULER_H
#define SCHEDULER_H
#include <stdint.h>

#define delay_ms	linux_delay_ms
#define get_ms		linux_get_ms
#define get_us 		linux_get_us
//extern void timer_update(union sigval v);
int scheduler_init();
int scheduler_begin();
int linux_get_ms(uint32_t *count);
int linux_get_us(uint32_t *count);
void delay_microseconds(uint32_t usec);
void linux_delay_ms(uint32_t num_ms);

#endif
