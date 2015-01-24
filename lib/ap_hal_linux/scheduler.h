#ifndef SCHEDULER_H
#define SCHEDULER_H

int scheduler_init();
int scheduler_begin();
int get_ms(unsigned long *count);
int get_us(unsigned long *count);
void delay_microseconds(unsigned long usec);
void delay_ms(unsigned long num_ms);

#endif
