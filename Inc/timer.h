#ifndef TIMER_H
#define TIMER_H

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;

void timer_start();
void print_timer_ppb();
void print_timer_offset();
int32_t timer_ppt();
void set_frequency(int32_t frequency);
void add_offset(int32_t offset);
void adjust_pps();

#endif
