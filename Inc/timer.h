#ifndef TIMER_H
#define TIMER_H

//extern RTC_HandleTypeDef hrtc;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim14;

void print_timer_status();

#define FREQUENCY_MHZ 48
// 1/(48MHz*21) = ~0.99ppb
#define MAX_CORRECTIONS 21

#endif
