#include "stm32f0xx_hal.h"

#include <stdlib.h>
#include <math.h>

#include "timer.h"
#include "uart.h"
#include "i2c_slave.h"

static uint32_t next_correction = 0;

// start with 0 correction
static uint16_t corrections[MAX_CORRECTIONS] = {FREQUENCY_MHZ*1000-1, 0};

void recalculate_corrections() {
  // this can be 1 cycle wrong due to rounding, but floating point math doesn't fit in the flash
  int32_t cycles = (FREQUENCY_MHZ * MAX_CORRECTIONS * i2c_registers.freq_correction) / 1000;

  int32_t corrected_cycles = 0;
  for(uint8_t i = 1; i <= MAX_CORRECTIONS; i++) {
    int16_t this_cycle = cycles*i/MAX_CORRECTIONS - corrected_cycles;
    corrections[i-1] = 47999-this_cycle;
    corrected_cycles += this_cycle;
  }
}

// TIMER1 overflow interrupt
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  i2c_registers.milliseconds++;

  // adjust on 1st millisecond
  if(__HAL_TIM_GET_COUNTER(&htim3) == 1) {
    // TIM1 is acting as the prescaler, change its reload value
    htim->Instance->ARR = corrections[next_correction];

    i2c_registers.TIM_interrupt_latency = __HAL_TIM_GET_COUNTER(htim);

    // move to the next correction
    next_correction++;
    // if that's the end, loop back to the first
    if(corrections[next_correction] == 0 || next_correction >= MAX_CORRECTIONS) {
      next_correction = 0;
      if(i2c_registers.apply_change) {
        recalculate_corrections();
        i2c_registers.apply_change = 0;
      }
    }
  } else if(__HAL_TIM_GET_COUNTER(&htim3) == 2) { // normal speed on 2nd and later ms
    htim->Instance->ARR = FREQUENCY_MHZ*1000-1;
  }
}

void print_timer_status() {
  write_uart_s("current correction ");
  write_uart_u(corrections[next_correction]);
  write_uart_s(" ");
  write_uart_u(next_correction);
  write_uart_s(" now ");
  write_uart_u(__HAL_TIM_GET_COUNTER(&htim3));
  write_uart_s(" pend=");
  write_uart_u(i2c_registers.apply_change);
  write_uart_s("\n");
}
