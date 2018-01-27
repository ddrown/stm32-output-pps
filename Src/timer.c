#include "stm32f0xx_hal.h"

#include "timer.h"
#include "adc.h"
#include "uart.h"

// in hundreds of picoseconds, range +/-214ms
static int32_t offset_ps = 0;
static int32_t static_ppt = 0;
// 1 cycle in 48MHz in hundreds of picoseconds
#define PS_PER_COUNT 208
// 1ms in 48MHz
#define DEFAULT_PRESCALER 47999
// limit movement to about +/-250 ppm (beware 16 bit overflow with DEFAULT_PRESCALER+MAX_ADJUST)
#define MAX_ADJUST 15000

static uint16_t lastadjust = DEFAULT_PRESCALER;
static uint8_t pending_prescale = 0;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if(pending_prescale == 1) {
    htim3.Instance->ARR = lastadjust;
    pending_prescale = 2;
  } else if(pending_prescale == 2) {
    htim3.Instance->ARR = DEFAULT_PRESCALER;
    pending_prescale = 0;
  }
}

void adjust_pps() {
  int32_t offset_count;

  offset_ps -= (timer_ppt() + static_ppt) / 100;
  offset_count = offset_ps / PS_PER_COUNT;

  if(offset_count > MAX_ADJUST)
    offset_count = MAX_ADJUST;
  if(offset_count < -1*MAX_ADJUST)
    offset_count = -1*MAX_ADJUST;

  // remove the adjusted part, leave the rest
  offset_ps -= offset_count * PS_PER_COUNT;
  lastadjust = DEFAULT_PRESCALER - offset_count;
  pending_prescale = 1;
}

// caller should limit offset to prevent wraps
void add_offset(int32_t offset) {
  offset_ps += offset;
}

void set_frequency(int32_t frequency) {
  static_ppt = frequency;
}

void timer_start() {
  HAL_TIM_Base_Start_IT(&htim3);
  HAL_TIM_Base_Start(&htim1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
}

// these values are from a specific TCXO, you'll need to measure your own hardware for best results.
// fixed-point math is used to save flash space, Cortex-M0 uses software floating point
// most terms are multipled by 1e6
// g(x) = 1.234535 + 0.02229774 * (x + 1.215454) - 0.0002962121 * (x + 1.215454)**2
//
#define TCXO_A 1234535
#define TCXO_B 22298
#define TCXO_C 1215454
// 1000000/-0.0002962121 = -3375959321
#define TCXO_D -3375959321

// +/- 2,000 ppm
int32_t timer_ppt() {
  int32_t temp_uF = last_temp() * 1000;
  int64_t ppt_a64 = TCXO_B * (int64_t)(temp_uF + TCXO_C) / 1000000;
  int64_t ppt_b64 = ((int64_t)(temp_uF + TCXO_C))*(temp_uF + TCXO_C) / TCXO_D;
  int64_t ppt = TCXO_A + ppt_a64 + ppt_b64;

  return ppt;
}

void print_timer_ppb() {
  write_uart_s("compensation ");
  // round up
  write_uart_i((timer_ppt() + 500)/1000);
  write_uart_s(" ppb, configured ");
  write_uart_i(static_ppt/1000);
  write_uart_s(" ppb\n");
}

void print_timer_offset() {
  write_uart_s("offset ");
  write_uart_i((offset_ps + 5)/10);
  write_uart_s(" ns\n");
  write_uart_s("last adjust ");
  write_uart_i(lastadjust);
  write_uart_s("\n");
}
