#include "stm32f0xx_hal.h"

#include "timer.h"
#include "adc.h"
#include "uart.h"

// in hundreds of picoseconds, range +/-214ms
static int32_t offset_ps = 0;
static int32_t static_ppt = 0;
// 48MHz, in hundreds of picoseconds
#define PS_PER_COUNT 208
// 1ms in 48MHz
#define DEFAULT_PRESCALER 47999
// limit movement to about +/-250 ppm (beware 16 bit overflow with DEFAULT_PRESCALER+MAX_ADJUST)
#define MAX_ADJUST 15000

static uint16_t lastadjust = DEFAULT_PRESCALER;
static enum {PRESCALE_NORMAL, PRESCALE_PENDING, PRESCALE_ACTIVE} pending_prescale = PRESCALE_NORMAL;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if(pending_prescale == PRESCALE_PENDING) {
    htim3.Instance->ARR = lastadjust;
    pending_prescale = PRESCALE_ACTIVE;
  } else if(pending_prescale == PRESCALE_ACTIVE) {
    htim3.Instance->ARR = DEFAULT_PRESCALER;
    pending_prescale = PRESCALE_NORMAL;
  }
}

void adjust_pps() {
  int32_t offset_count;

  // hundreds of picoseconds
  offset_ps -= (timer_ppt() + static_ppt) / 100;
  offset_count = offset_ps / PS_PER_COUNT;

  if(offset_count > MAX_ADJUST)
    offset_count = MAX_ADJUST;
  if(offset_count < -1*MAX_ADJUST)
    offset_count = -1*MAX_ADJUST;

  // remove the adjusted part, leave the rest
  offset_ps -= offset_count * PS_PER_COUNT;
  lastadjust = DEFAULT_PRESCALER - offset_count;
  pending_prescale = PRESCALE_PENDING;
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
// g(x) = TEMP_ADJUST_A + TEMP_ADJUST_B * (x + TEMP_ADJUST_C) + ((x + TEMP_ADJUST_C)**2) / TEMP_ADJUST_D

// 1.234535 * 1,000,000
#define TEMP_ADJUST_A 1234535
// 0.02229774 * 1,000,000
#define TEMP_ADJUST_B 22298
// 1.215454 * 1,000,000
#define TEMP_ADJUST_C 1215454
// 1/0.0002962121 * -1,000,000
#define TEMP_ADJUST_D -3375959321

// limits: +/- 2,147 ppm
int32_t timer_ppt() {
  int32_t temp_uF = last_temp() * 1000;
  int64_t ppt_a64 = TEMP_ADJUST_B * (int64_t)(temp_uF + TEMP_ADJUST_C) / 1000000;
  int64_t ppt_b64 = ((int64_t)(temp_uF + TEMP_ADJUST_C))*(temp_uF + TEMP_ADJUST_C) / TEMP_ADJUST_D;
  int64_t ppt = TEMP_ADJUST_A + ppt_a64 + ppt_b64;

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
