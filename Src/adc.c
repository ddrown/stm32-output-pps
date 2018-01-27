#include "stm32f0xx_hal.h"
#include <stdint.h>
#include "adc.h"
#include "uart.h"

// 3.3V per count of 4096 in nV
#define NV_PER_COUNT 805664

#define AVERAGE_SAMPLES_ADC 10

static uint16_t internal_temps[AVERAGE_SAMPLES_ADC];
static uint16_t internal_vrefs[AVERAGE_SAMPLES_ADC];
static int8_t adc_index = -1;

#define AVERAGE_SAMPLES_CALC 60

static int32_t temps[AVERAGE_SAMPLES_CALC];
static uint64_t vccs[AVERAGE_SAMPLES_CALC];
static int8_t calc_index = -1;

// addresses from the STM32F030 datasheet
static uint16_t *ts_cal1 = (uint16_t *)0x1ffff7b8;
static uint16_t *ts_cal2 = (uint16_t *)0x1ffff7c2;
static uint16_t *vrefint_cal = (uint16_t *)0x1ffff7ba;

static uint16_t avg_16(uint16_t *values, uint8_t index) {
  uint32_t sum = 0;
  for(uint8_t i = 0; i <= index; i++) {
    sum += values[i];
  }
  return sum / (index+1);
}

static uint64_t avg_64(uint64_t *values, uint8_t index) {
  uint64_t sum = 0;
  for(uint8_t i = 0; i <= index; i++) {
    sum += values[i];
  }
  return sum / (index + 1);
}

static int32_t avg_i(int32_t *values, uint8_t index) {
  int64_t sum = 0;
  for(uint8_t i = 0; i <= index; i++) {
    sum += values[i];
  }
  return sum / (index + 1);
}


void adc_poll() {
  if(adc_index < (AVERAGE_SAMPLES_ADC-1)) {
    adc_index++;
  } else {
    for(uint8_t i = 0; i < (AVERAGE_SAMPLES_ADC-1); i++) {
      internal_temps[i] = internal_temps[i+1];
      internal_vrefs[i] = internal_vrefs[i+1];
    }
  }

  HAL_ADC_Start(&hadc);
  if(HAL_ADC_PollForConversion(&hadc, 10) == HAL_OK) {
    internal_temps[adc_index] = HAL_ADC_GetValue(&hadc);
  }
  if(HAL_ADC_PollForConversion(&hadc, 10) == HAL_OK) {
    internal_vrefs[adc_index] = HAL_ADC_GetValue(&hadc);
  }
  HAL_ADC_Stop(&hadc);
}

static int32_t last_temp_mF = 0;
static uint64_t last_vcc_nv = 0;

void print_last_temp() {
  write_uart_s("temp ");
  write_uart_u(last_temp_mF);
  write_uart_s(" mF\n");
}

int32_t last_temp() {
  return last_temp_mF;
}

void print_last_vcc() {
  write_uart_s("vcc ");
  write_uart_u(last_vcc_nv/1000);
  write_uart_s(" uV\n");
}

void update_adc() {
  uint16_t temp = avg_16(internal_temps, adc_index);
  uint16_t vref = avg_16(internal_vrefs, adc_index);

  if(calc_index < (AVERAGE_SAMPLES_CALC-1)) {
    calc_index++;
  } else {
    for(uint8_t i = 0; i < (AVERAGE_SAMPLES_CALC-1); i++) {
      temps[i] = temps[i+1];
      vccs[i] = vccs[i+1];
    }
  }

  uint32_t vref_expected_nv = (*vrefint_cal) * NV_PER_COUNT;
  uint32_t vref_actual_nv = vref*NV_PER_COUNT;
  uint64_t vcc_nv = (uint64_t)((vref_expected_nv+500)/1000)*3300000000/((vref_actual_nv+500)/1000);

  vccs[calc_index] = vcc_nv;
  last_vcc_nv = avg_64(vccs, calc_index);

  uint32_t nv_per_count = last_vcc_nv / 4096;

  uint32_t temp_nv = temp * nv_per_count;
  uint32_t nv_30C = (*ts_cal1) * NV_PER_COUNT;
  uint32_t nv_110C = (*ts_cal2) * NV_PER_COUNT;

  int32_t uv_per_C = (((int32_t)nv_110C - (int32_t)nv_30C) / (110-30) + 500)/1000;
  int32_t temp_mC = ((int32_t)temp_nv - (int32_t)nv_30C) / uv_per_C + 30000;
  int32_t temp_mF = temp_mC * 9/5+32000;
  temps[calc_index] = temp_mF;
  last_temp_mF = avg_i(temps, calc_index);
}
