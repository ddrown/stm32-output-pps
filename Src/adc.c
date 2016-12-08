#include "stm32f0xx_hal.h"

#include "adc.h"
#include "uart.h"
#include "i2c_slave.h"

// example value: 1758
#define TEMP30_CAL_PTR ((uint16_t *)0x1FFFF7B8)
// example value: 1326
#define TEMP110_CAL_PTR ((uint16_t *)0x1FFFF7C2)
// example value: 1525
#define VREFINT_CAL_PTR ((uint16_t *)0x1FFFF7BA)

void start_adc() {
  write_uart_s("adc: ");
  write_uart_i(i2c_registers.adc_temp);
  write_uart_s(" ");
  write_uart_u(i2c_registers.adc_voltage);
  write_uart_s("\n");
  HAL_ADC_Start_IT(&hadc);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
  if( __HAL_ADC_GET_FLAG(hadc, ADC_FLAG_EOS)) { 
    int32_t voltage = HAL_ADC_GetValue(hadc);

    // work in 0.01V
    voltage = voltage * 330 / 4095;

    i2c_registers.adc_voltage = voltage;
  } else {
    int32_t temperature = HAL_ADC_GetValue(hadc);

    // assumption: 3.3V vref
    // convert to 30C center so the slope conversion works
    temperature = temperature - *TEMP30_CAL_PTR;
    // work in 0.1degC
    temperature *= 10;
    // adjust for linear-ish slope between 30C and 110C measurements
    temperature *= (110-30); 
    temperature /= ((int32_t)*TEMP110_CAL_PTR - (int32_t)*TEMP30_CAL_PTR);
    // adjust for lower measurement being 30.0C
    temperature += 300;

    i2c_registers.adc_temp = temperature;
  }
}
