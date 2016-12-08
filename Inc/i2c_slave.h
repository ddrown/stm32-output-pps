#ifndef I2C_SLAVE
#define I2C_SLAVE

extern I2C_HandleTypeDef hi2c1;

void i2c_slave_start();
void i2c_show_data();

extern struct i2c_registers_type {
  uint32_t milliseconds;
  uint32_t TIM_interrupt_latency;
  int16_t adc_temp;              // in 1/10 C units
  uint16_t adc_voltage;          // in 1/100 V units
  uint16_t frequency_MHz;
  int16_t freq_correction;       // in ppb units
  uint8_t apply_change;         // set to non-zero to apply freq_correction
} i2c_registers;
// total size: 17

#endif
