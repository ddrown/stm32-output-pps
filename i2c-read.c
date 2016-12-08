#include <errno.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include <stdio.h>

#define I2C_ADDR 0x4

void write_i2c(int fd, void *buffer, size_t len) {
  ssize_t status;

  status = write(fd, buffer, len);
  if(status < 0) {
    perror("write to i2c failed");
    exit(1);
  }
  if(status != len) {
    printf("write not %u bytes: %d\n", len, status);
    exit(1);
  }
}

void read_i2c(int fd, void *buffer, size_t len) {
  ssize_t status;

  status = read(fd, buffer, len);
  if(status < 0) {
    perror("read to i2c failed");
    exit(1);
  }
  if(status != len) {
    printf("read not %u bytes: %d\n", len, status);
    exit(1);
  }
}

#define REGISTER_CORRECTION 14
#define REGISTER_APPLY_CHANGE 16
#define APPLY_CHANGE 1

int main() {
  int fd;
  ssize_t len;
  uint8_t writebuf[4]; 
  struct i2c_registers_type {
    uint32_t milliseconds;
    uint32_t TIM_interrupt_latency;
    int16_t adc_temp;              // in 1/10 C units
    uint16_t adc_voltage;          // in 1/100 V units
    uint16_t frequency_MHz;
    int16_t freq_correction;       // in ppb units
    uint8_t apply_change;         // set to non-zero to apply freq_correction
  } i2c_registers;
  
  fd = open("/dev/i2c-1", O_RDWR);
  if (fd < 0) {
    perror("open /dev/i2c-1 failed");
    exit(1);
  }

  if (ioctl(fd, I2C_SLAVE, I2C_ADDR) < 0) {
    perror("ioctl i2c slave addr failed");
    exit(1);
  }

/*
 * This sets the 1PPS to +1000 ppb correction
  writebuf[0] = REGISTER_CORRECTION;
  writebuf[1] = (1000) & 0xFF;
  writebuf[2] = (1000 >> 8) & 0xFF;
  writebuf[3] = APPLY_CHANGE;
  write_i2c(fd, writebuf, 4);
*/

  while(1) {
    read_i2c(fd, &i2c_registers, 17);

    printf("milli: %u\n", i2c_registers.milliseconds);
    printf("intlat: %u\n", i2c_registers.TIM_interrupt_latency);
    printf("temp: %.1f\n", i2c_registers.adc_temp/10.0);
    printf("Vref: %.2f\n", i2c_registers.adc_voltage/100.0);
    printf("freq: %u\n", i2c_registers.frequency_MHz);
    printf("corr: %d\n", i2c_registers.freq_correction);
    printf("apply: %u\n", i2c_registers.apply_change);
    sleep(1);
  }
}
