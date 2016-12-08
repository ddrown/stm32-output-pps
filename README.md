Depends on make and arm-none-eabi-gcc being in your path.

arm-none-eabi-gcc should come with newlib (or other embedded c library)

"make" - builds the binary, output is in build/output-pps.elf + build/output-pps.bin

"make flash" - build the binary and flash it with a ST-Link (depends on ST-LINK\_CLI being in your path)

Use STM32CubeMX to view the pinout

 * Src/adc.c - ADC (for core temperature)
 * Src/i2c\_slave.c - i2c slave
 * Src/timer.c - hardware timers driving 1PPS (TIM3 = drives PPS output, TIM1 = overflows@1khz and drives TIM3), TIM14 is configured for 1PPS but not used
 * Src/uart.c - uart print and receive
 * Src/main.c - setup and main loop
 * Src/stm32f0xx\_hal\_msp.c - auto-generated GPIO mapping code
 * Src/stm32f0xx\_it.c - auto-generated interrupt handlers
