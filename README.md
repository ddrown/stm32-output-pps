Hardware is a STM32F030F4P6 with 12MHz TCXO, communication is via uart.  Clocks are setup for 12MHz HSE (bypass not crystal) and 48MHz PLL

Depends on make and arm-none-eabi-gcc being in your path.  Recommended using cygwin on Windows or Linux.

arm-none-eabi-gcc should come with newlib (or other embedded c library)

"make" - builds the binary, output is in build/output-pps-uart.elf + build/output-pps-uart.bin

"make flash" - build the binary and flash it with a ST-Link (depends on ST-LINK\_CLI being in your path)

Use STM32CubeMX to view the pinout

 * Src/adc.c - ADC (for core temperature, measured at 10Hz)
 * Src/timer.c - hardware timers driving 1PPS (TIM1 = drives PPS output, TIM3 = overflows at 1khz and drives TIM1)
 * Src/uart.c - uart print and receive
 * Src/main.c - setup and main loop
 * Src/commandline.c - uart commandline
 * Src/stm32f0xx\_hal\_msp.c - auto-generated GPIO mapping code
 * Src/stm32f0xx\_it.c - auto-generated interrupt handlers
 * Src/system\_stm32f0xx.c - auto-generated system initialization

The function timer\_ppt in Src/timer.c converts temperature sensor reading into an expected TCXO frequency error.  Measure your TCXO and use your own TCXO\_\* values for best results.

Example uart session (comments marked with #):

```
> f -90
compensation 806 ppb, configured -90 ppb

# clock drifted 1861ns fast in 131 seconds, 14ppb

> f -76
compensation 798 ppb, configured -76 ppb
> o -790
adding -790 ns
offset -799 ns
last adjust 48034

# clock drifted 1125ns slow in 303 seconds, 3ppb

> f -79
compensation 806 ppb, configured -79 ppb
> o 1361
adding 1361 ns
offset 1353 ns
last adjust 48034

```

After this first adjustment, the offset between this PPS and a GPS module's PPS:

![Offset](images/run-tcxo.png?raw=true)

Allan deviation of the above offset

![adev](images/adev-tcxo.png?raw=true)
