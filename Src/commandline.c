#include "stm32f0xx_hal.h"
#include "commandline.h"
#include "uart.h"
#include "adc.h"
#include "timer.h"

#include <string.h>
#include <stdlib.h>

static void print_help() {
  write_uart_s(
  "commands:\n"
  "temp - temp sensor\n"
  "vcc - voltage sensor\n"
  "ppb - print timer freq\n"
  "offset - print timer offset\n"
  "o [ns] - add offset\n"
  "f [ppb] - set frequency\n"
  "help - print help\n"
  );
}

static void add_offset_cmdline(const char *cmdline) {
  int32_t hps = atoi(cmdline);
  write_uart_s("adding ");
  write_uart_i(hps);
  write_uart_s(" ns\n");
  hps = hps * 10;
  add_offset(hps);
  print_timer_offset();
}

static void set_frequency_cmdline(const char *cmdline) {
  int32_t ppt = atoi(cmdline);
  ppt = ppt * 1000;
  set_frequency(ppt);
  print_timer_ppb();
}

static void run_command(char *cmdline) {
  if(strcmp("temp", cmdline) == 0) {
    print_last_temp();
  } else if(strcmp("vcc", cmdline) == 0) {
    print_last_vcc();
  } else if(strcmp("ppb", cmdline) == 0) {
    print_timer_ppb();
  } else if(strcmp("offset", cmdline) == 0) {
    print_timer_offset();
  } else if(strncmp("o ", cmdline, 2) == 0) {
    add_offset_cmdline(cmdline+2);
  } else if(strncmp("f ", cmdline, 2) == 0) {
    set_frequency_cmdline(cmdline+2);
  } else {
    print_help();
  }
}

void cmdline_prompt() {
  write_uart_s("> ");
}

static char cmdline[40];
static uint32_t cmd_len = 0;

void reprint_prompt() {
  cmdline_prompt();
  if(cmd_len > 0) {
    write_uart_s(cmdline);
  }
}

void cmdline_addchr(char c) {
  switch(c) {
    case '\r':
    case '\n':
      write_uart_s("\n");
      run_command(cmdline);
      cmdline_prompt();
      cmdline[0] = '\0';
      cmd_len = 0;
      break;
    case '\b':
    case '\x7F':
      if(cmd_len > 0) {
        write_uart_s("\x7F");
        cmd_len--;
        cmdline[cmd_len] = '\0';
      }
      break;
    default:
      if(cmd_len < sizeof(cmdline)-1) {
        write_uart_ch(c);
        cmdline[cmd_len] = c;
        cmdline[cmd_len+1] = '\0';
        cmd_len = cmd_len + 1;
      }
  }
}
