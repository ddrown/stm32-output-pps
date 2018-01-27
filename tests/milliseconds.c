#include <stdio.h>
#include <stdint.h>
#include <sys/time.h>
#include <unistd.h>

uint32_t mstime() {
  struct timeval tv;
  gettimeofday(&tv, NULL);
  return tv.tv_usec/1000;
}

int main() {
  while (1) {
    uint16_t next_adc = 50; // start at 50ms
    while(1) {
      while(mstime() < next_adc) {
        usleep(1000);
      }
      printf("%u %u\n", mstime(), next_adc);
      next_adc += 100; // every 100ms
      if(next_adc > 1000) {
        break;
      }
    }
    // happens after 950ms
    printf("U\n");
    while(mstime() >= 950) {
      usleep(1000);
    }
  }
}
