#include <stdio.h>
#include <stdint.h>

int main() {
  for(uint32_t temp_uF = 72000000; temp_uF < 103000000; temp_uF += 100000) {
    uint64_t ppt_a64 = 22298 * (uint64_t)(temp_uF + 1215454) / 1000000;
    int64_t ppt_b64 = ((int64_t)(temp_uF + 1215454))*(temp_uF + 1215454) / -3375959321;
    uint64_t ppt_64 = 1234535 + ppt_a64 + ppt_b64;
    uint32_t ppm_32 = (ppt_64 + 500) / 1000;

    printf("%.1f %u\n",temp_uF/1000000.0,ppm_32);
  }
}
