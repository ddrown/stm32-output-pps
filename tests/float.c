#include <stdio.h>
#include <math.h>

int main() {
  for(float temp_F = 72.0; temp_F < 103.0; temp_F += 0.1) {
    float ppm_a = 0.02229774 * (temp_F + 1.215454);
    float ppm_b = -0.0002962121 * pow(temp_F + 1.215454, 2);
    float ppm = 1.234535 + ppm_a + ppm_b;

    printf("%.1f %.3f\n",temp_F,ppm);
  }
}
