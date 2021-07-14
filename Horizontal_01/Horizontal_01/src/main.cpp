#include <mbed.h>


const float throttleLow = 0.001;

PwmOut s1(p26);
PwmOut s2(p25);
PwmOut s3(p24);
PwmOut s4(p23);
PwmOut s5(p22);
PwmOut s6(p21);

double level;

double convmsec2sec(double x);

int main() {

  double pwmval;

  pwmval = convmsec2sec(4.0);

  s1.period(pwmval);
  s2.period(pwmval);
  s3.period(pwmval);
  s4.period(pwmval);
  s5.period(pwmval);
  s6.period(pwmval);

  s1.pulsewidth(throttleLow);
  s2.pulsewidth(throttleLow);
  s3.pulsewidth(throttleLow);
  s4.pulsewidth(throttleLow);
  s5.pulsewidth(throttleLow);
  s6.pulsewidth(throttleLow);

  wait_us(8000000);

  while(1) {

        s1.pulsewidth(0.0012);
        s2.pulsewidth(0.0012);
  }
}

double convmsec2sec (double x) {
  return x * 0.001;
}
