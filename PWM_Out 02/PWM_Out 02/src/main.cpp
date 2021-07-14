#include <mbed.h>

#define LAZER p7

const int throttleLow = 0.001;

int input = -1;
int o_input = 0;
int n_input = 0;

DigitalOut lazer(LAZER);

PwmOut s1(p26);
PwmOut s2(p25);
PwmOut s3(p24);
PwmOut s4(p23);
PwmOut s5(p22);
PwmOut s6(p21);

double level;

double convmsec2sec(double x);

Serial PC(USBTX, USBRX, 9600);

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
    input = PC.getc();
    /*
    if(input != -1){
      if(input != 10){
        o_input = o_input*10 + (input - 48);
      }
      else{ */
        n_input = o_input;
        o_input = 0;
        s1.pulsewidth(n_input/1000000);
        PC.printf("---");
        PC.printf("%d", input);
        PC.printf("---\n");
      //}
    //}

    //Lazer Output
    lazer = 1;
  }
}

double convmsec2sec (double x) {
  return x * 0.001;
}
