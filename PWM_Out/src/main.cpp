#include <mbed.h>
#include "Pulse.h"

#define PULSE_READ p6
#define LAZER p7

//defining motor parameter
const int throttle_low = 0.001;

//defining photo diode sensing
int lazer_val;
double rpm;

//defining measurement frequency
unsigned long l_time = 0;
unsigned long h_time = 0;
unsigned long t_time = 0;

int input = -1;
int o_input = 0;
int n_input = 0;

PulseInOut lpulse(PULSE_READ);
DigitalOut lazer(LAZER);

PwmOut s1(p26);
PwmOut s2(p25);
PwmOut s3(p24);
PwmOut s4(p23);
PwmOut s5(p22);
PwmOut s6(p21);
 

double level;

double convmsec2sec(double x);

Serial PC(USBTX, USBRX);

int main()
{
    double pwmval;
    
    pwmval = convmsec2sec(4.0);
    
    s1.period(pwmval);
    s2.period(pwmval);
    s3.period(pwmval);
    s4.period(pwmval);
    s5.period(pwmval);
    s6.period(pwmval);
    
    s1.pulsewidth(throttle_low);
    s2.pulsewidth(throttle_low);
    s3.pulsewidth(throttle_low);
    s4.pulsewidth(throttle_low);
    s5.pulsewidth(throttle_low);
    s6.pulsewidth(throttle_low);
    
    wait_us(8000000);
    while(1) {
        
        input = PC.getc();
        if(input != -1){
            if(input != 10){
                o_input = o_input*10 + (input - 48);
            }
            else{
                n_input = o_input;
                o_input = 0;
                s1.pulsewidth(n_input/1000000);
                PC.printf("---");
                PC.printf("%d", n_input);
                PC.printf("---\n");
            }
        }
        
        //Lazer output
        lazer = 1;
        
        //Read Sensor Pulse Time
        h_time = (unsigned long)lpulse.read_high_us();
        l_time = (unsigned long)lpulse.read_low_us();
        t_time = h_time + l_time;
        rpm = 20.0 * 1000000 / (double)t_time;
        PC.printf("%.3lf\n", rpm);
        wait_us(100000);       
    }
}


double convmsec2sec(double x){
    
    return x * 0.001;

}
