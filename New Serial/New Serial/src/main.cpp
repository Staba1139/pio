#include <mbed.h>
#include <cstdlib>

Serial pc(USBTX, USBRX, 9600);

int main() {

  char data[32];
  int count = 0;
  int number = 0;

  while(1) {
    if(pc.readable() > 0) {
      data[count] = pc.getc();
      if(count > 30 || data[count] == '=') {
        data[count] = '\0';
        count = 0;
        number = atoi(data);
        pc.printf("%s\n", data);
        pc.printf("Number = %d\n", number);
      }
      else {
        count++;
      }
    }
  }
}
