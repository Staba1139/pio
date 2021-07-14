#include <mbed.h>
#include <cstdlib>

BufferedSerial pc(USBTX, USBRX, 9600);

int main() {

  char data[32];
  int count = 0;
  int number = 0;
  char msg[] = "Number = ";


  while(1) {
    if(pc.readable() > 0){
      pc.read(data, sizeof(data));
      if(count > 30 || data[count] == '=') {
        data[count] = '\0';
        number = atoi(data);
        pc.write(data, sizeof(data));
        pc.write(msg, sizeof(msg));
        pc.write(number, sizeof(number));
      }
      else {
        count++;
      }
    }
  }
}
