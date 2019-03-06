#include <SoftwareSerial.h>
#include "wireless.h"

SoftwareSerial xx(SS_RX, SS_TX);

void wireless_init()
{
  xx.begin(9600);  
}

void TransferStr(char *str)
{
  xx.println(str);  
}

void transferData(float data[], int num)
{
  int i;
  
  for(i = 0; i < num; i++) {
      xx.print(data[i]);
      xx.print(",");
      if(i>=3) break;
  }
if(num>4){
   xx.print("motor_control(");
   xx.print(data[4]);
   xx.print("  ");
   xx.print(data[5]);
   xx.print(")");
}
 
  
  xx.print(millis());
  xx.println();
}

int wirelessAvailable(void)
{
  return(xx.available());
}

char receiveData(void)
{  
  xx.read();  
}

