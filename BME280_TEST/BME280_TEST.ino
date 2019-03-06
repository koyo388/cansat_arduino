#include <cactus_io_BME280_SPI.h>
#include <SPI.h>
#include <SoftwareSerial.h>
#include "wireless.h"

#define BME_SCK 13      // Serial Clock
#define BME_MISO 12     // Serial Data Out
#define BME_MOSI 11     // Serial Data In
#define BME_CS A2       // Chip Select
 
float HI = 12.2 ;//現在地の高度 調べる
float Pos = 1025.22 ;//海面気圧　センサーの値から算出　

//#define MODE_TEST   //センサー動作テスト
//#define MODE_SEA_LEVEL  //海面気圧
#define MODE_HIGHT  //高度
//#define MODE_PRESSURE //気圧センサーのみ

BME280_SPI bme(BME_CS,BME_MOSI,BME_MISO,BME_SCK); 

//海面気圧
float sea_press(float h,float T,float P){
  float Po = P * pow((1 - ((0.0065 * h/(0.0065 * h + T + 273.15)))),-5.257) ;
  return Po ;
}

//高度
float hight(float Po,float P,float T){
  float a = pow((Po/P),(1/5.257));
  float h = ((a-1)*(T + 273.15))/0.0065 ;
  return h ;
}

double calcAltitude(float pressure,float temperature) 
 { 
   // Equation taken from BMP180 datasheet (page 16): 
   //  http://www.adafruit.com/datasheets/BST-BMP180-DS000-09.pdf 
 
 
   // Note that using the equation from wikipedia can give bad results 
   // at high altitude.  See this thread for more information: 
   //  http://forums.adafruit.com/viewtopic.php?f=22&t=58064 
   /*double altitude = (temperature + 273.15) * (pow(MEAN_SEA_LEVEL_PRESSURE/pressure, 0.190294957)-1.0) / 0.0065;*/ 
   double altitude = 44330.0 * (1.0 - pow(pressure / Pos, 0.190294957)); 
   return altitude; 
 } 

 void ReceiveStr(char *str) {
  int i;
  char ch;

  for (i = 0; ch != '!';) {
    if (wirelessAvailable()) {
      ch = receiveData();
      str[i] = ch;
      i++;
    }
  }
  str[i - 1] = '\0';
}  //?

void SPI_init()  //通信
{
  digitalWrite(10, HIGH);
  pinMode(10, OUTPUT);
  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE3);
  SPI.setClockDivider(SPI_CLOCK_DIV8); // 16MHz/8 = 2MHz; (max 10MHz)
}



#ifdef MODE_TEST
void setup() {
  // put your setup code here, to run once:
   Serial.begin(9600);
  Serial.println("Bosch BME280 Barometric Pressure - Humidity - Temp Sensor | cactus.io"); 
  if (!bme.begin()) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }
  digitalWrite(10, HIGH);
  pinMode(10, OUTPUT);
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE3);
  SPI.setClockDivider(SPI_CLOCK_DIV8); // 16MHz/8 = 2MHz; (max 10MHz)
  
  bme.setTempCal(-1);

   char str[100];
  Serial.println("check"); //シリアルモニターに送る
  TransferStr("Ready...");
    ReceiveStr(str);
  Serial.println("Pressure\tHumdity\t\tTemp\t\tTemp");
  TransferStr("Start!"); //始まる
  unsigned long time = millis(); //時間を測り始める

}

void loop() {
  // put your main code here, to run repeatedly:
   bme.readSensor();
    float pressure = bme.getPressure_MB() ;
    float temperature = bme.getTemperature_C() ;
    float data[3] ;
  if(pressure <= 1100 && pressure >= 900) {
    Serial.print(bme.getPressure_MB()); Serial.print("\t\t");    // Pressure in millibars
    Serial.print(bme.getHumidity()); Serial.print("\t\t");
    Serial.print(bme.getTemperature_C()); Serial.print(" *C\t");
    Serial.print(bme.getTemperature_F()); Serial.println(" *F\t");

    // add a 2 second delay to slow down the output
    data[0] = pressure ;
    data[1] = bme.getHumidity() ;
    data[2] = temperature ;
    transferData(data, 3);
    delay(1000);
  }

}
#endif

#ifdef MODE_SEA_LEVEL
void setup() {
  // put your setup code here, to run once:
   Serial.begin(9600);
  Serial.println("Bosch BME280 Barometric Pressure - Humidity - Temp Sensor | cactus.io"); 

  if (!bme.begin()) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }

  bme.setTempCal(-1);
  
  Serial.println("Pressure\tHumdity\t\tTemp\t\tTemp");

}

void loop() {
  // put your main code here, to run repeatedly:
   bme.readSensor();
    float pressure = bme.getPressure_MB() ;
    float temperature = bme.getTemperature_C() ;
    Serial.println (sea_press(HI,temperature,pressure)) ;
    delay(2000);
}
#endif

#ifdef MODE_HIGHT
void setup() {
  // put your setup code here, to run once:
   Serial.begin(9600);
  Serial.println("Bosch BME280 Barometric Pressure - Humidity - Temp Sensor | cactus.io"); 

  if (!bme.begin()) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }

  bme.setTempCal(-1);
  
  Serial.println("Pressure\tHumdity\t\tTemp\t\tTemp");

}

void loop() {
  // put your main code here, to run repeatedly:
   bme.readSensor();
    float pressure = bme.getPressure_MB() ;
    float temperature = bme.getTemperature_C() ;
    float altitude = hight(Pos,pressure,temperature) - HI ;
    Serial.println(altitude) ;
    delay(2000) ;
}
#endif

#ifdef MODE_PRESSURE
void setup() {
  // put your setup code here, to run once:
   Serial.begin(9600);
  Serial.println("Bosch BME280 Barometric Pressure - Humidity - Temp Sensor | cactus.io"); 

  if (!bme.begin()) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }


  bme.setTempCal(-1);


}

void loop() {
  // put your main code here, to run repeatedly:
   bme.readSensor();
    float pressure = bme.getPressure_MB() ;
    float temperature = bme.getTemperature_C() ;
    
  if(pressure >= 900 && pressure <= 1100){
    Serial.print(bme.getPressure_MB()); Serial.print("\t\t");    // Pressure in millibars
    Serial.print(bme.getHumidity()); Serial.print("\t\t");
    Serial.print(bme.getTemperature_C()); Serial.print(" *C\t");
    Serial.print(bme.getTemperature_F()); Serial.println(" *F\t");

    // add a 2 second delay to slow down the output
    delay(100);
  }

}
#endif



