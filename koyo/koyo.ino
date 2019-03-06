#include <SPI.h>
#include <SD.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <Servo.h>
#include <cactus_io_BME280_SPI.h>
#include "motor.h"
#include "wireless.h"
#include "sd.h"
#include "fall.h"

Servo H_servo ;

#define GOAL_LAT 35.5168418// 35.515785  134.171798 ゴールの緯度
#define GOAL_LON 134.17147829//ゴールの経度
#define DEG2RAD (PI/180.0)//π表記
#define RAD2DEG (180.0/PI)//数字表記
#define GOAL_RANGE 2//ゴールからの距離
#define HERVEST_RANGE 5 //採取する位置とゴールの距離

#define ser 9           //採取機構のサーボ
#define d_motor1 A5      //採取機構のモータ
#define d_motor2 A4      //     〃
#define BME_SCK 13      // Serial Clock
#define BME_MISO 12     // Serial Data Out
#define BME_MOSI 11     // Serial Data In
#define BME_CS A2       // Chip Select

#define PGAIN 2.0//微調整　動作試験
#define IGAIN 0.01

//#define MODE_HERVEST   //採取動作試験
//#define MODE_BME280    //気圧センサー動作試験
//#define MODE_MOTOR      //モータの調整
#define MODE_GPS    //gps動作試験
//#define MODE_RUN   //走行試験  
//#define HERVEST_RUN  

// Create the BME280 object
BME280_SPI bme(BME_CS,BME_MOSI,BME_MISO,BME_SCK);
 
float SEA_LEVEL ; //海面気圧
float regulate ; //高度調整

//高度
float hight(float Po,float P,float T){
  float a = pow((P/Po),(1/5.257));
  float h = ((a-1)*(T + 273.15))/0.0065 ;
  return h ;
}

float DestLat, DestLon, OriginLat, OriginLon;// 最新の緯度、最新の経度、現在の緯度、現在の経度
float falldata[3];
float rundata[6];
int i = 1 ;

TinyGPSPlus gps;//GPS
SoftwareSerial ss(7, 8); //rx=7,tx=8　←rx,txの場所指定
SoftwareSerial zz(2, 4); //rx=7,tx=8　←rx,txの場所指定


void hervest()
{
  motor_control(0,0) ;
  pinMode(d_motor1,OUTPUT) ;
  pinMode(d_motor2,OUTPUT) ;
  H_servo.attach(ser) ;

  digitalWrite(d_motor1,HIGH) ;
  digitalWrite(d_motor2,LOW) ;
  delay(3000) ;
  H_servo.write(100) ;
  delay(3000) ;
  digitalWrite(d_motor1,LOW) ;
  digitalWrite(d_motor2,LOW) ;
  delay(1000) ;
  digitalWrite(d_motor1,LOW) ;
  digitalWrite(d_motor2,HIGH) ;
  delay(500) ;
  H_servo.write(-100) ;
  delay(3000);

  digitalWrite(d_motor1,LOW) ;
  digitalWrite(d_motor2,LOW) ;
}

void SPI_init()  //通信
{
  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE3);
  SPI.setClockDivider(SPI_CLOCK_DIV8); // 16MHz/8 = 2MHz; (max 10MHz)
}


float PIDcontrol(float command, float current)  //command 目標地　current　現在地
{
  float controlValue;
  float error;
  static float i_error = 0.0;

  error = command - current;
  i_error += error;

  controlValue = PGAIN * error + IGAIN * i_error; //比例制御　
  //Serial.print("error = ");
  //Serial.print(error);
  //Serial.print("\t");
  //Serial.print("I_error = ");
  //Serial.print(i_error);
  //Serial.print("\t");
  //Serial.print("controlValue = ");
  //Serial.println(controlValue);

  return (controlValue);
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
}                      //?


float getDt(void) //時間の定義
{
  float time;
  static float lastTime = (float)millis() / 1000.0;
  float currentTime = (float)millis() / 1000.0;

  time = currentTime - lastTime; //現在の時刻-さっきまでの時刻
  lastTime = (float)millis() / 1000.0;

  return (time);
}

float AngleNormalization(float angle) //左右の判別をしやすくするための角度の定義の修正
{
  if (angle >= 180)
    angle =  angle - 360;
  else if (angle <= -180)
    angle = angle + 360;

  return (angle);
}

void gelay(unsigned long ms) //少し待ってデータをもらう
{
  unsigned long start = millis();
  do
  {
    while (ss.available())
      gps.encode(ss.read());
  }
  while (millis() - start < ms);
}

#ifdef MODE_HERVEST
 
void setup() {
  // 得たアナログ値を1/4して、0-1023の値を0-255に変換
  pinMode(d_motor1,OUTPUT) ;
  pinMode(d_motor2,OUTPUT) ;
 // H_servo.attach(ser) ;
}

void loop() {
  digitalWrite(d_motor1,HIGH) ;
  digitalWrite(d_motor2,LOW) ;
  //H_servo.write(90) ;
  //delay(1000) ;
  //H_servo.write(-90) ;
  //delay(1000);
}
#endif

#ifdef MODE_BME280
void setup() {
  Serial.begin(9600);
  zz.begin(9600);
  Serial.println("Bosch BME280 Barometric Pressure - Humidity - Temp Sensor | cactus.io"); 

  if (!bme.begin()) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }

  bme.setTempCal(-1);
  
  Serial.println("Pressure\tHumdity\t\tTemp\t\tTemp");
}

void loop() {

    bme.readSensor();
    float pressure = bme.getPressure_MB() ;
    float temperature = bme.getTemperature_C() ;
  
    Serial.print(bme.getPressure_MB()); Serial.print("\t\t");    // Pressure in millibars
    Serial.print(bme.getHumidity()); Serial.print("\t\t");
    Serial.print(bme.getTemperature_C()); Serial.print(" *C\t");
    Serial.print(bme.getTemperature_F()); Serial.println(" *F\t");
    zz.print(bme.getPressure_MB()); zz.println("\t\t");
    // add a 2 second delay to slow down the output
    delay(2000);
}
#endif 
#ifdef MODE_MOTOR
void setup(){
  motor_init();
}

void loop(){
  motor_control(75,75);
}
#endif

#ifdef MODE_GPS

static void printFloat(float val, bool valid, int len, int prec)
{
  if (!valid)
  {
    while (len-- > 1)
      Serial.print('*');
    Serial.print(' ');
  }
  else
  {
    Serial.print(val, prec);
    int vi = abs((int)val);
    int flen = prec + (val < 0.0 ? 2 : 1); // . and -
    flen += vi >= 1000 ? 4 : vi >= 100 ? 3 : vi >= 10 ? 2 : 1;
    for (int i=flen; i<len; ++i)
      Serial.print(' ');
  }
  gelay(0);
}
void setup() {
  Serial.begin(9600);
  ss.begin(9600);
  
  
}
void loop() {
  float LAT;
  float LON;
  float rundata[2] ;
  unsigned long t;

  gelay(1000);
  LAT = gps.location.lat();
  LON = gps.location.lng();
  rundata[0] = LAT ;
  rundata[1] = LON ;
  Serial.print("");
  Serial.print(LAT);
  Serial.print("\t");
  Serial.println(LON);
  /*Serial.print("LAT");
  printFloat(LAT, gps.location.isValid(), 11, 8);
  Serial.print("\t");
  Serial.print("LON");
  printFloat(LON, gps.location.isValid(), 12, 8);
  Serial.print("\n");
  */

  transferData(rundata, 2);
  
  t = millis()/1000;
  if(3600*i<t)
  {
    Serial.print(i);
    Serial.println("time");
    i++; 
  }
  delay(1000);
}
#endif

#ifdef MODE_RUN

void setup()
{
  //float x, y, z;
  float angle1, angle2, angle3;
  float controlValue;
  float error; //誤差
  unsigned long distance; //ゴールからの距離
  float p;
  float pressure_origin;//最初の気圧
  unsigned long f;


  Serial.begin(9600); //コンピュータの通信の際の通信しあう頻度の調整　大体9600
  ss.begin(9600);
  motor_init();
  zz.begin(9600);
  /*sd_init();
  bme.setTempCal(-1);

  char str[100];
  Serial.println("check"); //シリアルモニターに送る
  TransferStr("Ready...");
  while (strcmp(str, "START") != 0) {  //始まりの合図
    ReceiveStr(str);
  }

  TransferStr("Start!"); //始まる
  unsigned long time = millis(); //時間を測り始める

  while (1) {  //気球に詰められてる状況
    float h, t, p;
    bme.readSensor();
    t = bme.getPressure_MB();//気温
    p = bme.getPressure_MB(); //気圧
    h = hight(SEA_LEVEL, p, t) + regulate  ;
    
    falldata[0] = t;
    falldata[1] = p;
    falldata[2] = h;
    transferData(falldata, 3); //データを送る

    if (check_st(h) == ST_LAND) {   //パラシュートの切り離し
      Serial.println("release");
      release_para(PARAPIN, 1000); //1秒電圧をかける
      break;
    }
    if (millis() - time >= 600000) {       //600000←10分間　経過で切り離し
      Serial.println("release");
      release_para(PARAPIN, 1000);
      break;
    }

  }
*/
 /* delay(90000);
  Serial.println("release");
  zz.println("release");
  //release_para(PARAPIN, 1000); //1秒電圧をかける
  */
  delay(10000);
  gelay(2000);
  OriginLat = gps.location.lat();
  OriginLon = gps.location.lng();

  rundata[0] = OriginLat;
  rundata[1] = OriginLon;
  /*
    Serial.print("OriginLat=");
    Serial.print(OriginLat);
    Serial.print("\t");
    Serial.print("OriginLon=");
    Serial.println(OriginLon);
  */
  distance =  (unsigned long)TinyGPSPlus::distanceBetween(
                OriginLat,
                OriginLon,
                GOAL_LAT,
                GOAL_LON);

  rundata[2] = distance;
  rundata[3] = 0;

  rundata[4] = 0;
  rundata[5] = 0;
  transferData(rundata, 6);
  if (distance <= GOAL_RANGE)
  {
    Serial.println("goal");
    zz.println("goal");
    motor_control(0, 0);
    while (1);
  }

  motor_control(75, 75);
  delay(10000);
  motor_control(0, 0);
  gelay(1000);
  DestLat = gps.location.lat();
  DestLon = gps.location.lng();


  angle1 = TinyGPSPlus::courseTo(
             OriginLat, OriginLon, DestLat, DestLon);
  angle2 = TinyGPSPlus::courseTo(
             DestLat, DestLon, GOAL_LAT, GOAL_LON);
  angle3 = angle1 - angle2;
  angle3 = -DEG2RAD * AngleNormalization(angle3);

  OriginLat = DestLat;
  OriginLon = DestLon;
  controlValue = PIDcontrol(0, angle3);

  if (controlValue >= 0)
  {
  controlValue = constrain(controlValue, 0, 77);
  }
  else 
  {
    controlValue = constrain(abs(controlValue), 0 , 77);
  }
  
  motor_control(75 - controlValue, 75 + controlValue);

}
void loop()
{
  float x, y, z;
  float originLat, originLon;
  float angle1, angle2, angle3;
  static float angle = 0;
  float dt;
  float controlValue;
  float error;
  unsigned long distance;

  //measure_gyro(&x, &y, &z);
  dt = getDt();
  //angle += z * dt;
  //angle = DEG2RAD*AngleNormalization(angle);
  angle = 0;
  gelay(2000);

  DestLat = gps.location.lat();
  DestLon = gps.location.lng();
/*  
    Serial.print("DestLat=");
    Serial.print(DestLat, 7);
    Serial.print("\tDestLon=");
    Serial.println(DestLon, 7);
*/  
  rundata[0] = DestLat;
  rundata[1] = DestLon;

  save_log(rundata[0] , rundata[1]) ;   //sdに書き込み

  distance = (unsigned long)TinyGPSPlus::distanceBetween(
               DestLat,
               DestLon,
               GOAL_LAT,
               GOAL_LON);
  rundata[2] = distance;

 /* if(distance <= HERVEST_RANGE && i = 1)
  {
    hervest() ;
    i++ ;
  }
*/
  if (distance <= GOAL_RANGE)
  {
    Serial.println("goal");
    zz.println("goal");
    motor_control(0, 0);
    while (1);
  }

  angle1 = TinyGPSPlus::courseTo(
             OriginLat, OriginLon, DestLat, DestLon);
  angle2 = TinyGPSPlus::courseTo(
             OriginLat, OriginLon, GOAL_LAT, GOAL_LON);

  angle3 = angle1 - angle2;
  angle3 = -DEG2RAD * AngleNormalization(angle3);

  rundata[3] = angle3;

  error = angle3 - angle;

  controlValue = PIDcontrol(0, error);
  controlValue = constrain(controlValue, -77, 77);
  //Serial.print("controlValue = ");
  //Serial.println(controlValue);
  motor_control(75 - controlValue, 75 + controlValue);

  rundata[4] = 75 - controlValue;
  rundata[5] = 75 + controlValue;

   transferData(rundata, 6);
  
  OriginLat = DestLat;
  OriginLon = DestLon;
}
#endif

#ifdef HERVEST_RUN
void setup()
{
  motor_init();

  
}
void loop()
{
  motor_control(120,120);
  delay(5000);
  hervest();
  return 0;  
}
#endif


