#include "motor.h"

void motor_init() {
  // put your setup code here, to run once:
 pinMode(LDIR,OUTPUT);
 pinMode(LPWM,OUTPUT);
 pinMode(RPWM,OUTPUT);
 pinMode(RDIR,OUTPUT);

}

void motor_control(int L,int R)
{
  L = constrain(L,0,255); //範囲
  R = constrain(R,0,255);
  
  if(L>=0&&R>=0){
 digitalWrite(LDIR,LOW);
 analogWrite(LPWM,L);
 analogWrite(RPWM,R);
 digitalWrite(RDIR,LOW);
  }
 else if(L==0&&R==0){
 digitalWrite(LDIR,LOW);
 analogWrite(LPWM,LOW);
 analogWrite(RPWM,LOW);
 digitalWrite(RDIR,LOW);
 }
 else if(L>=0&&R<=0){
 digitalWrite(LDIR,LOW);
 analogWrite(LPWM,L);
 analogWrite(RPWM,abs(R));
 digitalWrite(RDIR,LOW);
 }
 else if(R>=0&&L<=0){
 digitalWrite(LDIR,LOW);
 analogWrite(LPWM,abs(L));
 analogWrite(RPWM,R);
 digitalWrite(RDIR,LOW);
 }

}


void motor_control2(int L, int R) //モーターで進む向きの調整
{
  if (L < 0) {
    digitalWrite(LDIR, HIGH);
    L = -L;
  } //-を+に変換
  else digitalWrite(LDIR, LOW);
  if (R < 0) {
    digitalWrite(RDIR, LOW);
    R = -R;
  }
  else digitalWrite(RDIR, HIGH);
  L = constrain(L, 0, 255);//0,255←最大、最小電圧
  R = constrain(R, 0, 255);
  analogWrite(LPWM, L);
  analogWrite(RPWM, R);
}



