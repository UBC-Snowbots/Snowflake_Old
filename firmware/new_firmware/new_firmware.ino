/*Firmware for old remote for UBC Snowbots
 *Test version
 *Author: Vincent Yuan
 *Date: March 25
*/
#include <SoftwareSerial.h>
#include <stdlib.h>
#include <Servo.h>
#include <math.h>
#define TRIM 8 

Servo LeftM;//5
Servo RightM;
int lx,ly,az = 0;
int R1,R2,R3,R4,Mode = 0;
int lx_h,lx_l,az_h,az_l = 0;
void setup() {
  Serial.begin(9600);
  pinMode(2,INPUT);
  pinMode(3,INPUT);
  pinMode(4,INPUT);
  pinMode(5,INPUT);
  LeftM.attach(9);
  RightM.attach(10);
}

void loop() {
  if (Serial.read() == 'I'){Serial.print("DRIVE");}
  else Serial.flushRX();
  //responding to queries from usb
  sig_read();
  if (Mode == -1)serial_read();
  else if (Mode == 1){lx = 128; ly = 128; az = 128;}
  else {lx = R2; ly=128; az = R4;}
  //Serial.print("lx: ");Serial.print(lx);Serial.print(" ly: ");Serial.print(ly);Serial.print(" az: ");Serial.println(az);

  convert();
  drive();
}

void set_off(){
  int lx_mid = 1915; 
  int az_mid = 1915;
  lx_h = lx_mid+100; lx_l = lx_mid-100;
  az_h = az_mid+100; az_l = az_mid-100;
}

void sig_read(){
  //RY = A3, 1880 - 1050  UP-DOWN
  //RX = A1, 1038 - 1867 LEFT-RIGHT
  //LY = A2, 1878 - 1052 UP - DOWN
  //LX = A4, 1021 - 1850 - LEFT_RIGHT
  //R1 = pulseIn(2,HIGH);//1038 - 1867 RX LEFT-RIGHT
  R2 = pulseIn(3,HIGH);//1878 - 1052 LY UP-DOWN 1350 2480
  R3 = pulseIn(4,HIGH);//1880 - 1050 RY UP-DOWN
  R4 = pulseIn(5,HIGH);//1021 - 1850 LX LEFT-RIGHT
  //R1 = map (R1, 1035, 1880, 0, 255);
  //Serial.print("Mode: ");Serial.print(R3);Serial.print("az: ");Serial.print(R2);Serial.print("lx: ");Serial.println(R4);
  R2 = map (R2, 1350, 2480, 0, 255);//az
  Mode = map (R3, 1880,1050 , 0, 2);
  R4 = map (R4, 1350, 2480, 0, 255); //lx
  //Serial.print("Mode: ");Serial.print(Mode);Serial.print("az: ");Serial.print(R2);Serial.print("lx: ");Serial.println(R4);
  if(abs(R2-90) < TRIM) R2 = 90;
  if(abs(R4-90) < TRIM) R4 = 90;
}
 
void serial_read(){
  if (Serial.available()>9){
    if (Serial.read() == 'B'){
    lx = (Serial.read()-'0')*100 + (Serial.read()-'0')*10 + (Serial.read()-'0');
    ly =(Serial.read()-'0')*100 + (Serial.read()-'0')*10 + (Serial.read()-'0');
    az = (Serial.read()-'0')*100 + (Serial.read()-'0')*10 + (Serial.read()-'0');
    }
    else{lx = ly = az = 128;}
}
else{  
lx = ly = az = 128;}
//Serial.end();
//Serial.begin(9600);
  Serial.flushRX();

}

void convert(){
 if (lx > 255)lx = 255; else if (lx < 0)lx = 0;
 if (az > 255)lx = 255; else if (az < 0)az = 0;
 lx = map (lx, 0, 255, 80, 100);
 ly = map (ly, 0, 255, 80, 100);
 az = map (az, 0, 255, 75, 115);
}


void drive(){
  //Serial.print("lx: ");Serial.print(lx);Serial.print(" ly: ");Serial.println(ly);Serial.print(" az: ");Serial.println(az);
 if(lx == 90){
  if(ly == 90){
   if (az == 90){
     LeftM.write(90);
     RightM.write(90);
   }
   else{
    LeftM.write(az);
    RightM.write(az);  
   }
  }
  else{
   LeftM.write(ly);
   RightM.write(ly);
   }
 }
 else{
   if (lx > 90){
   LeftM.write(lx);//if 80 //if 100 
   RightM.write(lx+20);//needs to be 100 //needs to be 80 
   }
   else{LeftM.write(lx);
   RightM.write(lx-20);}
 }
}
