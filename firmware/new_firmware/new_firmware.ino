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
#define LEFT_TURN 9
#define RIGHT_TURN -3

Servo LeftM;//5
Servo RightM;
int lx,ly,az = 0;
int R1,R2,R3,R4,B2,B4,Mode = 0;
int lx_h,lx_l,az_h,az_l = 0;
void setup() {
  Serial.begin(115200);
  pinMode(2,INPUT);
  pinMode(3,INPUT);
  pinMode(4,INPUT);
  pinMode(5,INPUT);
  LeftM.attach(9);
  RightM.attach(10);
  set_off();
}

void loop() {
  //if (Serial.read() == 'I'){Serial.print("DRIVE");}
  //else Serial.flushRX();
  //responding to queries from usb
  sig_read();
  //Serial.println(Mode);
  //serial_read();
  if (Mode == -1){//Auto Mode
    serial_read();lx = 128; ly = 128; az = B4;  
    convert();
    drive();
  }
  else if (Mode == 0){//RC Mode
    //Serial.println(az);
    lx = 128; ly=R2; az = R4;  
    convert();
    drive();  
    Serial.flushRX();
  }
  else{//STOP MODE
    lx = 128; ly = 128; az = 128;  
    convert();
    drive();  
    Serial.flushRX();
}
  //Serial.print("lx: ");Serial.print(lx);Serial.print(" ly: ");Serial.print(ly);Serial.print(" az: ");Serial.println(az);

}

void set_off(){
  int lx_mid = 1915; //pulseIn(3,HIGH); 
  int az_mid = 1850; //pulseIn(5,HIGH);
  lx_h = lx_mid+150; lx_l = lx_mid-150;
  az_h = az_mid+150; az_l = az_mid-150;
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
  if (R2 < az_h && R2 > az_l)
  R2 = 128;
  else
  R2 = map (R2, 1350, 2480, 0, 255);//az
  Mode = map (R3, 1880,1050 , 0, 2);
  if (R4 < lx_h && R4 > lx_l)
  R4 = 128;
  else 
  R4 = map (R4, 1350, 2480, 0, 255); //lx

  
  //Serial.print("Mode: ");Serial.print(Mode);Serial.print("az: ");Serial.print(R2);Serial.print("lx: ");Serial.println(R4);
  if(abs(R2-90) < TRIM) R2 = 128;
  if(abs(R4-90) < TRIM) R4 = 128;
}
 
void serial_read(){
  if (Serial.available()>9){
    if (Serial.read() == 'B'){
    int B9 = (Serial.read()-'0')*100 + (Serial.read()-'0')*10 + (Serial.read()-'0');
    B2 =(Serial.read()-'0')*100 + (Serial.read()-'0')*10 + (Serial.read()-'0');
    B4 = (Serial.read()-'0')*100 + (Serial.read()-'0')*10 + (Serial.read()-'0');
   //Serial.print("B4: "); Serial.print(B4);
   
    }
    else{B2 = B4 = 128;}
    //B2 = 128;
}
else{  
B2 = B4 = 128;}
//Serial.end();
//Serial.begin(9600);
  Serial.flushRX();
}

void convert(){
 if (ly > 255)ly = 255; else if (ly < 0)ly = 0;
 if (az > 255)az = 255; else if (az < 0)az = 0;
 lx = map (lx, 0, 255, 80, 100);
 ly = map (ly, 0, 255, 85, 95);
 az = map (az, 0, 255, 80, 100);
}


void drive(){
  //Serial.print("lx: ");Serial.print(lx);Serial.print(" ly: ");Serial.print(ly);Serial.print(" az: ");Serial.println(az);
 if(lx == 90){
  if(ly == 90){
   if (az == 90){
     LeftM.write(90);
     RightM.write(90);
   }
   else{
     if(az > 90){
    LeftM.write(az+LEFT_TURN);
    RightM.write(az+LEFT_TURN);  
           Serial.print("left: ");Serial.print(az+10);Serial.print(" right: ");Serial.println(az+10);
         }
     else{
       LeftM.write(az+RIGHT_TURN);
       RightM.write(az+RIGHT_TURN);      
       Serial.print("left: ");Serial.print(az);Serial.print(" right: ");Serial.println(az);
   }
   }
  }
  else{
   if (ly > 90){
   int y = ly - 90; 
   LeftM.write(90+y+13);//if 80 //if 100 
   RightM.write(90-y-7);//needs to be 100 //needs to be 80 
   //Serial.print("left: ");Serial.print(90+y);Serial.print(" right: ");Serial.println(90-y);
   }
   else{
   int y = 90-ly;
   LeftM.write(90-y-7);
   RightM.write(90+y+11);
     //Serial.print("left: ");Serial.print(90-y);Serial.print(" right: ");Serial.println(90+y);
   }
 }
 }
 else{
   if (lx > 90){
   //int x = lx - 90;
   //LeftM.write(90+x+13);//if 80 //if 100 
   //RightM.write(90-x-7);//needs to be 100 //needs to be 80 
      //Serial.print("left: ");Serial.print(90+x);Serial.print(" right: ");Serial.println(90-x);
  LeftM.write(90);
  RightM.write(90);
   }
   else{
   //int x = lx + 90;  
   //LeftM.write(90-x-7);
   //RightM.write(90+x+11);
       //Serial.print("left: ");Serial.print(90-x);Serial.print(" right: ");Serial.println(90+x);
   LeftM.write(90);
   RightM.write(90);
}
 }
}
