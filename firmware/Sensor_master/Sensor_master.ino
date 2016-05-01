//Author: Vincent Yuan
//Important: Make Sure SoftwareSerial Library has been modified to properly flush input buffer
#include <Wire.h>
#include <SoftwareSerial.h> 
#define address 0x1E //0011110b, I2C 7bit address of HMC5883
int x, y, z, x_offset, y_offset, z_offset;
void setup()
{
  Serial.begin(9600);      // sets the serial port to 9600
  delay (2000);
  x_offset = analogRead(0);       // read analog input pin 0
  y_offset = analogRead(1);       // read analog input pin 1
  z_offset = analogRead(2); 
  Compass_Setup();
}

void loop()
{
  //if (Serial.read() == 'B'){
    send_accel();
  //Compass_Read();
  //Serial.flushRX();
  //}
  
}

void send_accel(){
  x = (analogRead(0)-x_offset);       
  y = (analogRead(1)-y_offset);       
  z = (analogRead(2)-z_offset);
  Serial.print('X');
  Serial.println(x);   
  Serial.print('Y');
  Serial.println(y);
  Serial.print('Z');
  Serial.println(z);
  //Serial.println('E'); 
  delay(100); 
}
void Compass_Setup(){
  Wire.begin();

  //Put the HMC5883 IC into the correct operating mode
  Wire.beginTransmission(address); //open communication with HMC5883
  Wire.write(0x02); //select mode register
  Wire.write(0x00); //continuous measurement mode
  Wire.endTransmission();
}
void Compass_Read(){
  //Tell the HMC5883L where to begin reading data
  Wire.beginTransmission(address);
  Wire.write(0x03); //select register 3, X MSB register
  Wire.endTransmission();


  //Read data from each axis, 2 registers per axis
  Wire.requestFrom(address, 6);
  if(6<=Wire.available()){
    x = Wire.read()<<8; //X msb
    x |= Wire.read(); //X lsb
    z = Wire.read()<<8; //Z msb
    z |= Wire.read(); //Z lsb
    y = Wire.read()<<8; //Y msb
    y |= Wire.read(); //Y lsb
  }

  /*Serial.print(x);
   Serial.print(",");
   Serial.print(y);
   Serial.print(",");
   Serial.println(z);
   */
  delay(250);
}

