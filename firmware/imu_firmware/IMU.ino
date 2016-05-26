/* Encoder + Accel + Gyro Firmware Code
*/
#include <Encoder.h>

static const double wheel_factor = 1.068;//need to calculate
Encoder knobLeft(2, 4);
Encoder knobRight(5,6);
int az;
int pinPowerDown = 13;  // digital pin for Power Down mode
int off = 0;
// LOW = Normal, HIGH = Power down
int x, y, z, x_offset, y_offset, z_offset;
float v=0, d=0;
long positionLeft  = 1; //-990
long positionRight = 1;

void setup()
{
  Serial.begin(9600);           // sets the serial port to 9600
  pinMode(pinPowerDown, OUTPUT);
  digitalWrite(pinPowerDown, LOW);  // set gyroscope to Normal
  delay (2000);
  x_offset = analogRead(0);       // read analog input pin 0
  y_offset = analogRead(1);       // read analog input pin 1
  z_offset = analogRead(2); 
}

void loop()
{
 //send_gyro();
 //send_accel(); 
 //send_odom();
 send_odom2();
 delay(100);
}

void send_gyro(){
  az = analogRead(3);       // read analog input pin 0
  Serial.print("rotation rate in z: ");
  Serial.print(az, DEC);  // print the rotation rate in the Z axis
}

void send_accel(){
  x = (analogRead(0)-x_offset);       
  y = (analogRead(1)-y_offset);       
  z = (analogRead(2)-z_offset);
  Serial.print(" X: ");
  Serial.print(x);   
  Serial.print(" Y: ");
  Serial.print(y);
  Serial.print(" Z: ");
  Serial.print(z);
  //Serial.println('E'); 
}


void send_odom(){
  long newLeft, newRight, avg;
  newLeft = knobLeft.read();
  newRight = knobRight.read();
  if (newLeft != positionLeft || newRight != positionRight) {
    avg = ((positionLeft - newLeft)+(positionRight - newRight))/2;
    d += avg*wheel_factor;
    positionLeft = newLeft;
    positionRight = newRight;
  }
  else{
    avg = 0;
  }
  Serial.print( " D: "); //meters
  Serial.print(d - wheel_factor);
  v = avg*wheel_factor;
  Serial.print( " V: ");
  Serial.println(v); //meters/s 
}

void send_odom2() {
  long newLeft, newRight;
  newLeft = knobLeft.read();
  newRight = knobRight.read();
  if (newLeft != positionLeft || newRight != positionRight) {
    Serial.print("Left = ");
    Serial.print(newLeft);
    Serial.print(", Right = ");
    Serial.print(newRight);
    Serial.println();
    positionLeft = newLeft;
    positionRight = newRight;
  }
  // if a character is sent from the serial monitor,
  // reset both back to zero.
  if (Serial.available()) {
    Serial.read();
    Serial.println("Reset both knobs to zero");
    knobLeft.write(0);
    knobRight.write(0);
  }
}

