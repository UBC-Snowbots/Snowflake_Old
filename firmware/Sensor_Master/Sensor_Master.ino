
/*
Code for single axis gyroscope LISY300AL
> Connect 3.3 V to 3.3V (well, duh!)
> Connect GND to GND (the GND on the arduino that is NOT next to the 5V)

Gyroscope 
> Connect OUT to A3
> Connect PD to digital 8

Accelerometer 
> Connect X to A0 
> Connect Y to A1
> Connect z to A2

Don't forget to calibrate it. 
No need to connect ST anywhere

Best of luck! 

Love,
Arjun
*/

int z2, offset, rot; //gyroscope variables
int x, y, z, x_offset, y_offset, z_offset; //accelerometer variables 

int pinPowerDown = 8;  // digital pin for Power Down mode
// LOW = Normal, HIGH = Power down

void setup()
{
  Serial.begin(9600);           // sets the serial port to 9600
  pinMode(pinPowerDown, OUTPUT);
  digitalWrite(pinPowerDown, LOW);  // set gyroscope to Normal
  delay (2000);
  offset=analogRead(3);
  
  x_offset = analogRead(0);       // read analog input pin 0
  y_offset = analogRead(1);       // read analog input pin 1
  z_offset = analogRead(2); 
}

void loop()
{
  z2 = analogRead(3);   // read analog input pin 3
  rot= z2-offset;
  
  x = (analogRead(0)-x_offset);       // read analog input pin 0
  y = (analogRead(1)-y_offset);       // read analog input pin 1
  z = (analogRead(2)-z_offset);       // read analog input pin 1
  //Serial.print(rot,x,y,z);  
  Serial.print(x, DEC);    // print the acceleration in the X axis
  Serial.print(",");       // prints a space between the numbers
  Serial.print(y, DEC);    // print the acceleration in the Y axis
  Serial.print(",");       // prints a space between the numbers
  Serial.print(z, DEC);  // print the acceleration in the Z axis
  Serial.print(",");
  Serial.println(rot);  // print the rotation rate in the Z axis
  delay(100);              // wait 100ms for next reading
  
}


