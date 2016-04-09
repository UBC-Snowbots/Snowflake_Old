// create a new folder named "Adafruit_GPS" in Arduino/Library then clone the library
// Library at: https://github.com/adafruit/Adafruit_GPS
// Full guide: https://learn.adafruit.com/adafruit-ultimate-gps/arduino-wiring
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>

SoftwareSerial mySerial(3, 2);
Adafruit_GPS GPS(&mySerial);
#define GPSECHO  false
boolean usingInterrupt = false; //keeps track of using interrupt, off by default!
void useInterrupt(boolean);

void setup() {
  Serial.begin(9600);
  gps_setup();
}

void loop() {
  gps_check_new_data();
  char input = Serial.read();
  if (input == 'I') {
    send_gps(false);
  } else if (input == 'D') {
    send_gps(true);
  }
}

void gps_setup(){
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  //GPS.sendCommand(PGCMD_ANTENNA);
  useInterrupt(true);
  delay(1000);
}

void gps_check_new_data() {
  if (GPS.newNMEAreceived()) {
    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
  }
}
void send_gps(boolean data) {
  //F:fix,N:latitude,W:longitude
  if (data) {
    Serial.print("G"); Serial.print(GPS.latitudeDegrees, 4);
    Serial.print(","); Serial.print(GPS.longitudeDegrees, 4);
    Serial.print(","); Serial.print((int)GPS.fix);
  } else {
    Serial.print("GPS");
  }
}
// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
#ifdef UDR0
  if (GPSECHO)
    if (c) UDR0 = c;
  // writing direct to UDR0 is much much faster than Serial.print
  // but only one character can be written at a time.
#endif
}
void useInterrupt(boolean v) {
  if (v) {
    // Timer0 is already used for millis() - we'll just interrupt somewhere
    // in the middle and call the "Compare A" function above
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  } else {
    // do not call the interrupt function COMPA anymore
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
}
