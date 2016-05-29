/* *************************************************************
   Encoder driver function definitions - by James Nugen
   ************************************************************ */
   
   
#ifdef ARDUINO_ENC_COUNTER
  // Left encoder
  #define c_LeftEncoderInterrupt 0
  #define c_LeftEncoderPinA 2
  #define c_LeftEncoderPinB 4
  #define LeftEncoderIsReversed
  
  // Right encoder
  #define c_RightEncoderInterrupt 1
  #define c_RightEncoderPinA 3
  #define c_RightEncoderPinB 5
#endif
   
long readEncoder(int i);
void resetEncoder(int i);
void resetEncoders();

