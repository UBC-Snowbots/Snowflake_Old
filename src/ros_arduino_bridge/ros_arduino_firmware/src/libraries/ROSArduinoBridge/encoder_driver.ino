/* *************************************************************
   Encoder definitions
   
   Add an "#ifdef" block to this file to include support for
   a particular encoder board or library. Then add the appropriate
   #define near the top of the main ROSArduinoBridge.ino file.
   
   ************************************************************ */


#ifdef USE_BASE

#ifdef ROBOGAIA
  /* The Robogaia Mega Encoder shield */
  #include "MegaEncoderCounter.h"

  /* Create the encoder shield object */
  MegaEncoderCounter encoders = MegaEncoderCounter(4); // Initializes the Mega Encoder Counter in the 4X Count mode
  
  /* Wrap the encoder reading function */
  long readEncoder(int i) {
    if (i == LEFT) return encoders.YAxisGetCount();
    else return encoders.XAxisGetCount();
  }

  /* Wrap the encoder reset function */
  void resetEncoder(int i) {
    if (i == LEFT) return encoders.YAxisReset();
    else return encoders.XAxisReset();
  }
#elif defined(ARDUINO_ENC_COUNTER)
  volatile bool _LeftEncoderBSet;
  volatile long _LeftEncoderTicks = 0L;
  volatile bool _RightEncoderBSet;
  volatile long _RightEncoderTicks = 0L;
  
  // Interrupt service routines for the left motor's quadrature encoder
  void HandleLeftMotorInterruptA()
  {
    // Test transition; since the interrupt will only fire on 'rising' we don't need to read pin A
    _LeftEncoderBSet = digitalReadFast(c_LeftEncoderPinB);   // read the input pin
    // and adjust counter + if A leads B
    #ifdef LeftEncoderIsReversed
      _LeftEncoderTicks -= _LeftEncoderBSet ? -1 : +1;
    #else
      _LeftEncoderTicks += _LeftEncoderBSet ? -1 : +1;
    #endif
  }
   
  // Interrupt service routines for the right motor's quadrature encoder
  void HandleRightMotorInterruptA()
  {
    // Test transition; since the interrupt will only fire on 'rising' we don't need to read pin A
    _RightEncoderBSet = digitalReadFast(c_RightEncoderPinB);   // read the input pin
   
    // and adjust counter + if A leads B
    #ifdef RightEncoderIsReversed
      _RightEncoderTicks -= _RightEncoderBSet ? -1 : +1;
    #else
      _RightEncoderTicks += _RightEncoderBSet ? -1 : +1;
    #endif
  }
  
  /* Wrap the encoder reading function */
  long readEncoder(int i) {
    if (i == LEFT) return _LeftEncoderTicks;
    else return _RightEncoderTicks;
  }

  /* Wrap the encoder reset function */
  void resetEncoder(int i) {
    if (i == LEFT){
      _LeftEncoderTicks=0L;
      return;
    } else { 
      _RightEncoderTicks=0L;
      return;
    }
  }
#else
  #error A encoder driver must be selected!
#endif

/* Wrap the encoder reset function */
void resetEncoders() {
  resetEncoder(LEFT);
  resetEncoder(RIGHT);
}

#endif

