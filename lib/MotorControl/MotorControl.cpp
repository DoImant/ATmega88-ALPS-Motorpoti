//////////////////////////////////////////////////////////////////////////////
/// @file MotorControl.cpp
/// @author Kai R.
/// @brief DC motor control class. Control of a DC brush motor with the help of an 
///        L293(D) Quadruple Half-H Driver IC
/// 
/// @date 2021-05-20
/// @version 1.0
/// 
/// @copyright Copyright (c) 2021 Kai R.
/// 
//////////////////////////////////////////////////////////////////////////////
#include "MotorControl.hpp"

//////////////////////////////////////////////////////////////////////////////
/// @brief Method to initialize timer and output pins
/// 
//////////////////////////////////////////////////////////////////////////////
void MotorControl::init(void) {
  ATOMIC_BLOCK(ATOMIC_FORCEON)  {
  #ifdef MC_TIMER2                                // Init itmer2 8 Bit Phase correct PWM, Pin 3 (PD3)
    DDRD |= _BV(PD3);
    TCCR2A = 0;                                   // At first set to normal Operation OC2B disconneted
    OCR2A = OCRXA_PRELOAD;
    TCCR2A |= _BV(WGM20);                         // Mode 5 Phase Correct PWM
    TCCR2B = PRESCALE;
    TCCR2B |= _BV(WGM22);
    TCNT2 = 0;                                    // not really required, since we have an 8 bit counter, but makes the signal more reproducible
  #else                                           // Init timer1 16 Bit Phase correct PWM, Pin 10 (PB2)
    DDRB |= _BV(PB2);
    TCCR1A = 0;
    TCCR1A &= ~(_BV(COM1B1) | _BV(COM1B0));       // At first set to normal Operation OC1B disconneted
    //TCCR1A = _BV(WGM11) | _BV(WGM10);           // Mode 11 PWM, Phase Correct, Top is OCR1A
    TCCR1A = _BV(WGM11);                          // Mode 10 PWM, Phase Correct, Top is ICR1
    TCCR1B = PRESCALE;
    TCCR1B |= _BV(WGM13);
    TCNT1 = 0;                                    // not really required, but makes the signal more reproducible
    ICR1 = PWM_FREQ_PRELOAD;
  #endif
    *OCRXB = DEFAULT_DUTY;                        // Default 50% DutyCycle
  }
  digitalWriteFast(_forwardPin,LOW);
  digitalWriteFast(_reversePin,LOW);
}

//////////////////////////////////////////////////////////////////////////////
/// @brief Method for turning on the PWM signal
/// 
//////////////////////////////////////////////////////////////////////////////
void MotorControl::startPwm(void) {
  if (*OCRXB > 0) {                                // Do only enable if Dutycycle > 0
    ATOMIC_BLOCK(ATOMIC_FORCEON) {
      PWM_ON;
    }
    _pwmIsOn = true;
  }
}

//////////////////////////////////////////////////////////////////////////////
/// @brief Method for turning off the PWM signal
/// 
//////////////////////////////////////////////////////////////////////////////
void MotorControl::stopPwm(void) {
  ATOMIC_BLOCK(ATOMIC_FORCEON) {
    PWM_OFF;
  }
  _pwmIsOn = false;
}

//////////////////////////////////////////////////////////////////////////////
/// @brief Method for setting the direction of motor rotation
/// 
/// @param forward 
/// 
/// If the parameter forward = true then the direction of rotation is also "forward". 
/// If false is passed instead, the direction of rotation is "reverse".
/// The predefined constants FORWARD and REVERSE can be used as parameters.
//////////////////////////////////////////////////////////////////////////////
void MotorControl::switchRotation(bool forward) {
  #ifdef DEBUGMOTOR
  if (forward) {
    Serial.println(F("Switch to forward"));
  } else {
    Serial.println(F("Switch to reverse"));
  }
  #endif
  stopPwm();
  digitalWriteFast(_forwardPin,forward);
  digitalWriteFast(_reversePin,!forward); 
  startPwm();
}

//////////////////////////////////////////////////////////////////////////////
/// @brief Method for stopping the motor.
/// 
//////////////////////////////////////////////////////////////////////////////
void MotorControl::stop() {
 if (PWM_IS_ENABLED) {                  // Stop PWM only if it is enabled
    #ifdef DEBUGMOTOR
    Serial.println("Motor Stop");
    #endif
    stopPwm();
    digitalWriteFast(_forwardPin,LOW);
    digitalWriteFast(_reversePin,LOW);
    _delay_ms(MOTOR_WAIT);              // Wait a millisecond before the Motor can be restarted.
  }
}

//////////////////////////////////////////////////////////////////////////////
/// @brief Method for setting the speed of rotation of the motor.
/// 
/// @param speed 
///
/// speed ist Duty cycle in percent
/// 100 = full speed, 50 = half speed, 0 = stop
//////////////////////////////////////////////////////////////////////////////
void MotorControl::setSpeed(uint8_t speed) {
  #ifdef MC_TIMER2
  *OCRXB = (((PWM_FREQ_PRELOAD * (speed > 100 ? 100 : speed)) + 50) / 100); // +50 to round up
  #else
  ATOMIC_BLOCK(ATOMIC_FORCEON)  {
    *OCRXB = (((static_cast<uint32_t>(PWM_FREQ_PRELOAD) * (speed > 100 ? 100 : speed)) + 50) / 100); // +50 to round up  
  }
  #endif
}

//////////////////////////////////////////////////////////////////////////////
/// @brief Getter method for querying the direction of rotation
/// 
/// @return true 
/// @return false 
///
/// return is true if the direction of rotation is forward else it's false
//////////////////////////////////////////////////////////////////////////////
bool MotorControl::isForward() const {
  return (digitalReadFast(_forwardPin) && !digitalReadFast(_reversePin));
} 

//////////////////////////////////////////////////////////////////////////////
/// @brief Getter method for querying the direction of rotation
/// 
/// @return true 
/// @return false 
///
/// return is true if the direction of rotation is reverse  else it's false
//////////////////////////////////////////////////////////////////////////////
bool MotorControl::isReverse() const {
  return (!digitalReadFast(_forwardPin) && digitalReadFast(_reversePin));
}

//////////////////////////////////////////////////////////////////////////////
/// @brief getter method to query whether the motor is rotating or stopped.
/// 
/// @return true 
/// @return false 
///
/// return is true if the motor is rotating
//////////////////////////////////////////////////////////////////////////////
bool MotorControl::isRunning() const {
  return (digitalReadFast(_forwardPin) || digitalReadFast(_reversePin)) && _pwmIsOn;
} 