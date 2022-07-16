#ifndef _MOTOR_CONTROL_HPP
#define _MOTOR_CONTROL_HPP
//////////////////////////////////////////////////////////////////////////////
/// @file MotorControl.h
/// @author Kai R. 
/// @brief Headerfile of a class definition for the control of a DC brush motor
///        with the help of an L293(D) Quadruple Half-H Driver IC
/// 
/// @date 2021-05-20
/// @version 1.0
///
/// @date 2022-07-17
/// - Changed #defines to constexpr
/// 
/// @copyright Copyright (c) 2021 Kai R.
/// 
//////////////////////////////////////////////////////////////////////////////

#include <Arduino.h>
#include <digitalWriteFast.h>
#include <util/atomic.h>

//////////////////////////////////////////////////
// Definitions
//////////////////////////////////////////////////

// #define DEBUGMOTOR
// #define MC_TIMER2    // Enable if Timer2 is to be used

// Timer 1 
// Frequency PRELOAD  =  F_CPU / (PRESCALER * Freq * 2) // phase correct PWM
// CS22 CS21  CS20
//  0     0     0   No Source Clock
//  0     0     1   F_CPU no prescaling
//  0     1     0   F_CPU /    8
//  0     1     1   F_CPU /   64
//  1     0     0   F_CPU /  256
//  1     0     1   F_CPU / 1024
//  1     1     0   External clock source on T1 pin. Clock on falling edge
//  1     1     1   External clock source on T1 pin. Clock on rising edge.


// Timer 2  
// Frequency PRELOAD  =  F_CPU / (PRESCALER * Freq * 2) // phase correct PWM
// CS22 CS21  CS20
//  0     0     0   No Source Clock
//  0     0     1   F_CPU no prescaling
//  0     1     0   F_CPU /    8
//  0     1     1   F_CPU /   32
//  1     0     0   F_CPU /   64
//  1     0     1   F_CPU /  128
//  1     1     0   F_CPU /  256
//  1     1     1   F_CPU / 1024

#if (F_CPU == 16000000L)
  #define PWM_FREQ_PRELOAD 250                  // 500Hz
  #ifdef MC_TIMER2
    #define PRESCALE (_BV(CS22))                // Prescaler 64 timer2
  #else 
    #define PRESCALE (_BV(CS21) | _BV(CS20))    // Prescaler 64 timer1
  #endif
#elif (F_CPU == 8000000L)
  #ifdef MC_TIMER2
    #define PWM_FREQ_PRELOAD 250                // 500Hz
    #define PRESCALE (_BV(CS21) | _BV(CS20))    // Prescaler 32 timer2 
  #else
    #define PWM_FREQ_PRELOAD 1000
    #define PRESCALE (_BV(CS21))                // Prescaler  8 timer1 
  #endif
#elif (F_CPU == 4000000L)
  #ifdef MC_TIMER2
    #define PWM_FREQ_PRELOAD 250                // 250Hz
    #define PRESCALE (_BV(CS21) | _BV(CS20))    // Prescaler 32 timer2
  #else
    #define PWM_FREQ_PRELOAD 500                // 500Hz
    #define PRESCALE (_BV(CS21))                // Prescaler  8 timer1 
  #endif
#else
  #error No processor frequency has been defined! 
#endif

#ifdef MC_TIMER2
  #define OCRXB (&OCR2B)
  #define PWM_ON  (TCCR2A |= _BV(COM2B1))
  #define PWM_OFF (TCCR2A &= ~(_BV(COM2B1) | _BV(COM2B0)))
  #define PWM_IS_ENABLED (TCCR2A & _BV(COM2B1))
#else
  #define OCRXB (&OCR1B)
  #define PWM_ON  (TCCR1A |= _BV(COM1B1))
  #define PWM_OFF (TCCR1A &= ~(_BV(COM1B1) | _BV(COM1B0)))
  #define PWM_IS_ENABLED (TCCR1A & _BV(COM1B1))
#endif

//////////////////////////////////////////////////
// Global constants and variables
//////////////////////////////////////////////////

constexpr bool FORWARD      {true};                     // Usable for the switchRotation method.
constexpr bool REVERSE     {false};                     // Usable for the switchRotation method.
constexpr uint8_t DPIN_PD7     {7};                     // 7 = PD7 Arduino D7 Forward_Pin
constexpr uint8_t DPIN_PB0     {8};                     // 8 = PB0 Arduino D8 Reverse_Pin 
constexpr uint16_t DEFAULT_DUTY {PWM_FREQ_PRELOAD/2};   // Default is 50%
constexpr uint16_t MOTOR_WAIT  {1};                     // Value in ms

//////////////////////////////////////////////////////////////////////////////
/// @brief Class definition for the control of a DC brush motor.
/// 
//////////////////////////////////////////////////////////////////////////////
class MotorControl {

  public:
    MotorControl(uint8_t forwardPin = DPIN_PD7, uint8_t reversePin = DPIN_PB0) :
    _forwardPin(forwardPin), 
    _reversePin(reversePin) {
      pinModeFast(_forwardPin, OUTPUT);
      pinModeFast(_reversePin, OUTPUT);
    };

    void init(void);
    void switchRotation(bool);
    void stop(void);
    void setSpeed(uint8_t);
    bool isForward(void) const;
    bool isReverse(void) const;
    bool isRunning(void) const;    
    
  private:
    void startPwm(void);
    void stopPwm(void);
    uint8_t _enablePin;
    uint8_t _forwardPin;
    uint8_t _reversePin;
    bool _pwmIsOn = false;
};
#endif