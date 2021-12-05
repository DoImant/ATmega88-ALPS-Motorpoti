//////////////////////////////////////////////////////////////////////////////
/// @file main.cpp
/// @author Kai R.
/// @brief Main program for controlling an ALPS potentiometer with the help 
///        of an IR remote control (amplifier volume control).
/// 
/// @date 2021-05-20
/// @version 1.0
/// 
/// @copyright Copyright (c) 2021 Kai R.
/// 
//////////////////////////////////////////////////////////////////////////////

#include <Arduino.h>
#include <avr/sleep.h>        // this AVR library contains the methods that controls the sleep modes

#define IR_USE_AVR_TIMER2     // ATMega88 only: Without this definition, IRremote cannot be compiled.
#define DECODE_NEC
#include <IRremote.hpp>       // Do not change header order.
#include <MotorControl.hpp>

//////////////////////////////////////////////////
// Function forward declaration
//////////////////////////////////////////////////
unsigned long doAlpsMotorAction(void);
void goingToSleep(void);

//////////////////////////////////////////////////
// Definitions
//////////////////////////////////////////////////

//#define DEBUGIRREMOTE

#define IR_RECEIVE_PIN       2  // -> PD2 physical Pin 4.
#define IR_MOTOR_FWD_CODE 0x18
#define IR_MOTOR_RVS_CODE 0x52

#define CPU_ENABLE_TIME 5000   // Leave the Controller on for 5.000 milliseconds. 
#define L293_ENABLE_PIN    4   // -> PD4 physical Pin 6.
#define L293_ENABLE_DELAY  5   // ms
#define MOTOR_STEP       120   // The PWM is enabled the given Time in Milliseconds each time a button is pressed.

//////////////////////////////////////////////////
// Global constants and variables
//////////////////////////////////////////////////

// Default Motor enablePin (PWM) = Timer1 -> 10 PB2 physical Pin 16 / Timer2 -> 3 PD3 physical Pin 5
// Default Motor forwardPin =  7 PD7 physical Pin 13
// Default Motor reversePin =  8 PB0 physical Pin 14
MotorControl myMotor;

//////////////////////////////////////////////////////////////////////////////
/// @brief Set up. Basic settings for the program
/// 
//////////////////////////////////////////////////////////////////////////////

void setup()
{
  #if (defined(DEBUGIRREMOTE) || defined(DEBUGMOTOR))
  Serial.begin(74880);
  #endif

  ADCSRA = 0;       // disable ADC (saves power)

  IrReceiver.begin(IR_RECEIVE_PIN);
  
  #ifdef DEBUGIRREMOTE
  Serial.print(F("Ready to receive IR signals at pin "));
  Serial.println(IR_RECEIVE_PIN);
  #endif

  myMotor.init();
  myMotor.setSpeed(100);
  pinModeFast(L293_ENABLE_PIN,OUTPUT);
}

//////////////////////////////////////////////////////////////////////////////
/// @brief Main program loop
/// 
//////////////////////////////////////////////////////////////////////////////
void loop() {
  static unsigned long activePeriod;
  static unsigned long mStep;

  if ((millis() > activePeriod)) {
    digitalWriteFast(L293_ENABLE_PIN, HIGH);  // Switch motordriver power Off (PNP MOS)    
  #ifdef DEBUGIRREMOTE
    Serial.println("Going to sleep");  
    Serial.flush();
  #endif
    _delay_ms(20);
    goingToSleep();
    // Make sure that after 5 seconds the sleep mode is activated in any case, 
    // even if IrReceiver.decode() is not true. (Interception of possible interference signals)
    activePeriod = millis() + CPU_ENABLE_TIME;  
  }

  if (IrReceiver.decode()) {
  #ifdef DEBUGIRREMOTE
    IrReceiver.printIRResultShort(&Serial);
    if (IrReceiver.decodedIRData.protocol == UNKNOWN) {
        // We have an unknown protocol here, print more info
        IrReceiver.printIRResultRawFormatted(&Serial, true);
    }
  #endif  
    activePeriod = millis() + CPU_ENABLE_TIME;
    if (IrReceiver.decodedIRData.protocol == NEC) {
      switch (IrReceiver.decodedIRData.command) {
      case IR_MOTOR_FWD_CODE:
      case IR_MOTOR_RVS_CODE:
        mStep = doAlpsMotorAction();
        break;
      }
    }
    IrReceiver.resume();
  } else {
    if (myMotor.isRunning() && millis() > mStep) {
      myMotor.stop();
    }
  }  
}

//////////////////////////////////////////////////////////////////////////////
/// @brief Control of the potentiometer motor
///
/// Switching on the L293 motor driver IC. 
/// If the correct IR signals have been sent, the motor of the potentiometer 
/// for volume control is activated (high / low).
/// 
/// @return unsigned long 
/// 
/// Time span that the motor must at least remain switched on 
/// after leaving the function.
///
//////////////////////////////////////////////////////////////////////////////
unsigned long doAlpsMotorAction() {
  if (digitalReadFast(L293_ENABLE_PIN)) {
    digitalWriteFast(L293_ENABLE_PIN,LOW);   // Switch motordriver power on (LOW because of PNP-MOSFet)
    _delay_ms(L293_ENABLE_DELAY);
  }
  switch (IrReceiver.decodedIRData.command) {
  case IR_MOTOR_FWD_CODE:
    if (!myMotor.isForward()) {
      myMotor.switchRotation(FORWARD);
    }
    break;
  case IR_MOTOR_RVS_CODE:
    if (!myMotor.isReverse()) {
      myMotor.switchRotation(REVERSE);
    }
    break;
  }
  return(millis() + MOTOR_STEP);
}

//////////////////////////////////////////////////////////////////////////////
/// @brief Put the controller in sleep mode
/// 
//////////////////////////////////////////////////////////////////////////////
void goingToSleep() {
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);  //Setting the sleep mode, in our case full sleep
  sleep_enable();                       //Enabling sleep mode
  //cli();
  ATOMIC_BLOCK(ATOMIC_FORCEON) {
    // Enable interrupt for D2 (PD2) INT0 falling
    EICRA &= ~(_BV(ISC00) | _BV(ISC01));  // clear existing flags
    EICRA |= _BV(ISC01);                  // set wanted flags (falling level interrupt)
    EIFR   = _BV(INTF0);                  // clear flag for interrupt 0
    EIMSK |= _BV(INT0);                   // enable it
  }
  // We are guaranteed that the sleep_cpu call will be done
  // as the processor executes the next instruction after
  // interrupts are turned on.
  //sei();
  sleep_cpu();                           // activating sleep mode
}

//////////////////////////////////////////////////////////////////////////////
/// @brief Interrupt Service Routine
/// 
/// A falling edge on PD2 (phys. PIN 4) generates an interrupt 
/// and the controller is woken up again.
/// The interrupt is generated by operating an IR remote control.
///
//////////////////////////////////////////////////////////////////////////////
ISR(INT0_vect) {
  sleep_disable();                         // disable sleep mode
  EIMSK &= ~(_BV(INT0));                   // disable interrupt (detachInterrupt)
}