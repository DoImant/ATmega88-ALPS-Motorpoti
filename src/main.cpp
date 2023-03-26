//////////////////////////////////////////////////////////////////////////////
/// @file main.cpp
/// @author Kai R.
/// @brief Main program for controlling an ALPS potentiometer with the help
///        of an IR remote control (amplifier volume control).
///
/// @date 2021-05-20
/// @version 1.0
///
/// @date 2022-07-15
/// @version 1.0.1
/// - Changed handling with millis()
/// - Changed #defines to constexpr
/// - Added additional condition to turn off motor driver (in loop())
///
/// @date 2023-03-25
/// @version 1.0.2
/// The handling of millis() encapsulated in a class.
/// doAlpsMotorAction() changed
/// Added a delay after the µC wakes up to be able to use IRremote version 4.x as well.  
///
/// @copyright Copyright (c) 2021 Kai R.
///
//////////////////////////////////////////////////////////////////////////////

#include <Arduino.h>
#include <avr/sleep.h>   // this AVR library contains the methods that controls the sleep modes

// #define IR_USE_AVR_TIMER2   // IRremote Ver <=3.1.1 ATMega88 only: Without this definition, IRremote cannot be
// compiled.
#define DECODE_NEC
#include <IRremote.hpp>   // Do not change header order.
#include <MotorControl.hpp>

//////////////////////////////////////////////////
// Definitions
//////////////////////////////////////////////////

class Timer {
public:
  void start() { timeStamp = millis(); }
  bool operator()(const unsigned long duration) const { return (millis() - timeStamp >= duration) ? true : false; }

private:
  unsigned long timeStamp {0};
};

//////////////////////////////////////////////////
// Global constants and variables
//////////////////////////////////////////////////

constexpr uint8_t IR_RECEIVE_PIN {2};   // -> PD2 physical Pin 4.
constexpr uint8_t IR_MOTOR_FWD_CODE {0x18};
constexpr uint8_t IR_MOTOR_RVS_CODE {0x52};

constexpr uint16_t CPU_ENABLE_TIME {5000};   // Leave the Controller on for 5.000 milliseconds.
constexpr uint8_t L293_ENABLE_PIN {4};       // -> PD4 physical Pin 6.
constexpr uint8_t L293_ENABLE_DELAY {5};     // ms
constexpr uint8_t MOTOR_STEP {120};    // The PWM is enabled the given Time in Millis each time a button is pressed.
constexpr uint8_t MOTOR_SPEED {100};   // Speed is duty cycle in percent

Timer stepTimer;
Timer enableTimer;

// Default Motor enablePin (PWM) = Timer1 -> 10 PB2 physical Pin 16 / Timer2 -> 3 PD3 physical Pin 5
// Default Motor forwardPin =  7 PD7 physical Pin 13
// Default Motor reversePin =  8 PB0 physical Pin 14
MotorControl myMotor;

//////////////////////////////////////////////////
// Function forward declaration
//////////////////////////////////////////////////
// unsigned long doAlpsMotorAction(void);
void doAlpsMotorAction(uint16_t);
void goingToSleep(void);

//////////////////////////////////////////////////////////////////////////////
/// @brief Set up. Basic settings for the program
///
//////////////////////////////////////////////////////////////////////////////

void setup() {
#if (defined(DEBUGIRREMOTE) || defined(DEBUGMOTOR))
  Serial.begin(74880);
#endif

  // Disable Analog Comparator
  ACSR |= bit(ACD);
  power_adc_disable();

  IrReceiver.begin(IR_RECEIVE_PIN);

#ifdef DEBUGIRREMOTE
  Serial.print(F("Ready to receive IR signals at pin "));
  Serial.println(IR_RECEIVE_PIN);
#endif

  myMotor.init();
  myMotor.setSpeed(MOTOR_SPEED);
  pinModeFast(L293_ENABLE_PIN, OUTPUT);

  enableTimer.start(); 
}

//////////////////////////////////////////////////////////////////////////////
/// @brief Main program loop
///
//////////////////////////////////////////////////////////////////////////////
void loop() {

  if (enableTimer(CPU_ENABLE_TIME)) {
    digitalWriteFast(L293_ENABLE_PIN, HIGH);   // Switch motordriver power Off (PNP MOS)
#if (defined(DEBUGIRREMOTE) || defined(DEBUGMOTOR))
    Serial.println("Going to sleep");
    Serial.flush();
#endif
    delay(20);
    goingToSleep();
    // Give IRremote some time before decoding. Necessary from version 4.x.x. Version 3.1.1 works without delay
    delay(120); 
  }

  if (IrReceiver.decode()) {
#ifdef DEBUGIRREMOTE
    IrReceiver.printIRResultShort(&Serial);
    if (IrReceiver.decodedIRData.protocol == UNKNOWN) {
      // We have an unknown protocol here, print more info
      IrReceiver.printIRResultRawFormatted(&Serial, true);
    }
#endif
    enableTimer.start();
    if (IrReceiver.decodedIRData.protocol == NEC) {
      switch (IrReceiver.decodedIRData.command) {
        case IR_MOTOR_FWD_CODE: [[fallthrough]];
        case IR_MOTOR_RVS_CODE:
          doAlpsMotorAction(IrReceiver.decodedIRData.command);
          stepTimer.start();
          break;
        default:
          // Turn off the motor driver if no matching IR code is received.
          if (!digitalReadFast(L293_ENABLE_PIN)) { digitalWriteFast(L293_ENABLE_PIN, HIGH); }
      }
    }
    IrReceiver.resume();
  } else if (myMotor.isRunning() && stepTimer(MOTOR_STEP)) {
    myMotor.stop();
  }
}

//////////////////////////////////////////////////////////////////////////////
/// @brief Control of the potentiometer motor
///
/// Switching on the L293 motor driver IC.
/// If the correct IR signals have been sent, the motor of the potentiometer
/// for volume control is activated (high / low).
///
//////////////////////////////////////////////////////////////////////////////
void doAlpsMotorAction(uint16_t command) {
  if (digitalReadFast(L293_ENABLE_PIN)) {
    digitalWriteFast(L293_ENABLE_PIN, LOW);   // Switch motordriver power on (LOW because of PNP-MOSFet)
    delay(L293_ENABLE_DELAY);
  }
  switch (command) {
    case IR_MOTOR_FWD_CODE:
      if (!myMotor.isForward()) { myMotor.switchRotation(FORWARD); }
      break;
    case IR_MOTOR_RVS_CODE:
      if (!myMotor.isReverse()) { myMotor.switchRotation(REVERSE); }
      break;
  }
}

//////////////////////////////////////////////////////////////////////////////
/// @brief Put the controller in sleep mode
///
//////////////////////////////////////////////////////////////////////////////
void goingToSleep() {
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);   // Setting the sleep mode, in our case full sleep
  sleep_enable();
  ATOMIC_BLOCK(ATOMIC_FORCEON) {
    // Enable interrupt for D2 (PD2) INT0 falling
    EICRA &= ~(_BV(ISC00) | _BV(ISC01));
    EICRA |= _BV(ISC01);   // set wanted flags (falling level interrupt)
    EIFR = _BV(INTF0);     // clear flag for interrupt 0
    EIMSK |= _BV(INT0);    // enable it
  }
  sleep_cpu();   // activating sleep mode
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
  sleep_disable();
  EIMSK &= ~(_BV(INT0));   // disable interrupt (detachInterrupt)
}