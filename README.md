# IR control for an ALPS motor potentiometer

This program enables the remote control of an ALPS motor rotary potentiometer. This enables volume control for amplifiers via IR remote control. The motor is controlled with the help of an L293 (D) H-bridge driver.

To keep power consumption low, the microcontroller is normally in sleep mode. The microcontroller is only woken up when an impulse comes from the remote control and the motor is controlled according to the programmed key codes. If there is no signal for five seconds at a time, the controller goes back to sleep mode.

## Circuit diagram

[Sheet](https://github.com/DoImant/Stuff/blob/main/ALPS-Drehpoti/ALPS-Potisteuerung.pdf)

## Some details

* The program is designed for IR remote controls that use the NEC protocol.\
 \
If another protocol is to be used, \#define DECODE_NEC must be changed to\
one or more of the following protocol types: DECODE_DENON, DECODE_JVC\
DECODE_KASEIKYO, DECODE_PANASONIC, DECODE_KASEIKYO, DECODE_LG,\
DECODE_SAMSUNG, DECODE_SONY, DECODE_RC5, DECODE_RC6, DECODE_BOSEWAVE,\
DECODE_LEGO_PF, DECODE_WHYNTER.\
\
For more information, please refer to the [IRremote documentation](https://github.com/Arduino-IRremote/Arduino-IRremote).

* The two definitions\
 \
\#define IR_MOTOR_FWD_CODE 0x18  
\#define IR_MOTOR_RVS_CODE 0x52
 \
 \
contain the code that corresponds to the buttons on the remote control\
which are to control the motor (forward, backward). If a different protocol\
or keys other than those defined here are to be used, the values must be adapted\
accordingly.
