# MiniWI
MIDI woodwind controller

NAME:                 MiniWI 
WRITTEN BY:           JOHAN BERGLUND
CREDITS:              State machine from the Gordophone blog by GORDON GOOD
DATE:                 2016-04-13
FILE SAVED AS:        MiniWI.ino
FOR:                  Arduino Pro Mini, ATmega328, version with breakouts for A6 and A7
CLOCK:                16.00 MHz CRYSTAL                                        
PROGRAMME FUNCTION:   Wind Controller with EWI style key setup, Freescale MPX5010GP breath sensor, PS2 style thumb joysticks 
                      for octave selection and pb/mod control, output to 5-pin DIN MIDI 

HARDWARE NOTES:
* For the MIDI connection, attach a MIDI out Female 180 Degree 5-Pin DIN socket to Arduino.
* Socket is seen from solder tags at rear.
* DIN-5 pinout is:                                         _______ 
*    pin 2 - GND                                          /       \
*    pin 4 - 220 ohm resistor to +5V                     | 1     3 |  MIDI jack
*    pin 5 - Arduino Pin 1 (TX) via a 220 ohm resistor   |  4   5  |
*    all other pins - unconnected                         \___2___/
*
* Left hand thumb joystick controls octaves.
* X and Y are connected to Arduino pins A6 and A7, 
* this means a Pro Mini version with breakouts for these pins is required.
* 
*       +1   +2
*       ^
* -1  < o >  +1
*       v
* -2   -1
*
* Right hand thumb joystick controls pitch bend and modulation.
* Pitch bend and modulation are connected to Arduino pins A4 and A5,
* not on DIP rows.
* 
*     PB up
*       ^
* Mod < o > Mod
*       v
*     PB dn
*     
* The Freescale MPX5010GP pressure sensor output (V OUT) is connected to Arduino pin A3.
* 
* Sensor pinout
* 1: V OUT (pin with indent)
* 2: GND
* 3: VCC (to 5V)    
* 4: n/c
* 5: n/c
* 6: n/c
*     
*     
* All key switches connect Arduino digital inputs (with internal pullups) to GND
* 
