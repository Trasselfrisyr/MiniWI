/*
NAME:                 MiniWI Lite Ver.
WRITTEN BY:           JOHAN BERGLUND
CREDITS:              State machine from the Gordophone blog by GORDON GOOD
DATE:                 2016-04-13
FILE SAVED AS:        MiniWI-lite.ino
FOR:                  Arduino Pro Mini, ATmega328
CLOCK:                16.00 MHz CRYSTAL                                        
PROGRAMME FUNCTION:   Wind Controller with EWI style key setup, Freescale MPX5010GP breath sensor 
                      and output to 5-pin DIN MIDI 

HARDWARE NOTES:
* For the MIDI connection, attach a MIDI out Female 180 Degree 5-Pin DIN socket to Arduino.
* Socket is seen from solder tags at rear.
* DIN-5 pinout is:                                         _______ 
*    pin 2 - GND                                          /       \
*    pin 4 - 220 ohm resistor to +5V                     | 1     3 |  MIDI jack
*    pin 5 - Arduino Pin 1 (TX) via a 220 ohm resistor   |  4   5  |
*    all other pins - unconnected                         \___2___/
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
* All key switches connect Ardino digital inputs (with internal pullups) to GND
* 
*/

//_______________________________________________________________________________________________ DECLARATIONS

#define ON_Thr 40       // Set threshold level before switching ON
#define ON_Delay   20   // Set Delay after ON threshold before velocity is checked (wait for tounging peak)
#define breath_max 300  // Blowing as hard as you can
#define modsLo_Thr 411  // Low threshold for mod stick center
#define modsHi_Thr 611  // High threshold for mod stick center
#define octsLo_Thr 311  // Low threshold for octave stick center
#define octsHi_Thr 711  // High threshold for octave stick center

// The three states of our state machine

// No note is sounding
#define NOTE_OFF 1

// We've observed a transition from below to above the
// threshold value. We wait a while to see how fast the
// breath velocity is increasing
#define RISE_WAIT 2

// A note is sounding
#define NOTE_ON 3

// Send CC data no more than every CC_INTERVAL
// milliseconds
#define CC_INTERVAL 40 


//variables setup

int state;                         // The state of the state machine
unsigned long ccSendTime = 0L;     // The last time we sent CC values
unsigned long breath_on_time = 0L; // Time when breath sensor value went over the ON threshold
int initial_breath_value;          // The breath value at the time we observed the transition

long lastDebounceTime = 0;         // The last time the fingering was changed
long debounceDelay = 30;           // The debounce time; increase if the output flickers
int lastFingering = 0;             // Keep the last fingering value for debouncing

byte MIDIchannel=0;                // MIDI channel 1

int breathLevel;

int pressureSensor;  // pressure data from breath sensor, for midi breath cc and breath threshold checks
byte velocity;       // remapped midi velocity from breath sensor

int fingeredNote;    // note calculated from fingering (switches) and octave joystick position
byte activeNote;     // note playing
byte startNote=61;   // set startNote to C# (change this value in steps of 12 to start in other octaves)

byte midistatus=0;
byte x;
byte LedPin = 13;    // select the pin for the LED

            // Key variables, TRUE (1) for pressed, FALSE (0) for not pressed
byte LH1;   // Left Hand key 1 (pitch change -2)
byte LHb;   // Left Hand bis key (pitch change -1 unless both LH1 and LH2 are pressed)
byte LH2;   // Left Hand key 2  (with LH1 also pressed pitch change is -2, otherwise -1)
byte LH3;   // Left Hand key 3 (pitch change -2)
byte LHp1;  // Left Hand pinky key 1 (pitch change +1)
byte LHp2;  // Left Hand pinky key 2 (pitch change -1)
byte RHs;   // Right Hand side key  (pitch change -2 unless LHp1 is pressed)
byte RH1;   // Right Hand key 1 (with LH3 also pressed pitch change is -2, otherwise -1)
byte RH2;   // Right Hand key 2 (pitch change -1)
byte RH3;   // Right Hand key 3 (pitch change -2)
byte RHp1;  // Right Hand pinky key 1 (pitch change +1)
byte RHp2;  // Right Hand pinky key 2 (pitch change -1)
byte RHp3;  // Right Hand pinky key 3 (pitch change -2)
byte OCTup; // Octave switch key (pitch change +12)



//_______________________________________________________________________________________________ SETUP

void setup() {
  pinMode(2, INPUT_PULLUP); // Set inputs with pull-up
  pinMode(3, INPUT_PULLUP); 
  pinMode(4, INPUT_PULLUP); 
  pinMode(5, INPUT_PULLUP); 
  pinMode(6, INPUT_PULLUP); 
  pinMode(7, INPUT_PULLUP); 
  pinMode(8, INPUT_PULLUP); 
  pinMode(9, INPUT_PULLUP); 
  pinMode(10, INPUT_PULLUP); 
  pinMode(11, INPUT_PULLUP); 
  pinMode(12, INPUT_PULLUP);  
  pinMode(14, INPUT_PULLUP); 
  pinMode(15, INPUT_PULLUP); 
  pinMode(16, INPUT_PULLUP);
  
  state = NOTE_OFF;  // initialize state machine
  
  pinMode(LedPin,OUTPUT);   // declare the LED's pin as output

  for (x=1; x<=4; x++){  // Do the flashy-flashy to say we are up and running
    digitalWrite( LedPin, HIGH );
    delay(300);
    digitalWrite( LedPin, LOW );
    delay(300);
  }

  Serial.begin(31250);  // start serial with midi baudrate 31250
  Serial.flush();
}

//_______________________________________________________________________________________________ MAIN LOOP

void loop() {
  
  pressureSensor = analogRead(A3); // Get the pressure sensor reading from analog pin A3

  if (state == NOTE_OFF) {
    if (pressureSensor > ON_Thr) {
      // Value has risen above threshold. Move to the ON_Delay
      // state. Record time and initial breath value.
      breath_on_time = millis();
      initial_breath_value = pressureSensor;
      state = RISE_WAIT;  // Go to next state
    }
  } else if (state == RISE_WAIT) {
    if (pressureSensor > ON_Thr) {
      // Has enough time passed for us to collect our second
      // sample?
      if (millis() - breath_on_time > ON_Delay) {
        // Yes, so calculate MIDI note and velocity, then send a note on event
        readSwitches();
     
        //calculate midi note number from pressed keys and octave shifts
        fingeredNote=startNote-2*LH1-(LHb && !(LH1 && LH2))-LH2-(LH2 && LH1)-2*LH3+LHp1-LHp2+(RHs && !LHp1)-RH1-(RH1 && LH3)-RH2-2*RH3+RHp1-RHp2-2*RHp3+12*OCTup;

        // We should be at tonguing peak, so set velocity based on current pressureSensor value        
        // If initial value is greater than value after delay, go with initial value, constrain input to keep mapped output within 7 to 127
        velocity = map(constrain(max(pressureSensor,initial_breath_value),ON_Thr,breath_max),ON_Thr,breath_max,7,127);
        midiSend((0x90 | MIDIchannel), fingeredNote, velocity); // send Note On message for new note
        activeNote=fingeredNote;
        state = NOTE_ON;
      }
    } else {
      // Value fell below threshold before ON_Delay passed. Return to
      // NOTE_OFF state (e.g. we're ignoring a short blip of breath)
      state = NOTE_OFF;
    }
  } else if (state == NOTE_ON) {
    if (pressureSensor < ON_Thr) {
      // Value has fallen below threshold - turn the note off
      midiSend((0x80 | MIDIchannel), activeNote, velocity); //  send Note Off message 
      state = NOTE_OFF;
    } else {
      // Is it time to send more CC data?
      if (millis() - ccSendTime > CC_INTERVAL) {
         // deal with Breath
         breath();
         ccSendTime = millis();
      }    
      readSwitches();     
      //calculate midi note number from pressed keys
      fingeredNote=startNote-2*LH1-(LHb && !(LH1 && LH2))-LH2-(LH2 && LH1)-2*LH3+LHp1-LHp2+(RHs && !LHp1)-RH1-(RH1 && LH3)-RH2-2*RH3+RHp1-RHp2-2*RHp3+12*OCTup;
  
      if (fingeredNote != lastFingering){ //
        // reset the debouncing timer
        lastDebounceTime = millis();
      }
      if ((millis() - lastDebounceTime) > debounceDelay) {
      // whatever the reading is at, it's been there for longer
      // than the debounce delay, so take it as the actual current state
        if (fingeredNote != activeNote) {
          // Player has moved to a new fingering while still blowing.
          // Send a note off for the current note and a note on for
          // the new note.
          midiSend((0x80 | MIDIchannel), activeNote, velocity); // send Note Off message
          activeNote=fingeredNote;
          velocity = map(constrain(pressureSensor,ON_Thr,breath_max),ON_Thr,breath_max,7,127); // set new velocity value based on current pressure sensor level
          midiSend((0x90 | MIDIchannel), activeNote, velocity); // send Note On message
        }
      }
    }
  }
  lastFingering=fingeredNote; 
}
//_______________________________________________________________________________________________ FUNCTIONS

//  Send a three byte midi message  
  void midiSend(byte midistatus, byte data1, byte data2) {
  digitalWrite(LedPin,HIGH);  // indicate we're sending MIDI data
  Serial.write(midistatus);
  Serial.write(data1);
  Serial.write(data2);
  digitalWrite(LedPin,LOW);  // indicate we're sending MIDI data
}

//***********************************************************

void breath(){
  breathLevel = analogRead(A3); // read voltage on analog pin A3
  breathLevel = map(constrain(breathLevel,ON_Thr,breath_max),ON_Thr,breath_max,0,127);
  midiSend((0xB0 | MIDIchannel), 2, breathLevel);
}

//***********************************************************

void readSwitches(){  
  // Read switches and put inverted value in variables
  LH1=!digitalRead(2);
  LHb=!digitalRead(3);
  LH2=!digitalRead(4);
  LH3=!digitalRead(5);
  LHp1=!digitalRead(6);
  LHp2=!digitalRead(7);
  RHs=!digitalRead(8);
  RH1=!digitalRead(9);
  RH2=!digitalRead(10);
  RH3=!digitalRead(11);
  RHp1=!digitalRead(12);
  RHp2=!digitalRead(14);
  RHp3=!digitalRead(15);
  OCTup=!digitalRead(16);
}

