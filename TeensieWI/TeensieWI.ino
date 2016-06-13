/*
NAME:                 TeensieWI
WRITTEN BY:           JOHAN BERGLUND
CREDITS:              State machine from the Gordophone blog by GORDON GOOD
DATE:                 2016-06-09
FILE SAVED AS:        TeensieWI.ino
FOR:                  PJRC Teensy LC and Teensyduino
CLOCK:                48.00 MHz                                       
PROGRAMME FUNCTION:   Simple Wind Controller using the Freescale MPX5010GP breath sensor
                      and capacitive touch keys. Output to USB MIDI.  

HARDWARE NOTES:
    
* The Freescale MPX5010GP pressure sensor output (V OUT) is connected to pin 21(A7).
* (Warning: no voltage limiting before input, can harm Teensy if excessive pressure is applied.)
* 
* Sensor pinout
* 1: V OUT (pin with indent)
* 2: GND (to GND pin of Teensy)
* 3: VCC (to 5V pin of Teensy)    
* 4: n/c
* 5: n/c
* 6: n/c
*     
* Touch sensors are using the Teensy LC built in touchRead function.
* Electrodes connect directly to Teensy pins.
* 
*/

//_______________________________________________________________________________________________ DECLARATIONS

#define ON_Thr 70       // Set threshold level before switching ON
#define ON_Delay   20   // Set Delay after ON threshold before velocity is checked (wait for tounging peak)
#define breath_max 550  // Threshold for maximum breath

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
#define CC_INTERVAL 20 


//variables setup

int state;                         // The state of the state machine
unsigned long ccSendTime = 0L;     // The last time we sent CC values
unsigned long breath_on_time = 0L; // Time when breath sensor value went over the ON threshold
int initial_breath_value;          // The breath value at the time we observed the transition

unsigned long lastDebounceTime = 0;         // The last time the fingering was changed
unsigned long debounceDelay = 30;           // The debounce time; increase if the output flickers
int lastFingering = 0;             // Keep the last fingering value for debouncing

byte MIDIchannel=1;                // MIDI channel 1

int breathLevel=0;   // breath level (smoothed) not mapped to CC value

int pressureSensor;  // pressure data from breath sensor, for midi breath cc and breath threshold checks
byte velocity;       // remapped midi velocity from breath sensor

int fingeredNote;    // note calculated from fingering (switches) and octave joystick position
byte activeNote;     // note playing
byte startNote=73;   // set startNote to C# (change this value in steps of 12 to start in other octaves)


            // Key variables, TRUE (1) for pressed, FALSE (0) for not pressed
byte LH1;   // Left Hand key 1 (pitch change -2) 
            // Casio style 2nd octave: If LH1 is not touched when LH2 and LH3 are, pitch change +9
byte LH2;   // Left Hand key 2  (with LH1 also pressed pitch change is -2, otherwise -1)
byte LH3;   // Left Hand key 3 (pitch change -2)
byte LHp1;  // Left Hand pinky key 1 (pitch change +1)
byte RH1;   // Right Hand key 1 (with LH3 also pressed pitch change is -2, otherwise -1)
byte RH2;   // Right Hand key 2 (pitch change -1)
byte RH3;   // Right Hand key 3 (pitch change -2)
byte RHp2;  // Right Hand pinky key 2 (pitch change -1)
byte RHp3;  // Right Hand pinky key 3 (pitch change -2, in this version -1 if RHp2 is pressed)
byte OCTup; // Octave switch key (pitch change +12) 
byte OCTdn; // Octave switch key (pitch change -12) 

//_______________________________________________________________________________________________ SETUP

void setup() {

  state = NOTE_OFF;  // initialize state machine
  
}

//_______________________________________________________________________________________________ MAIN LOOP

void loop() {
  
  pressureSensor = analogRead(A7); // Get the pressure sensor reading from analog pin A7

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
        // We should be at tonguing peak, so set velocity based on current pressureSensor value        
        // If initial value is greater than value after delay, go with initial value, constrain input to keep mapped output within 7 to 127
        velocity = map(constrain(max(pressureSensor,initial_breath_value),ON_Thr,breath_max),ON_Thr,breath_max,7,127);
        breathLevel=constrain(max(pressureSensor,initial_breath_value),ON_Thr,breath_max);
        usbMIDI.sendNoteOn(fingeredNote, velocity, MIDIchannel); // send Note On message for new note 
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
      usbMIDI.sendNoteOff(activeNote, velocity, MIDIchannel); //  send Note Off message 
      breathLevel=0;
      state = NOTE_OFF;
    } else {
      // Is it time to send more CC data?
      if (millis() - ccSendTime > CC_INTERVAL) {
         // deal with Breath, Pitch Bend and Modulation
         breath();
         ccSendTime = millis();
      }
    }
    readSwitches();
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
        velocity = map(constrain(pressureSensor,ON_Thr,breath_max),ON_Thr,breath_max,7,127); // set new velocity value based on current pressure sensor level
        usbMIDI.sendNoteOn(fingeredNote, velocity, MIDIchannel); // send Note On message for new note
        usbMIDI.sendNoteOff(activeNote, 0, MIDIchannel); // send Note Off message for previous note (legato)
        activeNote=fingeredNote;
      }
    }
  }
  lastFingering=fingeredNote; 
}
//_______________________________________________________________________________________________ FUNCTIONS

void breath(){
  int breathCC;
  breathLevel = breathLevel*0.8+pressureSensor*0.2; // smoothing of breathLevel value
  breathCC = map(constrain(breathLevel,ON_Thr,breath_max),ON_Thr,breath_max,0,127);
  usbMIDI.sendControlChange(2, breathCC, MIDIchannel);
}

//***********************************************************

void readSwitches(){  
  // Read switches and put value in variables
  LH1=touchRead(17)>1500;
  LH2=touchRead(4)>1500;
  LH3=touchRead(3)>1500;
  LHp1=touchRead(18)>1200;
  RH1=touchRead(19)>1500;
  RH2=touchRead(22)>1500;
  RH3=touchRead(23)>1500;
  RHp2=touchRead(1)>1200;
  RHp3=touchRead(0)>1200;
  OCTup=touchRead(15)>1500;
  OCTup=touchRead(16)>1500;
  //calculate midi note number from pressed keys  
  fingeredNote=startNote-2*LH1-LH2-(LH2 && LH1)-2*LH3+LHp1-RH1-(RH1 && LH3)-RH2-2*RH3-RHp2-2*RHp3+(RHp2 && RHp3)+12*OCTup-12*OCTdn+9*(!LH1 && LH2 && LH3);
}
