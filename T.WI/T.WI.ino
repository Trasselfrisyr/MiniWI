#include <EEPROM.h>

/*
NAME:                 T.WI
WRITTEN BY:           JOHAN BERGLUND
DATE:                 2017-10-17
FILE SAVED AS:        T.WI.ino
FOR:                  PJRC Teensy LC and Teensyduino
CLOCK:                48.00 MHz                                       
PROGRAMME FUNCTION:   Woodwind Controller using a Freescale MPX5010GP breath sensor,
                      a PSP1000 joystick and capacitive touch keys. Output to USB MIDI.  

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
* PSP style thumb slide joystick controls pitch bend and modulation.
* Pitch bend and modulation are connected to pins A6 and A0.
* Connections on joystick, bottom view with connectors towards you, L to R:
* 1: to VCC 3.3V
* 2: X (or Y depending on orientation) to analog input
* 3: to GND
* 4: Y (or X depending on orientation) to analog input
* 
*     PB up
*       ^
* Mod < o > Glide
*       v
*     PB dn
*   
*/

//_______________________________________________________________________________________________ DECLARATIONS

#define touch_Thr 1500  // threshold for Teensy touchRead, 1300-1800
#define ON_Thr 300      // Set threshold level before switching ON
#define ON_Delay   20   // Set Delay after ON threshold before velocity is checked (wait for tounging peak)
#define breath_max 2200 // Threshold for maximum breath
#define modsLo_Thr 1600 // Low threshold for mod stick center
#define modsHi_Thr 2480 // High threshold for mod stick center
#define modsMin 960     // PSP joystick min value
#define modsMax 3080    // PSP joystick max value
#define PB_sen1 4096    // Pitch Bend sensitivity 0 to 8192 where 8192 is full pb range, 4096 is half range
#define PB_sen2 683     // Selectable 1/12 PB sense for synths w fixed 12 semitones pb range
#define CCN_Port 5      // Controller number for portamento level
#define CCN_PortOnOff 65// Controller number for portamento on/off
#define portaMax  30    // 1 to 127, max portamento level

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
#define CC_INTERVAL 5 

// EEPROM addresses for settings
#define VERSION_ADDR 0
#define TRANS1_ADDR 14
#define MIDI_ADDR 16
#define BREATH_CC_ADDR 18
#define BREATH_AT_ADDR 20
#define PORTAM_ADDR 24
#define PB_ADDR 26
#define EXTRA_ADDR 28
#define TRANS2_ADDR 40
#define BREATHCURVE_ADDR 44

//"factory" values for settings
#define VERSION 4
#define MIDI_FACTORY 1            // 1-16
#define BREATH_CC_FACTORY 2       // thats CC#2, see ccList
#define BREATH_AT_FACTORY 0       // aftertouch default off
#define PORTAM_FACTORY 2          // 0 - OFF, 1 - ON, 2 - SW
#define PB_FACTORY 0              // 0 - 1/2, 1 - 1/12
#define EXTRA_FACTORY 0           // 0 - Modulation wheel, 1 - Pitch Bend Vibrato
#define BREATHCURVE_FACTORY 2     // 0 to 12 (-4 to +4, S1 to S4)
#define TRANS1_FACTORY 0          // 1 - +2 semitones (C to D, F to G)
#define TRANS2_FACTORY 0          // 1 - -7 semitones (C to F, D to G) "alto mode"

#define maxSamplesNum 120

//variables setup


static int waveformsTable[maxSamplesNum] = {
  // Sine wave
  0x7ff, 0x86a, 0x8d5, 0x93f, 0x9a9, 0xa11, 0xa78, 0xadd, 0xb40, 0xba1,
  0xbff, 0xc5a, 0xcb2, 0xd08, 0xd59, 0xda7, 0xdf1, 0xe36, 0xe77, 0xeb4,
  0xeec, 0xf1f, 0xf4d, 0xf77, 0xf9a, 0xfb9, 0xfd2, 0xfe5, 0xff3, 0xffc,
  0xfff, 0xffc, 0xff3, 0xfe5, 0xfd2, 0xfb9, 0xf9a, 0xf77, 0xf4d, 0xf1f,
  0xeec, 0xeb4, 0xe77, 0xe36, 0xdf1, 0xda7, 0xd59, 0xd08, 0xcb2, 0xc5a,
  0xbff, 0xba1, 0xb40, 0xadd, 0xa78, 0xa11, 0x9a9, 0x93f, 0x8d5, 0x86a,
  0x7ff, 0x794, 0x729, 0x6bf, 0x655, 0x5ed, 0x586, 0x521, 0x4be, 0x45d,
  0x3ff, 0x3a4, 0x34c, 0x2f6, 0x2a5, 0x257, 0x20d, 0x1c8, 0x187, 0x14a,
  0x112, 0xdf, 0xb1, 0x87, 0x64, 0x45, 0x2c, 0x19, 0xb, 0x2,
  0x0, 0x2, 0xb, 0x19, 0x2c, 0x45, 0x64, 0x87, 0xb1, 0xdf,
  0x112, 0x14a, 0x187, 0x1c8, 0x20d, 0x257, 0x2a5, 0x2f6, 0x34c, 0x3a4,
  0x3ff, 0x45d, 0x4be, 0x521, 0x586, 0x5ed, 0x655, 0x6bf, 0x729, 0x794
};


int state;                         // The state of the state machine
unsigned long ccSendTime = 0L;     // The last time we sent CC values
unsigned long breath_on_time = 0L; // Time when breath sensor value went over the ON threshold
int initial_breath_value;          // The breath value at the time we observed the transition

unsigned long lastDebounceTime = 0;         // The last time the fingering was changed
unsigned long debounceDelay = 20;           // The debounce time; increase if the output flickers
int lastFingering = 0;             // Keep the last fingering value for debouncing

byte MIDIchannel=1;                // MIDI channel 1

unsigned short breathCC;           // OFF:MW:BR:VL:EX:MW+:BR+:VL+:EX+
unsigned short breathAT;
unsigned short portamento;         // switching on cc65? just cc5 enabled? SW:ON:OFF
unsigned short curve;              // selected curve
unsigned short PB;
unsigned short mod;
unsigned short trans1;
unsigned short trans2;

byte ccList[9] = {0,1,2,7,11,1,2,7,11};  // OFF, Modulation, Breath, Volume, Expression,(then same sent in hires)

int breathLevel=0;   // breath level (smoothed) not mapped to CC value
int oldbreath=0;
unsigned int oldbreathhires=0;
unsigned int breathValHires=0;

byte portIsOn=0;     // keep track and make sure we send CC with 0 value when off threshold
int oldport=0;

int pressureSensor;  // pressure data from breath sensor, for midi breath cc and breath threshold checks
byte velocitySend;   // remapped midi velocity from breath sensor

int modLevel;
int oldmod=0;
int lfoDepth=2;
int lfoLevel=0;
int lfo=0;

int pitchBend;
int oldpb=8192;
int PB_sens;
int modCCnumber = 1;

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


unsigned int curveM4[] = {0,4300,7000,8700,9900,10950,11900,12600,13300,13900,14500,15000,15450,15700,16000,16250,16383};
unsigned int curveM3[] = {0,2900,5100,6650,8200,9500,10550,11500,12300,13100,13800,14450,14950,15350,15750,16150,16383};
unsigned int curveM2[] = {0,2000,3600,5000,6450,7850,9000,10100,11100,12100,12900,13700,14400,14950,15500,16000,16383};
unsigned int curveM1[] = {0,1400,2850,4100,5300,6450,7600,8700,9800,10750,11650,12600,13350,14150,14950,15650,16838};
unsigned int curveIn[] = {0,1023,2047,3071,4095,5119,6143,7167,8191,9215,10239,11263,12287,13311,14335,15359,16383};
unsigned int curveP1[] = {0,600,1350,2150,2900,3800,4700,5600,6650,7700,8800,9900,11100,12300,13500,14850,16838};
unsigned int curveP2[] = {0,400,800,1300,2000,2650,3500,4300,5300,6250,7400,8500,9600,11050,12400,14100,16383};
unsigned int curveP3[] = {0,200,500,900,1300,1800,2350,3100,3800,4600,5550,6550,8000,9500,11250,13400,16383};
unsigned int curveP4[] = {0,100,200,400,700,1050,1500,1950,2550,3200,4000,4900,6050,7500,9300,12100,16282};
unsigned int curveS1[] = {0,600,1350,2150,2900,3800,4700,6000,8700,11000,12400,13400,14300,14950,15500,16000,16383};
unsigned int curveS2[] = {0,600,1350,2150,2900,4000,6100,9000,11000,12100,12900,13700,14400,14950,15500,16000,16383};
unsigned int curveS3[] = {0,600,1350,2300,3800,6200,8700,10200,11100,12100,12900,13700,14400,14950,15500,16000,16383};
unsigned int curveS4[] = {0,600,1700,4000,6600,8550,9700,10550,11400,12200,12900,13700,14400,14950,15500,16000,16383};

//_______________________________________________________________________________________________ SETUP

void setup() {
  analogReadResolution(12);   // set resolution of ADCs to 12 bit
  state = NOTE_OFF;           // initialize state machine
  pinMode(13,OUTPUT);         // use Teensy LED for breath on indication
  digitalWrite(13,LOW);  

  // if stored settings are not for current version, they are replaced by factory settings
  if (readSetting(VERSION_ADDR) != VERSION){
    writeSetting(VERSION_ADDR,VERSION);
    writeSetting(MIDI_ADDR,MIDI_FACTORY);
    writeSetting(TRANS1_ADDR,TRANS1_FACTORY);
    writeSetting(TRANS2_ADDR,TRANS2_FACTORY);
    writeSetting(BREATH_CC_ADDR,BREATH_CC_FACTORY);
    writeSetting(BREATH_AT_ADDR,BREATH_AT_FACTORY);
    writeSetting(PORTAM_ADDR,PORTAM_FACTORY);
    writeSetting(PB_ADDR,PB_FACTORY);
    writeSetting(EXTRA_ADDR,EXTRA_FACTORY);
    writeSetting(BREATHCURVE_ADDR,BREATHCURVE_FACTORY);
  }
  // read settings from EEPROM
  MIDIchannel  = readSetting(MIDI_ADDR);
  trans1       = readSetting(TRANS1_ADDR);
  trans2       = readSetting(TRANS2_ADDR);
  breathCC     = readSetting(BREATH_CC_ADDR);
  breathAT     = readSetting(BREATH_AT_ADDR);
  portamento   = readSetting(PORTAM_ADDR);
  PB           = readSetting(PB_ADDR);
  mod          = readSetting(EXTRA_ADDR);
  curve        = readSetting(BREATHCURVE_ADDR);

  settings();

  if (PB) PB_sens = PB_sen2; else PB_sens = PB_sen1;
  if (trans1) startNote += 2;
  if (trans2) startNote -= 7;
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
        // If initial value is greater than value after delay, go with initial value, constrain input to keep mapped output within 1 to 127
        breathLevel=constrain(max(pressureSensor,initial_breath_value),ON_Thr,breath_max);
        breathValHires = breathCurve(map(constrain(breathLevel,ON_Thr,breath_max),ON_Thr,breath_max,0,16383));
        velocitySend = (breathValHires >>7) & 0x007F;
        velocitySend = constrain(velocitySend,1,127);
        breath(); // send breath data
        usbMIDI.sendNoteOn(fingeredNote, velocitySend, MIDIchannel); // send Note On message for new note 
        digitalWrite(13,HIGH);
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
      usbMIDI.sendNoteOff(activeNote, velocitySend, MIDIchannel); //  send Note Off message 
      digitalWrite(13,LOW);
      breathLevel=0;
      state = NOTE_OFF;
    } else {
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
          breathValHires = breathCurve(map(constrain(breathLevel,ON_Thr,breath_max),ON_Thr,breath_max,0,16383));
          velocitySend = (breathValHires >>7) & 0x007F;
          velocitySend = constrain(velocitySend,1,127); // set new velocity value based on current pressure sensor level
          usbMIDI.sendNoteOn(fingeredNote, velocitySend, MIDIchannel); // send Note On message for new note         
          usbMIDI.sendNoteOff(activeNote, 0, MIDIchannel); // send Note Off message for previous note (legato)
          activeNote=fingeredNote;
        }
      }
    }
  }
  // Is it time to send more CC data?
  if (millis() - ccSendTime > CC_INTERVAL) {
    // deal with Breath, Pitch Bend and Modulation
    breath();
    modulation();
    pitch_bend();
    ccSendTime = millis();
  }
  lastFingering=fingeredNote; 
}
//_______________________________________________________________________________________________ FUNCTIONS

// non linear mapping function (http://playground.arduino.cc/Main/MultiMap)
// note: the _in array should have increasing values
unsigned int multiMap(unsigned int val, unsigned int* _in, unsigned int* _out, uint8_t size)
{
  // take care the value is within range
  // val = constrain(val, _in[0], _in[size-1]);
  if (val <= _in[0]) return _out[0];
  if (val >= _in[size-1]) return _out[size-1];

  // search right interval
  uint8_t pos = 1;  // _in[0] allready tested
  while(val > _in[pos]) pos++;

  // this will handle all exact "points" in the _in array
  if (val == _in[pos]) return _out[pos];

  // interpolate in the right segment for the rest
  return (val - _in[pos-1]) * (_out[pos] - _out[pos-1]) / (_in[pos] - _in[pos-1]) + _out[pos-1];
}

//**************************************************************

// map breath values to selected curve
unsigned int breathCurve(unsigned int inputVal){
  // 0 to 16383, moving mid value up or down
  switch (curve){
    case 0:
      // -4
      return multiMap(inputVal,curveIn,curveM4,17);
      break;
    case 1:
      // -3
      return multiMap(inputVal,curveIn,curveM3,17);
      break;
    case 2:
      // -2
      return multiMap(inputVal,curveIn,curveM2,17);
      break;
    case 3:
      // -1
      return multiMap(inputVal,curveIn,curveM1,17);
      break;
    case 4:
      // 0, linear
      return inputVal;
      break;
    case 5:
      // +1
      return multiMap(inputVal,curveIn,curveP1,17);
      break;
    case 6:
      // +2
      return multiMap(inputVal,curveIn,curveP2,17);
      break;
    case 7:
      // +3
      return multiMap(inputVal,curveIn,curveP3,17);
      break;
    case 8:
      // +4
      return multiMap(inputVal,curveIn,curveP4,17);
      break;
    case 9:
      // S1
      return multiMap(inputVal,curveIn,curveS1,17);
      break;
    case 10:
      // S2
      return multiMap(inputVal,curveIn,curveS2,17);
      break;
    case 11:
      // S3
      return multiMap(inputVal,curveIn,curveS3,17);
      break;
    case 12:
      // S4
      return multiMap(inputVal,curveIn,curveS4,17);
      break;
  }
}

//**************************************************************

void breath(){
  int breathCCval,breathCCvalFine;
  unsigned int breathCCvalHires;
  breathLevel = breathLevel*0.8+pressureSensor*0.2; // smoothing of breathLevel value
  //breathCCval = map(constrain(breathLevel,ON_Thr,breath_max),ON_Thr,breath_max,0,127);
  breathCCvalHires = breathCurve(map(constrain(breathLevel,ON_Thr,breath_max),ON_Thr,breath_max,0,16383));
  breathCCval = (breathCCvalHires >>7) & 0x007F;
  breathCCvalFine = breathCCvalHires & 0x007F;

  if (breathCCval != oldbreath){ // only send midi data if breath has changed from previous value
    if (breathCC){
      // send midi cc
      usbMIDI.sendControlChange(ccList[breathCC], breathCCval, MIDIchannel);
    }
    if (breathAT){
      // send aftertouch
      usbMIDI.sendAfterTouch(breathCCval, MIDIchannel);
    }
    oldbreath = breathCCval;
  }
  
  if (breathCCvalHires != oldbreathhires){
    if (breathCC > 4){ // send high resolution midi
        usbMIDI.sendControlChange(ccList[breathCC]+32, breathCCvalFine, MIDIchannel);
    }
    oldbreathhires = breathCCvalHires;   
  }
}

//**************************************************************

void pitch_bend(){
  pitchBend = analogRead(A0); // read voltage on analog pin A0
  if (pitchBend > modsHi_Thr){
    pitchBend = oldpb*0.6+0.4*map(constrain(pitchBend,modsHi_Thr,modsMax),modsHi_Thr,modsMax,8192,(8193 + PB_sens)); // go from 8192 to 16383 (full pb up) when off center threshold going up
  } else if (pitchBend < modsLo_Thr){
    pitchBend = oldpb*0.6+0.4*map(constrain(pitchBend,modsMin,modsLo_Thr),modsMin,modsLo_Thr,(8192 - PB_sens),8192); // go from 8192 to 0 (full pb dn) when off center threshold going down
  } else {
    pitchBend = oldpb*0.6+8192*0.4; // released, so smooth your way back to zero
    if ((pitchBend > 8187) && (pitchBend < 8197)) pitchBend = 8192; // 8192 is 0 pitch bend, don't miss it bc of smoothing
  }
  if (mod || (ccList[breathCC]==1)){
    if (PB) pitchBend += lfoLevel/6; else pitchBend += lfoLevel;
    pitchBend=constrain(pitchBend, 0, 16383);
  }
  if (pitchBend != oldpb){// only send midi data if pitch bend has changed from previous value
    usbMIDI.sendPitchBend(pitchBend, MIDIchannel);
    oldpb=pitchBend;
  }
}

//***********************************************************

void modulation(){
  int modRead = analogRead(A6); // read voltage on analog pin A6
  if (modRead < modsLo_Thr){
    modLevel = map(constrain(modRead,modsMin,modsLo_Thr),modsMin,modsLo_Thr,127,0); // go from 0 to full modulation when off center threshold going left(?)
  } else {
    modLevel = 0; // zero modulation in center position
  }
  if (modLevel != oldmod){  // only send midi data if modulation has changed from previous value
    if (!mod  && (ccList[breathCC] != modCCnumber)) usbMIDI.sendControlChange(modCCnumber, modLevel, MIDIchannel);
    oldmod=modLevel;
  }
  if (mod || (ccList[breathCC] == modCCnumber)) {
    lfo = waveformsTable[(millis()/2)%maxSamplesNum] - 2047;
    lfoLevel = lfo * modLevel / 1024 * lfoDepth;
  }
  if (portamento && (modRead > modsHi_Thr)) {    // if we are enabled and over the threshold, send portamento
   if (!portIsOn) {
      if (portamento == 2){ // if portamento midi switching is enabled
        usbMIDI.sendControlChange(CCN_PortOnOff, 127, MIDIchannel);
      }
    portIsOn=1;
    }
    int portCC;
    portCC = map(constrain(modRead,modsHi_Thr,modsMax),modsHi_Thr,modsMax,0,portaMax); // go from 0 to full when off center threshold going right(?)
    if (portCC!=oldport){
      usbMIDI.sendControlChange(CCN_Port, portCC, MIDIchannel);
    }
    oldport = portCC;        
  } else if (portIsOn) {                                    // we have just gone below threshold, so send zero value
    usbMIDI.sendControlChange(CCN_Port, 0, MIDIchannel);
    if (portamento == 2){                                   // if portamento midi switching is enabled
      usbMIDI.sendControlChange(CCN_PortOnOff, 0, MIDIchannel);
    }
    portIsOn=0;
    oldport = 0; 
  }
}

//***********************************************************

void writeSetting(byte address, unsigned short value){
  union {
    byte v[2];
    unsigned short val;
  } data;
  data.val = value;
  EEPROM.write(address, data.v[0]);
  EEPROM.write(address+1, data.v[1]);  
}

//***********************************************************

unsigned short readSetting(byte address){
  union {
    byte v[2];
    unsigned short val;
  } data;  
  data.v[0] = EEPROM.read(address); 
  data.v[1] = EEPROM.read(address+1); 
  return data.val;
}

//***********************************************************

void readSwitches(){  
  // Read switches and put value in variables
  LH1=touchRead(17)>touch_Thr;
  LH2=touchRead(4)>touch_Thr;
  LH3=touchRead(3)>touch_Thr;
  LHp1=touchRead(18)>touch_Thr;
  RH1=touchRead(19)>touch_Thr;
  RH2=touchRead(22)>touch_Thr;
  RH3=touchRead(23)>touch_Thr;
  RHp2=touchRead(1)>touch_Thr;
  RHp3=touchRead(0)>touch_Thr;
  OCTup=touchRead(15)>touch_Thr;
  OCTdn=touchRead(16)>touch_Thr;
  //calculate midi note number from pressed keys  
  fingeredNote=startNote-2*LH1-LH2-(LH2 && LH1)-2*LH3+LHp1-RH1-(RH1 && LH3)-RH2-2*RH3-RHp2-2*RHp3+(RHp2 && RHp3)+12*OCTup-12*OCTdn+9*(!LH1 && LH2 && LH3);
}

//***********************************************************

void numberBlink(byte number){
  for (int i=0; i < number; i++){
    digitalWrite(13,HIGH);
    delay(200);
    digitalWrite(13,LOW);
    delay(200);
  }
  if (number == 0){
    digitalWrite(13,HIGH);
    delay(30);
    digitalWrite(13,LOW);
    delay(30);
    digitalWrite(13,HIGH);
    delay(30);
    digitalWrite(13,LOW);
    delay(30); 
    digitalWrite(13,HIGH);
    delay(30);
    digitalWrite(13,LOW);
    delay(30);   
    digitalWrite(13,HIGH);
    delay(30);
    digitalWrite(13,LOW);
    delay(200);
  }
}

//***********************************************************

void settings(){  
  int y = analogRead(A0); // read joystick y axis, A0
  int x = analogRead(A6); // read joystick x axis, A6
  byte sel = 0;
  byte bin = 0;
  byte p1,p2;
  
  if      (y > ((modsHi_Thr+modsMax)/2)) sel = 1; // PB up -> Breath CC/AT settings
  else if (y < ((modsLo_Thr+modsMin)/2)) sel = 2; // PB dn -> MIDI CH setting
  else if (x > ((modsHi_Thr+modsMax)/2)) sel = 3; // Glide -> Various on/off
  else if (x < ((modsLo_Thr+modsMin)/2)) sel = 4; // Mod   -> Breath curve setting

  numberBlink(sel);
  
  delay(1000);
  
  if (sel){
    // Read switches and put value in variables
    bin = (touchRead(19)>touch_Thr) + (touchRead(22)>touch_Thr)*2 + (touchRead(23)>touch_Thr)*4;
    p1 = touchRead(1)>touch_Thr;
    p2 = touchRead(0)>touch_Thr;

    switch (sel){
      case 1:
        breathCC = constrain(bin,0,8);
        breathAT = p1;
        writeSetting(BREATH_CC_ADDR,breathCC);
        writeSetting(BREATH_AT_ADDR,breathAT);
        break;
      case 2:
        MIDIchannel = bin + p1*8 + 1;
        writeSetting(MIDI_ADDR,MIDI_FACTORY);
        break;
      case 3:
        if (bitRead(bin,0)){
          if (portamento) portamento = 0; else portamento = 2; // portamento 2 is both switch on/off and portamento value, portamento 1 is just value
          writeSetting(PORTAM_ADDR,portamento);
        }
        if (bitRead(bin,1)){
          PB = !PB;
          writeSetting(PB_ADDR,PB);
        }
        if (bitRead(bin,2)){
          mod = !mod;
          writeSetting(EXTRA_ADDR,mod);
        }
        if (p1){
          trans1 = !trans1;
          writeSetting(TRANS1_ADDR,trans1);
        }
        if (p2){
          trans2 = !trans2;
          writeSetting(TRANS2_ADDR,trans2);
        }
        if ((bin == 7) && p1 && p2){ //restore factory settings
          writeSetting(MIDI_ADDR,MIDI_FACTORY);
          writeSetting(TRANS1_ADDR,TRANS1_FACTORY);
          writeSetting(TRANS2_ADDR,TRANS2_FACTORY);
          writeSetting(BREATH_CC_ADDR,BREATH_CC_FACTORY);
          writeSetting(BREATH_AT_ADDR,BREATH_AT_FACTORY);
          writeSetting(PORTAM_ADDR,PORTAM_FACTORY);
          writeSetting(PB_ADDR,PB_FACTORY);
          writeSetting(EXTRA_ADDR,EXTRA_FACTORY);
          writeSetting(BREATHCURVE_ADDR,BREATHCURVE_FACTORY);
          MIDIchannel  = readSetting(MIDI_ADDR);
          trans1       = readSetting(TRANS1_ADDR);
          trans2       = readSetting(TRANS2_ADDR);
          breathCC     = readSetting(BREATH_CC_ADDR);
          breathAT     = readSetting(BREATH_AT_ADDR);
          portamento   = readSetting(PORTAM_ADDR);
          PB           = readSetting(PB_ADDR);
          mod          = readSetting(EXTRA_ADDR);
          curve        = readSetting(BREATHCURVE_ADDR);
        }
        break;
      case 4:
        curve = constrain((bin+p1*8),0,12);
        writeSetting(BREATHCURVE_ADDR,curve);
    }
    numberBlink(bin);
    delay(1000);
    numberBlink(p1);
    delay(1000);
    numberBlink(p2);
  }
}



