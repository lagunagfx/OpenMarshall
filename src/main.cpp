/*

  Marshall Pedalboard for Arduino
  CC BY-SA Jorge Barrientos, 2011

  Revision:
  
  2011-08-04 :
  
  · Foot Switchs 3 and 4 seems to need debouncing
  
  2011-08-03 : 
  
  · First sketch using a protoboard.
  
*/

  // #defines pinout configuration. 
  // "button 5" acts as a SHIFT/FUNCTION key to create keystrokes

#include <Arduino.h>

#define HowManyButtons 5
#define HowManyLeds 4

#define button1Pin 11
#define button2Pin 10
#define button3Pin 9
#define button4Pin 8
#define button5Pin 12

#define led1Pin 7
#define led2Pin 6
#define led3Pin 5
#define led4Pin 4

// : MIDI implementation

#define MIDI_COMMAND_NOTEOFF   0x80
#define MIDI_COMMAND_NOTEON    0x90

// select the midi Note value for each bank
// values are defined to use with the excellente Selected Track Control script
// http://stc.wiffbi.com/

char midiNoteConfig[4][4] = {
  {0x18,0x13,0x0C,0x50},  // Play(selection or startmarker)/Stop (24),Record(19),Metronome(12),Undo(80)
  {0x12,0x10,0x11,0x56},  // Loop (18),Punch In (16),Punch Out (17),Tap Tempo(86)
  {0x00,0x2B,0x2C,0x30},  // Arm Track(0),Fire Selected Clip(43),Fire Next Available Clip(44),Stop Track Clip(48)
  {0x4A,0x4B,0x4C,0x4D}   // Toogle Browser (74), Session/Arrangement(75), Detail(76), Detail Clip/Device(77) 
};

int buttonPin[HowManyButtons] = {button1Pin,button2Pin,button3Pin,button4Pin,button5Pin};

int ledPin[HowManyLeds] = {led1Pin,led2Pin,led3Pin,led4Pin};

// array [1][n] are LAST STORED values, array [0][n] are CURRENT values
// I found it more intuitive this way. Feel free to change as you wish

boolean buttonState[2][HowManyButtons] = {
  {false,false,false,false,false},
  {false,false,false,false,false}
}; 

boolean ledState[] = {false,false,false,false};
boolean shiftMode,exitShift = false;
// blink speed (in miliseconds) for SHIFT mode
int blinkFreq = 75;
int currentBank = 0; 

// stores the internal Atmel timer
unsigned long timer;

// set debounce interval to avoid electric noise
boolean debounced = false;
unsigned long debounceTime = 0;
unsigned long debounceDelay = 20;

// This function sends a Midi CC. 
void midiOut(uint8_t midiCommand, uint8_t midiChannel, uint8_t midiData2, uint8_t midiData3) {

  /* The format of the message to send via serial */
  typedef union {
      struct {
  	uint8_t command;
  	uint8_t channel;
  	uint8_t data2;
  	uint8_t data3;
      } msg;
      uint8_t raw[4];
  } t_midiMsg;

  t_midiMsg midiMsg;

  midiMsg.msg.command = midiCommand;
  midiMsg.msg.channel = midiChannel;
  midiMsg.msg.data2   = midiData2;
  midiMsg.msg.data3   = midiData3;

  Serial.write(midiMsg.raw, sizeof(midiMsg));
}

void setup() {
  // set the push buttons input pins
  for (int f=0;f<HowManyButtons;f++)
    pinMode(buttonPin[f], INPUT);
  // set the LED output pins
  for (int f=0;f<HowManyLeds;f++)
    pinMode(ledPin[f], OUTPUT);

  // Set MIDI baud rate:
  //Serial.begin(31250);
  Serial.begin(115200);

  // Startup LED test sequence
  // just for debugging and visual pimpin'
  //
  // Blink all LEDs four times
  for (int f=0;f<4;f++) {
    // At first, blink the LEDs one at a time
    if ( f == 0 ) {
      for (int g=0;g<4;g++) {
        digitalWrite(ledPin[g], HIGH);      
        delay(400);
        digitalWrite(ledPin[g], LOW);      
      }
      delay(300); 
    }
    // Then blink all at once, 3 times
    else {
      for (int g=0;g<4;g++)
        digitalWrite(ledPin[g], HIGH);      
      delay(300);
      for (int g=0;g<4;g++)
        digitalWrite(ledPin[g], LOW);  
      delay(200);
    }    
  }  // End all the Bling Bling  

}  // END SETUP 



void loop() {  // MAIN LOOP START

  for (int f=0;f<HowManyButtons;f++) {
    // check button state
    buttonState[0][f] = digitalRead(buttonPin[f]);
    
    // if the button state has changed since the last iteration
    if ( (buttonState[0][f] != buttonState[1][f]) && ( debounced == true ) ) {
      // Store current pushbutton state [0] as last state [1]
      buttonState[1][f] = buttonState[0][f];            
      // if button is PRESSED      
      if ( buttonState[0][f] == HIGH ) {

        debounceTime = millis();

        // CHECK: is it one of the first FOUR buttons ?
        if ( f < (HowManyButtons - 1) ) {
          // if SHIFT mode is ON
          if ( shiftMode == true ) {
            // then change the current bank
            currentBank = f;
            // and exit from SHIFT mode
            shiftMode = false;
          }
          else {
            midiOut(MIDI_COMMAND_NOTEON,1,midiNoteConfig[currentBank][f],0x7F);
          }  
        } 
        // or is the SHIFT/FUNCTION key being pressed ???
        else {
          // switch SHIFT/FUNCTION state
          shiftMode = !shiftMode;
          exitShift = true;
        }        
      }
      // ELSE if button is RELEASED
      else 
      {
        if ( ( f < (HowManyButtons - 1) ) && (shiftMode == false) ) {
          // when releasing footswitches, send MIDI NOTEOFF messages
          // only if last key pressed was NOT the SHIFT/FUNCTION key
          if (exitShift == false) {
            midiOut(MIDI_COMMAND_NOTEOFF,1,midiNoteConfig[currentBank][f],0x00);
          }
          else {
            exitShift = false;
          }
        }
      }  
    }  // button check ENDIF  
  }  // end FOR statement

  // : lights block
  // Blink all LEDS if SHIFT mode is ON
  if (shiftMode == true) {
    if (millis() - timer > blinkFreq) {
      // reset timer (storing this loop as the last time the LED was blinked)
      timer = millis();
      // switch the state of the corresponding LED
      for (int l;l<HowManyLeds;l++)
        ledState[l] = !ledState[l];               
    }
    // Turns on or off the LEDs,according to the timer
    for (int l;l<HowManyLeds;l++)
        digitalWrite(ledPin[l],ledState[l]);
  } 
  else {
    for (int l;l<HowManyLeds;l++) {
      if ( l == currentBank ) 
        digitalWrite(ledPin[l], HIGH);
      else
        digitalWrite(ledPin[l], LOW);
    }        
  } 

  // : MIDI send block
  if ( millis() - debounceTime > debounceDelay ) {
     debounceTime = millis();
     debounced = true;
  }
  else {
    debounced = false;    
  }
  
}  // MAIN LOOP ENDS

// : functions description

