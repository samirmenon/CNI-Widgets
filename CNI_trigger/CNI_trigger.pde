/*
 * CNI_trigger sketch for Arduino.
 * 
 * 
 * A simple sketch to receive very brief (4usec), frequent pulses and 
 * convert them to longer-duration, less frequent pulses. At the Stanford CNI, 
 * we use this to convert the GE 3.3v scope trigger pulses, which occur once per
 * slice and are very brief (~4 usec), into 5v TTL pulses that are longer
 * duration (~1msec) and occur only at the beginning of a scan. To do this,
 * we assume that these slice trigger pulses occur at least every 1/2 second and
 * that a new scan never starts less than 1/2 sec later than the beginning of 
 * the previous scan's last slice. 
 * 
 *
 * Copyright 2011 Robert F. Dougherty.
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You might have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * TO DO:
 *   - Manual reset button
 *   - Switch to allow a 'pass-thru' state where we send the
 *     slice pulses.
 *   - Check for wrap-around for timers?
 *
 * HISTORY:
 * 2011.03.28 RFD (bobd@stanford.edu) finished a working version.
 */

#define VERSION "1.0"

// Flash library is available from http://arduiniana.org/libraries/Flash/
// We make extensive use of this so that we can be verbose in our messages
// without using up precious RAM. (Using Flash saved us over 2Kb of RAM!)
#include <Flash.h>
#include <Messenger.h>

#define BAUD 57600

// The approximate duration of the output pulse, in milliseconds.
// Note that we don't account for delays in the code, so it will 
// always be a bit longer than this.
unsigned int outPulseDurationMillisec = 5;

unsigned int g_sliceCount = 0;
unsigned int g_triggerCount = 0;

// The input pin should be the pin mapped to INT0, unless you change
// the interrupt number below. The output pin can be any digital pin.
#if defined(__AVR_AT90USB1286__)
  // Teensy2.0++ has LED on D6 and INT0 on pin 0 (D0)
  const byte outPin = 6;
  const byte inPin = 0;
#elif defined(__AVR_ATmega32U4__)
  // Teensy2.0 has LED on pin 11 and INT0 on pin 5 (D0)
  const byte outPin = 11;
  const byte inPin = 5;
#else
  // Assume Arduino (LED on pin 13)
  const byte outPin = 13;
  const byte inPin = 2;
#endif

// Internal vars used in the input-pulse detecting interrupt:
volatile byte outPinOn = false;
volatile unsigned long lastInPulseMillis;
volatile unsigned long outPinOnMillisec;

// Instantiate Messenger object used for serial port communication.
Messenger g_message = Messenger(',','[',']');

// Create the Message callback function. This function is called whenever a complete 
// message is received on the serial port.
void messageReady() {
  int val[10];
  byte i = 0;
  if(g_message.available()) {
    // get the command byte
    char command = g_message.readChar();
    switch(command) {
    
    case '?': // display help text
      Serial << F("CNI Trigger Device\n");
      Serial << F("Sends TTL pulses out on pin ") << (int)outPin << F(".\n");
      Serial << F("\nFor example:\n");
      Serial << F("[t] will send a trigger pulse immediately.\n");
      Serial << F("[s,0] will send one pulse at the beginning of the input pulse train.\n");
      Serial << F("[s,31] will send a pulse for every 31 input pulses.\n\n");
      break;
      
    case 's': // Set slice count
      while(g_message.available()) val[i++] = g_message.readInt();
      if(i!=1){
        Serial << F("ERROR: Set slice count requires one param.\n");
      }else{
          g_sliceCount = val[0];
          Serial << F("Slice count set to [") << g_sliceCount << F("]\n");
      }
      break;
 
    case 't': // force output trigger
      // First force the pin low, in case it was already on. This will ensure that
      // we get a change on the pin no matter what state we were in.
      if(outPinOn) digitalWrite(outPin, LOW);
      triggerOut();
      break;

    case 'r': // reset
      g_triggerCount = 0;
      digitalWrite(outPin, LOW);
      break;

    default:
      Serial << F("[") << command << F("]\n");
      Serial << F("ERROR: Unknown command.\n\n");
    } // end switch
  } // end while
}


void setup()
{
  Serial.begin(BAUD);
  Serial << F("*********************************************************\n");
  Serial << F("* CNI Trigger firmware version ") << VERSION << F("\n");
  Serial << F("* Copyright 2011 Bob Dougherty <bobd@stanford.edu>\n");
  Serial << F("* http://cniweb.stanford.edu/wiki/CNI_widgets\n");
  Serial << F("*********************************************************\n\n");
  
  // This probably isn't necessary- external interrupts work even in OUTPUT mode. 
  pinMode(inPin, INPUT);
  // Turn on the internal pull-up resistor:
  // NOTE: needs testing with GE trigger. An external pull-down might be better?
  // If we go with a pull-down, change FALLING to RISING (although with a 4usec
  // pulse, it doesn't really matter).
  digitalWrite(inPin, HIGH);
  
  pinMode(outPin, OUTPUT);
  // Most Arduino boards have two external interrupts: 
  // 0 (on digital pin 2) and 1 (on digital pin 3).
  attachInterrupt(0, triggerIn, FALLING);
  digitalWrite(outPin, LOW);
  
  // Attach the callback function to the Messenger
  g_message.attach(messageReady);
  
  Serial << F("CNI Trigger Ready. Send the ? command ([?]) for help.\n");
}

void loop(){
  // Reset the output, if needed:
  if(outPinOn){
    // Turn off the output pin after the requested duration.
    // Detect and correct counter wrap-around:
    if(millis()<outPinOnMillisec) outPinOnMillisec += 4294967295UL;
    if(millis()-outPinOnMillisec > outPulseDurationMillisec)
      digitalWrite(outPin, LOW);
  }
    
  // Handle Messenger's callback:
  if(Serial.available())  g_message.process(Serial.read());
}

// The following is an interrupt routine that is run each time the 
// a pulse is detected on the trigger input pin.
void triggerIn(){
  if(g_sliceCount>0){
    // g_sliceCount > 0 means that we trigger on every g_sliceCount slice
    if(g_triggerCount>=g_sliceCount)
      g_triggerCount = 0;
    if(g_triggerCount==0)
      triggerOut();
  }
  g_triggerCount++;
  lastInPulseMillis = millis();
  Serial << F("p");
}

void triggerOut(){
    digitalWrite(outPin, HIGH);
    outPinOn = true;
    outPinOnMillisec = millis();
    Serial << F("OUT\n");
}

