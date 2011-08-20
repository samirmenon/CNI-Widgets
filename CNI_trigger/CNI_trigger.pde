/*
 * CNI_trigger sketch for Arduino.
 * 
 * 
 * A simple sketch to generate TTl pulses when told to do so through a serial
 * port command.  
 *
 * When running this sketch on a Teensy, be sure to set the "USB Type" to
 * "Serial" (under the Arduino Tools menu). 
 * 
 * This sketch will also listen for input pulses and send out a serial port 
 * character whenever one is detected. It uses an external interrupt, so it
 * can reliably detect very brief pulses. We use this to detect the GE 3.3v 
 * scope trigger pulses (~4 usec pulse duration). 
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
 *
 * HISTORY:
 * 2011.03.28 RFD (bobd@stanford.edu) finished a working version.
 * 2011.05.24 RFD: cleaned up source & comments, removed down-counter code,
 * and added a serial message upon every input pulse that's detected.
 */

#define VERSION "1.0"

// Flash library is available from http://arduiniana.org/libraries/Flash/
#include <Flash.h>
#include <Messenger.h>

#define BAUD 57600

#define DEFAULT_OUT_PULSE_MSEC 5
#define DEFAULT_IN_PULSE_STATE 0

// Most Arduino boards have two external interrupts: 
// 0 (on digital pin 2) and 1 (on digital pin 3). On the 
// Teensy, these pins aremight be different (check docs).
#define INTERRUPT 0

// The approximate duration of the output pulse, in milliseconds.
// Note that we don't account for delays in the code, so it will 
// always be a bit longer than this.
unsigned int g_outPulseDuration = DEFAULT_OUT_PULSE_MSEC;

// The input pin should be the pin mapped to INT0, unless you change
// the interrupt number below. The output pin can be any digital pin.
#if defined(__AVR_AT90USB1286__)
  // Teensy2.0++ has LED on D6 and INT0 on pin 0 (D0)
  const byte g_outPin = 6;
  const byte g_inPin = 0;
#elif defined(__AVR_ATmega32U4__)
  // Teensy2.0 has LED on pin 11 and INT0 on pin 5 (D0)
  const byte g_outPin = 11;
  const byte g_inPin = 5;
#else
  #error "Unknown board!"
#endif
HardwareSerial g_uart = HardwareSerial();

byte g_inPulseEdge = FALLING;

volatile byte g_outPinOn = false;
volatile unsigned long g_outPulseStart;

// Instantiate Messenger object used for serial port communication.
Messenger g_message = Messenger(',','[',']');

// Create the Message callback function. This function is called whenever a complete 
// message is received on the serial port.
void messageReady() {
  int val[10];
  byte i,j;
  char serialBuffer[64];
  if(g_message.available()) {
    // get the command byte
    char command = g_message.readChar();
    switch(command) {
    
    case '?': // display help text
      Serial << F("CNI Trigger Device\n");
      Serial << F("Sends TTL pulses out on pin ") << (int)g_outPin << F(".\n");
      Serial << F("Listens for pulses in on pin ") << (int)g_inPin << F(".\n");
      Serial << F("\nCommands:\n");
      Serial << F("[t]   will send a trigger pulse. This also disables the input pulse\n");
      Serial << F("      detection. Send a [p] command to re-enable it.\n\n");
      Serial << F("[o,N] set the output pulse duration to N milliseconds. Send with\n");
      Serial << F("      no argument to show the current pulse duration.\n\n");
      Serial << F("[p]   enable input pulse detection. Send a [t] to disable.\n\n");
      Serial << F("[r]   reset default state.\n\n");
      Serial << F("[s,D] Send data (D) out over the local serial port. D can be up to 64\n");
      Serial << F("      bytes any binary data, except that it cannot contain the ASCII codes\n");
      Serial << F("      for '[', ']', or ','. The data are sent out at ") << BAUD << F(" bps.\n\n");      
      break;
      
    case 'o': // Set out-pulse duration (msec)
      while(g_message.available()) val[i++] = g_message.readInt();
      if(i>1){
        Serial << F("ERROR: Set output pulse duration requires no more than one param.\n");
      }else if(i==1){
          g_outPulseDuration = val[0];
      }
      Serial << F("Output pulse duration is set to ") << g_outPulseDuration << F(" msec.\n");
      break;
 
    case 't': // force output trigger
      // First force the pin low, in case it was already on. This will ensure that
      // we get a change on the pin no matter what state we were in.
      if(g_outPinOn) digitalWrite(g_outPin, LOW);
      triggerOut();
      // Automatically disable input pulse detection
      setInPulseState(0);
      break;

    case 'p': // turn on input pulse detection
      //Serial << F("Enabling input pulses\n");
      setInPulseState(1);
      break;

    case 'r': // reset
      setInPulseState(DEFAULT_IN_PULSE_STATE);
      g_outPulseDuration = DEFAULT_OUT_PULSE_MSEC;
      digitalWrite(g_outPin, LOW);
      break;

    case 's': // stream serial data
      j = 0;
      if(g_message.available()){
        j = g_message.copyString(serialBuffer, 64);
        g_uart.write((byte*)serialBuffer,j);
      }else{
        Serial << F("WARNING: empty string.\n");
      }
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
  pinMode(g_inPin, INPUT);
  
  pinMode(g_outPin, OUTPUT);
  digitalWrite(g_outPin, LOW);
  setInPulseState(DEFAULT_IN_PULSE_STATE);
  
  // Attach the callback function to the Messenger
  g_message.attach(messageReady);
  
  // Set up the UART to support streaming serial data through.
  g_uart.begin(BAUD);
  
  Serial << F("CNI Trigger Ready. Send the ? command ([?]) for help.\n\n");
}

void loop(){
  // Reset the output, if needed:
  if(g_outPinOn){
    // Turn off the output pin after the requested duration.
    // Detect and correct counter wrap-around:
    if(millis()<g_outPulseStart) g_outPulseStart += 4294967295UL;
    if(millis()-g_outPulseStart > g_outPulseDuration)
      digitalWrite(g_outPin, LOW);
  }
    
  // Handle Messenger's callback:
  if(Serial.available())  g_message.process(Serial.read());
}

void setInPulseState(byte state){
  // Turn on the internal pull-up resistor if we want to detect falling edges.
  if(g_inPulseEdge==FALLING)
    digitalWrite(g_inPin, HIGH);
  else
    digitalWrite(g_inPin, LOW);
  // Attach or detach the interrupt
  if(state)
    attachInterrupt(INTERRUPT, triggerIn, g_inPulseEdge);
  else
    detachInterrupt(INTERRUPT);
}

// The following is an interrupt routine that is run each time the 
// a pulse is detected on the trigger input pin.
void triggerIn(){
  Serial << F("p");
}

void triggerOut(){
    digitalWrite(g_outPin, HIGH);
    g_outPinOn = true;
    g_outPulseStart = millis();
    //Serial << F("OUT\n");
}

