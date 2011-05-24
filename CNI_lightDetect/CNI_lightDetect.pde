/*
 * CNI_lightDetect sketch for Arduino.
 * 
 * Some code borrowed from http://roamingdrone.wordpress.com/2008/11/13/arduino-and-the-taos-tsl230r-light-sensor-getting-started/
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
 *
 *
 * HISTORY:
 * 2011.05.11 RFD (bobd@stanford.edu) finished a working version.
 */

#define VERSION "0.1"

#ifndef ULONG_MAX 
#define ULONG_MAX 4294967295
#endif

#include <Flash.h>
//#include <Messenger.h>

#define BAUD 115200

// The input pin should be the pin mapped to INT0, unless you change
// the interrupt number below. The output pin can be any digital pin.
#if defined(__AVR_AT90USB1286__)
  // Teensy2.0++ has LED on D6 and INT0 on pin 0 (D0)
  const byte outPin = 6;
  const byte inPin = 0;
  const byte vddPin = 1;
#elif defined(__AVR_ATmega32U4__)
  // Teensy2.0 has LED on pin 11 and INT0 on pin 5 (D0)
  const byte outPin = 11;
  const byte inPin = 5;
  const byte vddPin = 6;
#else
  // Assume Arduino (LED on pin 13)
  const byte outPin = 13;
  const byte inPin = 2;
  const byte vddPin = 3;
#endif

// 1000us = 1ms
unsigned int g_sampleIntervalMicros = 1000;

// The following is the minimum number of pulses needed to make a new measurement.
// In low light conditions, when pulses are few and far between, the integration 
// time for each measurement is increased beyond that specified by the sample interval. 
// In these conditions, the returned value is simply a copy of the previous value.
byte g_minNumPulses = 2;

 // two variables used to track time
unsigned long g_curMicros = micros();
unsigned long g_preMicros = g_curMicros;

// Internal vars used in the frequency-detecting interrupt:
volatile unsigned long g_pulseCount;

 // we'll need to access the amount
 // of time passed
unsigned int g_microsDiff;

// The following is an interrupt routine that is run each time the 
// a pulse is detected on the interrupt pin.
void addPulse(){
  g_pulseCount++;
}

// These should be inside the following function, but that gives a compile error?!?!
static unsigned long irradiance = 0;
static unsigned long preMicros = micros();
unsigned long curMicros;
static unsigned int microsDiff = 0;
unsigned long getIrradiance() {
  // The TAOS 235R has only one sensitivity mode. According to 
  // Figure 1 in the datasheet, Irradiance in uW/cm2 (Ee) is linearly related
  // to frequency in kHz (f0), with Ee = f0. Irradiance in nanoW/cm2 = freq in Hz.

  // If we are in low-light conditions, 
  if(g_pulseCount > g_minNumPulses){
    // Disable interrupts during the calculation so that we get an accurate 'snapshot' of the
    // pulse count and the microsecond count (both are changing while interrupts are running).
    noInterrupts();
    preMicros = curMicros;
    curMicros = micros();
    if(curMicros >= preMicros) {
      microsDiff += curMicros - preMicros;
    }else{
      // handle overflow
      microsDiff += ULONG_MAX - preMicros + curMicros;
    } 
    irradiance = g_pulseCount * (1000000/microsDiff);
    microsDiff = 0;
    // reset the pulse counter
    g_pulseCount = 0;
    interrupts();
  }

  return(irradiance);
}

void setup(){
  Serial.begin(BAUD);
  Serial << F("*********************************************************\n");
  Serial << F("* CNI Light firmware version ") << VERSION << F("\n");
  Serial << F("* Copyright 2011 Bob Dougherty <bobd@stanford.edu>\n");
  Serial << F("* http://cniweb.stanford.edu/wiki/CNI_widgets\n");
  Serial << F("*********************************************************\n\n");
  
  // This probably isn't necessary- external interrupts work even in OUTPUT mode. 
  pinMode(inPin, INPUT);
  digitalWrite(inPin, LOW);
  // Attach interrupt 0
  attachInterrupt(0, addPulse, RISING);
  
  pinMode(outPin, OUTPUT);
  digitalWrite(outPin, LOW);
  
  // Power up the light detector:
  pinMode(vddPin, OUTPUT);
  digitalWrite(vddPin, HIGH);
  
  // Attach the callback function to the Messenger
  //g_message.attach(messageReady);
  
  Serial << F("CNI Light Ready.\n");
}

void loop() {
  // check the value of the light sensor every READ_TM ms
  // calculate how much time has passed
  g_preMicros = g_curMicros;
  g_curMicros = micros();

  if(g_curMicros > g_preMicros) {
    g_microsDiff += g_curMicros - g_preMicros;
  }else if(g_curMicros < g_preMicros) {
    // handle overflow and rollover (Arduino 011)
    g_microsDiff += (ULONG_MAX - g_preMicros + g_curMicros);
  } 
  
  // if enough time has passed to do a new reading...
  if(g_microsDiff >= g_sampleIntervalMicros) {
    // get our current frequency reading
    unsigned long irradiance = getIrradiance();
    Serial << g_microsDiff << F(": ") << irradiance << F("\n");
    // reset the ms counter
    g_microsDiff = 0;
  }
}

