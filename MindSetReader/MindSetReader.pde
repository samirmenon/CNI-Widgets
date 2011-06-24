/*
 * Arduino Bluetooth Interface with Mindset
 * 
 *
 * MindSetReader.pde
 * Modified from Tutorial.pde by Sean M. Montgomery 2010/09, which is 
 * example code provided license-free by NeuroSky, Inc.
 *
 * Program displays selected brainwave measurement on a bar LED and 
 * the "errorRate" measure of signal quality (zero = good) on an LED
 * and sends selected brainwave measurements and time of data aquisition
 * to a computer via the arduino serial port.
 *
 * See http://www.produceconsumerobot.com/mindset/ for additional 
 * installation and usage instructions.
 *
 * 
 */
 
#include <MindSet.h>

// Why is this here? Apparently there's a bug in the Arduino pre-pre-compiler.
int foo = 1;

#if defined(__AVR_AT90USB1286__)
  // Teensy2.0++ has LED on D6
  #define ERRLED 6
  HardwareSerial btSerial = HardwareSerial();
#elif defined(__AVR_ATmega32U4__)
  // Teensy2.0 has LED on pin 11
  #define ERRLED 11
  HardwareSerial btSerial = HardwareSerial();
#else
  // Assume Arduino (LED on pin 13)
  #define ERRLED 13
  Serial btSerial = Serial();
#endif

#define BAUDRATE 115200

#define REDLED  4
#define BLULED  5

MindSet ms;

void setup() {
  // Set up the serial port on the USB interface
  Serial.begin(BAUDRATE);
  Serial.println("Starting MindSet");

  // COnfigure LED pins
  pinMode(REDLED, OUTPUT);
  pinMode(BLULED, OUTPUT);
  pinMode(ERRLED, OUTPUT);
  
  btSerial.begin(BAUDRATE);
  for(byte i=0; i<255; i++){
    analogWrite(REDLED, i);
    analogWrite(BLULED, i);
    delay(10);
  }
  for(byte i=255; i>50; i--){
    analogWrite(REDLED, i);
    analogWrite(BLULED, i);
    delay(10);
  }
  
  // Attach the callback function to the MindSet packet processor
  ms.attach(dataReady);
  
  Serial.println("Ready to go.");
}

//
// Main program loop. 
//
void loop() {
  // We just feed bytes to the MindSet object as they come in. It will
  // call our callback whenever a complete data packet has been received and parsed.
  if(btSerial.available()) 
    ms.process(btSerial.read());
    
}

// 
// MindSet callback. 
// This function will be called whenever a new data packet is ready.
//
void dataReady() {
  static char str[64];
  sprintf(str,"%5d,%6d,%6d,%6d,%6d,%6d,%6d,%6d,%6d,%4d,%4d\n",
    ms.raw(),ms.delta(),ms.theta(),ms.alpha1(),ms.alpha2(),ms.beta1(),ms.beta2(),ms.gamma1(),ms.gamma2(),ms.meditation(),ms.attention());
  Serial.print(str);
  if(ms.errorRate()<127 && ms.attention()>0)
    analogWrite(REDLED, ms.attention()*2);
  if(ms.errorRate()<127 && ms.meditation()>0)
    analogWrite(BLULED, ms.meditation()*2);
     
  if(ms.errorRate() == 0)
    digitalWrite(ERRLED, HIGH);
  else
    digitalWrite(ERRLED, LOW);     
}

