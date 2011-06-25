/*
 * MindSetReader.pde
 *
 * Arduino Interface to the Neurosky MindSet EEG headset.
 *
 * Displays MindSet "meditation" and "attention" eSense readings as 
 * the brightness of two LEDs (e.g., the blue and red elements of a 
 * tri-color LED). The "errorRate" measure of signal quality (0 = good)
 * is shown on the built-in LED (LED on means data are good).
 * It also sends selected all the MindSet data measurements 
 * to a host computer via the Teensy USB serial port.
 *
 * 2011.06.23 Bob DOugherty <bobd@stanford.edu> wrote it.
 * 
 */
 
#include <MindSet.h>

// Sample interval is ~1.95ms (1000/512)
#define FLICKTICS 67
const byte flickPeriod = FLICKTICS;
byte flickCount;
const unsigned long flickDutyMs = 10;
unsigned long flickDutyStartMs;
byte repCount;
long buffer[FLICKTICS];


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

#define REDLED 9
#define BLULED 4
#define GRNLED 5

MindSet ms;

void setup() {
  // Set up the serial port on the USB interface
  Serial.begin(BAUDRATE);
  Serial.println("Starting MindSet");

  // COnfigure LED pins
  pinMode(REDLED, OUTPUT);
  pinMode(GRNLED, OUTPUT);
  pinMode(BLULED, OUTPUT);
  pinMode(ERRLED, OUTPUT);
  
  btSerial.begin(BAUDRATE);
  for(int i=0; i<255; i+=4){
    analogWrite(REDLED, i);
    analogWrite(GRNLED, i);
    analogWrite(BLULED, i);
    delay(10);
  }
  for(int i=255; i>=0; i-=4){
    analogWrite(REDLED, i);
    analogWrite(GRNLED, i);
    analogWrite(BLULED, i);
    delay(10);
  }
  analogWrite(REDLED, 0);
  analogWrite(GRNLED, 0);
  analogWrite(BLULED, 0);
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
  
  // Turn off the LED if the counter has expired
  if((millis()-flickDutyStartMs)>flickDutyMs){
    analogWrite(REDLED,0);
    analogWrite(GRNLED,0);
    analogWrite(BLULED,0);
  }
  
  if(repCount>=128){
    for(byte i=0; i<flickPeriod; i++){
      // bit-shift division, with rounding:
      Serial.print((buffer[i]+32)>>5);
      Serial.print(',');
      buffer[i] = 0;
    }
    Serial.println();
    repCount = 0;
  }
}

// 
// MindSet callback. 
// This function will be called whenever a new data packet is ready.
//
void dataReady() {
  //static char str[64];
  //if(ms.errorRate()<127 && ms.attention()>0)
  //  analogWrite(REDLED, ms.attention()*2);
  //if(ms.errorRate()<127 && ms.meditation()>0)
  //  analogWrite(BLULED, ms.meditation()*2);
     
  if(ms.errorRate() == 0)
    digitalWrite(ERRLED, HIGH);
  else
    digitalWrite(ERRLED, LOW);
  
  buffer[flickCount] += ms.raw();
  
  flickCount++;
  if(flickCount>=flickPeriod){
    flickCount = 0;
    repCount++;
  }
  if(flickCount==0){
    analogWrite(REDLED, 255);
    analogWrite(GRNLED, 255);
    analogWrite(BLULED, 255);
    flickDutyStartMs = millis();
  }
}

