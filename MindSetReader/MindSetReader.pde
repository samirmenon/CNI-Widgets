/*
 * Arduino Bluetooth Interface with Mindset
 * 
 * This is example code provided by NeuroSky, Inc. and is provided
 * license free.
 *
 * MindSetArduinoReader.pde
 * Modified from Tutorial.pde by Sean M. Montgomery 2010/09
 * Program displays selected brainwave measurement on a bar LED and 
 * the "errorRate" measure of signal quality (zero = good) on an LED
 * and sends selected brainwave measurements and time of data aquisition
 * to a computer via the arduino serial port.
 *
 * See http://www.produceconsumerobot.com/mindset/ for additional 
 * installation and usage instructions.
 *
 * 2010/09/16
 * Selected data is output in csv serial stream
 * added raw data parsing
 * added power data parsing
 * added sampling delay to accomodate slower display programs
 * fixed payloadData too small bug
 * fixed syncing bug
 * 
 */

#if defined(__AVR_AT90USB1286__)
// Teensy2.0++ has LED on D6
#define LED 6
HardwareSerial btSerial = HardwareSerial();
#elif defined(__AVR_ATmega32U4__)
// Teensy2.0 has LED on pin 11
#define LED 11
HardwareSerial btSerial = HardwareSerial();
#else
// Assume Arduino (LED on pin 13)
#define LED 13
Serial btSerial = Serial();
#endif

#define BAUDRATE 115200
#define DEBUGOUTPUT 0

#define BLUESMIRFON 2

#define GREENLED  4
#define BLUELED  5

// neuro data variables
byte errorRate = 200;
byte attention = 0;
byte meditation = 0;
short raw;
unsigned int delta;
unsigned int theta;
unsigned int alpha1;
unsigned int alpha2;
unsigned int beta1;
unsigned int beta2;
unsigned int gamma1;
unsigned int gamma2;

// system variables
unsigned long lastReceivedPacket = micros();
unsigned long totalTime = 0;
boolean newRawData = false;
boolean bigPacket = false;

//////////////////////////
// Microprocessor Setup //
//////////////////////////
void setup() {
  Serial.begin(BAUDRATE);           // USB
  Serial.println("Starting MindSet");

  pinMode(GREENLED, OUTPUT);
  pinMode(BLUELED, OUTPUT);
  
  pinMode(LED, OUTPUT);
  pinMode(BLUESMIRFON, OUTPUT);
  btSerial.begin(BAUDRATE);
  for(byte i=0; i<255; i++){
    analogWrite(GREENLED, i);
    analogWrite(BLUELED, i);
    delay(10);
  }
  for(byte i=255; i>50; i--){
    analogWrite(GREENLED, i);
    analogWrite(BLUELED, i);
    delay(10);
  }
    
  Serial.println("Turning on bluetooth radio...");
  digitalWrite(BLUESMIRFON, HIGH);
}


char str[64];
/////////////
//MAIN LOOP//
/////////////
void loop() {
  ReadData();
  
  if (newRawData) {
      newRawData = false;
      lastReceivedPacket = micros();
      sprintf(str,"%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
              raw,alpha1,alpha2,beta1,beta2,delta,theta,gamma1,gamma2,meditation,attention);
      Serial.print(str);
      analogWrite(GREENLED, attention*2);
      analogWrite(BLUELED, meditation*2);
  }
  if(bigPacket) {
    bigPacket = false;        
    if(errorRate == 0)
      digitalWrite(LED, HIGH);
    else
      digitalWrite(LED, LOW);            
  }
}

////////////////////////////////
// Read data from Serial UART //
////////////////////////////////
byte ReadOneByte() {
  int ByteRead;

  while(!btSerial.available());
  ByteRead = btSerial.read();

#if DEBUGOUTPUT  
  Serial.print((char)ByteRead);   // echo the same byte out the USB serial (for debug purposes)
#endif

  return ByteRead;
}

/////////////////////////////////////////
// Reading data from MindSet bluetooth //
////////////////////////////////////////
void ReadData() {
  static unsigned char payloadData[256]; 
  byte generatedChecksum;
  byte checksum; 
  byte vLength;
  int payloadLength;
  int powerLength = 3; // defined in MindSet Communications Protocol
  int k;

  
  // Look for sync bytes
  if(ReadOneByte() == 170) {
    if(ReadOneByte() == 170) {

      do { payloadLength = ReadOneByte(); }
      while (payloadLength == 170);
      
      if(payloadLength > 170) {    //Payload length can not be greater than 170
         return;
      }
      
      generatedChecksum = 0;        
      for(int i = 0; i < payloadLength; i++) {  
        payloadData[i] = ReadOneByte();            //Read payload into memory
        generatedChecksum += payloadData[i];
      }   

      checksum = ReadOneByte();                      //Read checksum byte from stream      
      generatedChecksum = 255 - generatedChecksum;   //Take one's compliment of generated checksum
      
      if(checksum != generatedChecksum) {  
        // checksum error  
      } else {  

        for(int i = 0; i < payloadLength; i++) {    // Parse the payload
          switch (payloadData[i]) {
          case 2:
            bigPacket = true;            
            i++;            
            errorRate = payloadData[i];         
            break;
          case 4:
            i++;
            attention = payloadData[i];                        
            break;
          case 5:
            i++;
            meditation = payloadData[i];
            break;
          case 0x80: // raw data
            newRawData = true;
            i++;
            vLength = payloadData[i]; 
            raw = 0;
            for (int j=0; j<vLength; j++) {
              raw = raw | ( payloadData[i+vLength-j]<<(8*j) ); // bit-shift little-endian
            }
            i += vLength;
            break;
          case 0x83:  // power data
            i++;
            vLength = payloadData[i]; 
            k = 0;
            
            // parse power data starting at the last byte
            gamma2 = 0; // mid-gamma (41 - 49.75Hz)
            for (int j=0; j<powerLength; j++) {
              gamma2 = gamma2 | ( payloadData[i+vLength-k]<<(8*j) ); // bit-shift little-endian
              k++;
            }
            gamma1 = 0; // low-gamma (31 - 39.75Hz)
            for (int j=0; j<powerLength; j++) {
              gamma1 = gamma1 | ( payloadData[i+vLength-k]<<(8*j) ); // bit-shift little-endian
              k++;
            }
            beta2 = 0; // high-beta (18 - 29.75Hz)
            for (int j=0; j<powerLength; j++) {
              beta2 = beta2 | ( payloadData[i+vLength-k]<<(8*j) ); // bit-shift little-endian
              k++;
            }
            beta1 = 0; // low-beta (13 - 16.75Hz)
            for (int j=0; j<powerLength; j++) {
              beta1 = beta1 | ( payloadData[i+vLength-k]<<(8*j) ); // bit-shift little-endian
              k++;
            }
            alpha2 = 0; // high-alpha (10 - 11.75Hz)
            for (int j=0; j<powerLength; j++) {
              alpha2 = alpha2 | ( payloadData[i+vLength-k]<<(8*j) ); // bit-shift little-endian
              k++;
            }
            alpha1 = 0; // low-alpha (7.5 - 9.25Hz)
            for (int j=0; j<powerLength; j++) {
              alpha1 = alpha1 | ( payloadData[i+vLength-k]<<(8*j) ); // bit-shift little-endian
              k++;
            }
            theta = 0; // theta (3.5 - 6.75Hz)
            for (int j=0; j<powerLength; j++) {
              theta = theta | ( payloadData[i+vLength-k]<<(8*j) ); // bit-shift little-endian
              k++;
            }
            delta = 0; // delta (0.5 - 2.75Hz)
            for (int j=0; j<powerLength; j++) {
              delta = delta | ( payloadData[i+vLength-k]<<(8*j) ); // bit-shift little-endian
              k++;
            }
            
            i += vLength;
            break;          
          default:
            break;
          } // switch
        } // for loop
      } // checksum success
    } // sync 2
  } // sync 1
} // ReadData

/* GetMicrosDelay 
** calculates time difference in microseconds between current time
** and passed time
** accounts for rollover of unsigned long
*/
unsigned long GetMicrosDelay(unsigned long t0) {
  unsigned long dt; // delay time (change)
  
  unsigned long t1 = micros();
  if ( (t1 - t0) < 0 ) { // account for unsigned long rollover
    dt = 4294967295 - t0 + t1 + 1; 
  } else {
    dt = t1 - t0;
  }
  return dt;
}

