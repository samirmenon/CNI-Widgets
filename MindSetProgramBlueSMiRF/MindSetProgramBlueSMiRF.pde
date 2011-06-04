////////////////////////////////////////////////////////////////////////
// Arduino BlueSMiRF initial baudrate changer
// 
// This is example code provided by NeuroSky, Inc. and is provided
// license free.
//
// * Connect BlueSMiRF RX and TX to Serial on Arduino board (pins 0 & 1)
// * Connect BlueSMiRF VCC to VCC (+5v) and GND to GND. BlueSMiRF RTS should
// * be connected directly to the BlueSMiRF CTS line with a jumper.
// * 
////////////////////////////////////////////////////////////////////////

/**** CleanProgramBlueSMiRF.pde ****
** Modified by Sean M Montgomery 2010/12
** Programs a BlueSMiRF module with the specified MAC address
** for use with the NeuroSky MindSet. 
** 
** Modified to do an automatic factory reset to avoid needing
** to run the program twice.
************************************/

/**** SET YOUR MAC ADDRESS HERE ****/

char mac[13] = "0013EF00315C";

/***********************************/


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

#define BLUESMIRFON 2

//#define FACTORYRESETBAUD 57600
#define FACTORYRESETBAUD 115200
#define DEFAULTBAUD 115200

char str[3];
char passkey[5] = "0000";

boolean success = false;

void setup() 
{ 
  //Initialize pins
  pinMode(LED, OUTPUT);
  digitalWrite(LED,HIGH);
  delay(1000);
  digitalWrite(LED,LOW);
  delay(1000);
  pinMode(BLUESMIRFON, OUTPUT);
  
  // First reset to factory defaults
  while (!success) {
    RunBlueSmirfSetup(true);
  }
  success = false;
  // Then set up with the correct mac address
  RunBlueSmirfSetup(false);
} 

void loop() {   
  if(success) {
    digitalWrite(LED,LOW);
    delay(1000);
    digitalWrite(LED,HIGH);
    delay(1000);
  }else{
    flash(8);
  }
}

void flash(byte n){
    digitalWrite(LED,LOW);
    delay(800);
    for(byte i=0; i<n; i++){
      digitalWrite(LED,HIGH);
      delay(200);
      digitalWrite(LED,LOW);
      delay(200);
    }
}

void RunBlueSmirfSetup(boolean factoryReset) {

  //Initialize serial ports
  if (factoryReset) {
    btSerial.begin(FACTORYRESETBAUD);   
  } else {
    btSerial.begin(DEFAULTBAUD);   
  }   

  digitalWrite(BLUESMIRFON, LOW);
  delay(2000);
  digitalWrite(BLUESMIRFON, HIGH);  
  delay(2000);			        //Wait for BlueSMIRF to turn on
  
  btSerial.print('$');			//Send command to put BlueSMIRF into programming mode
  btSerial.print('$');
  btSerial.print('$');
  
  delay(100);
  btSerial.flush();
  
   //Reset the module
  if (factoryReset) {
    btSerial.print('S');
    btSerial.print('F');
    btSerial.print(',');
    btSerial.print('1');
    btSerial.print('\r');  
    
    while(btSerial.available() < 3) ;
    str[0] = (char)btSerial.read();
    str[1] = (char)btSerial.read();
    str[2] = (char)btSerial.read();  
    if(str[0] == 'A' && str[1] == 'O' && str[2] == 'K') {
      success = true;
    } else {
      success = false;
      flash(1);
    }
    delay(100);
    btSerial.flush();
  } else {
    //Set the baudrate
    btSerial.print('S');
    btSerial.print('U');
    btSerial.print(',');
    btSerial.print('5');
    btSerial.print('7');
    btSerial.print('\r');  
    
    while(btSerial.available() < 3);
    str[0] = (char)btSerial.read();
    str[1] = (char)btSerial.read();
    str[2] = (char)btSerial.read();  
    if(str[0] == 'A' && str[1] == 'O' && str[2] == 'K') {
      success = true;
    } else {
      success = false;
      flash(2);
    }
    delay(100);
    btSerial.flush();
    
    //Set the remote MAC address
    btSerial.print('S');
    btSerial.print('R');
    btSerial.print(',');
    for(int i = 0; i < 12; i++) {
      btSerial.print(mac[i]);
    }
    btSerial.print('\r');  
    
    while(btSerial.available() < 3);
    str[0] = (char)btSerial.read();
    str[1] = (char)btSerial.read();
    str[2] = (char)btSerial.read();  
    if(str[0] == 'A' && str[1] == 'O' && str[2] == 'K') {
      success = true;
    } else {
      success = false;
      flash(3);
    }
    delay(100);
    btSerial.flush();
    
    //Set the passkey
    btSerial.print('S');
    btSerial.print('P');
    btSerial.print(',');
    for(int i = 0; i < 4; i++) {
      btSerial.print(passkey[i]);
    }
    btSerial.print('\r');  
    
    while(btSerial.available() < 3);
    str[0] = (char)btSerial.read();
    str[1] = (char)btSerial.read();
    str[2] = (char)btSerial.read();  
    if(str[0] == 'A' && str[1] == 'O' && str[2] == 'K') {
      success = true;
    } else {
      success = false;
      flash(4);
    }
    delay(100);
    btSerial.flush(); 
    
    //Set the BlueSMiRF mode
    btSerial.print('S');
    btSerial.print('M');
    btSerial.print(',');
    btSerial.print('3');
    btSerial.print('\r');
    
    while(btSerial.available() < 3);
    str[0] = (char)btSerial.read();
    str[1] = (char)btSerial.read();
    str[2] = (char)btSerial.read();  
    if(str[0] == 'A' && str[1] == 'O' && str[2] == 'K') {
      success = true;
    } else {
      success = false;
      flash(5);
    }
    delay(100);
    btSerial.flush();
    
    delay(100);
    //Exit command mode
  }
  btSerial.print('-');
  btSerial.print('-');
  btSerial.print('-');
  btSerial.print('\r');

  //delay(100);
  //btSerial.flush();
  //delay(100);
  //btSerial.end();
  //digitalWrite(BLUESMIRFON, LOW);
}


