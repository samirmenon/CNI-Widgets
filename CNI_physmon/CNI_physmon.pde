/*
 * Sketch to read serial data from the GE MR750 physiological monitoring 
 * port in the PGR cabinet. 
 *
 * The data come in 12-byte (96-bit) packets delivered at 115200 bps. A packet
 * comes every 5ms, with silence between packets. This produces data bursts
 * of 96bits / 115.2bits/ms = .8333ms with 5 - .8333 = 4.1667ms of silence.
 * (This timing pattern was confirmed with a scope.) So we can detect the start 
 * of a data packet by waiting for the silent period before the data arrives.
 * Here we only care about the PPG value. But the code could be easily modified 
 * to do something with the other physiological readings.
 *
 * To detect a pulse, we compute a running mean and standard deviation and
 * compute a z-score for each new data point. When several consecutive data
 * point z-scores are above threshold, we signal that a pulse was detected. 
 * With a Teensy 2++ (which has 8k SRAM), we can maintain a buffer size of
 * 1024, which is 1024 * 0.005 = 5.12 seconds. This seems to be enough to give
 * a very stable pulse detection.
 *
 * The PPG value sum and its sum-of-squares are maintained in long and unsigned 
 * long (respectively) buffers. So, if your PPG data values are high and your 
 * buffer big enough, you might overflow these containers. With a 1024 buffer, 
 * you can safely capture the full range of int16 values in the data sum. 
 * However, the sum-of-squares will limit you to sqrt(2^32/1024) = 2048 as your 
 * average deviation from the mean. If you expect deviations higher than this,
 * then you will want to decrease the buffer size or maybe try a larger container
 * for the sum-of-squares.
 *
 * To Do:
 *  - measure the data packet interval
 *  - allow all parameters to be adjusted
 *  - allow parameters to be saved to eeprom, and reloaded upon reboot
 *
 * Copyright 2011 Robert F. Dougherty (bobd@stanford.edu)
 */
 
/*

Python code to generate simulated data:

#!/usr/bin/env python
import serial, time, array, random

ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
data = array.array('h',[0, 0, 0, 0, 0, 0])
for i in range(1,10000):
  data[0] = i
  data[4] = random.randint(1, 200)
  # "Pulse" every second
  if i%200>=0 and i%200<20:
    data[4] = data[4]+100
  ser.write(data.tostring())   # write a data packet
  #data.tostring()
  time.sleep(0.004)

ser.close()

*/

#include <Flash.h>
#include <Messenger.h>
#include <SSD1306.h>

#define VERSION "1.0"

#define SQUARE(a) ((a)*(a))

// Everything is integer math, so we scale the z-scores to allow finer
// precision. A 10 here will allow tenths precision for z-scores.
#define ZSCALE 10

// The data interval is not measured, but assumed. Maybe we could measure it?
// Perhaps even just measure it once at the beginning to confirm the value set here?
#define DATA_INTERVAL_MILLISEC 5
// instantaneous bps = 1000/pulseIntervalMillisec,
// bpm = 60000/pulseIntervalMillisec = (60000/DATA_INTERVAL_MILLISEC)/pulseIntervalTics
#define BPM_SCALE (60000/DATA_INTERVAL_MILLISEC)
// The minimal silent period that we will use to detect the start of a new packet:
#define DATA_SILENCE_MILLISEC 1

#define DEFAULT_OUT_PULSE_MSEC 5
#define DEFAULT_PHYSIO_OUT_STATE 0
#define DEFAULT_ZSCALE (1.5*ZSCALE);
#define DEFAULT_NUM_CONSEC_Z 3
#define DEFAULT_REFRACTORY_TICS (300/DATA_INTERVAL_MILLISEC)
#define DEFAULT_REFRESH_INTERVAL 3
#define DEFAULT_IN_STATE 0
#define DEFAULT_IN_EDGE FALLING

// We need to be careful that our two buffers will fit in available SRAM. We 
// also want them to be a power of two so that we can use bit-shifting for division. 
// If the update rate is 5ms, then 256 samples will give us a temporal window
// of ~1.3 sec, 512 = 2.6 sec, and 1024 just over 5 sec.
#if defined(__AVR_AT90USB1286__)
  // the teensy 2.0++ (1286) has 8092 bytes of SRAM
  #define BUFF_SIZE_BITS 10
  // Teensy2.0++ has LED on D6
  #define PULSE_OUT_PIN 4
  #define TRIGGER_OUT_PIN 5
  #define TRIGGER_IN_PIN 18  // INT6 (pin E6)
  #define TRIGGER_IN_INT 6
  #define LED_RED_PIN 1
  #define LED_GRN_PIN 0
  #define LED_BLU_PIN 27
  // uart rx is D2, tx is D3
  // Pin definitions for the OLED graphical display
  #define OLED_DC 24
  #define OLED_RESET 25
  #define OLED_SS 20
  #define OLED_CLK 21
  #define OLED_MOSI 22
#elif defined(__AVR_ATmega32U4__)
  // teensy 2.0 (mega32) has 2560 bytes of SRAM. Enough for 2048 in buffers.
  #define BUFF_SIZE_BITS 9
  // Teensy2.0 has LED on pin 11
  #define PULSE_OUT_PIN 11
  #define TRIGGER_OUT_PIN 10
  #define TRIGGER_IN_PIN ?
  #define TRIGGER_IN_INT ?
  #define LED_RED_PIN 12
  #define LED_GRN_PIN 14
  #define LED_BLU_PIN 15
  // uart rx is D2, tx is D3
  // Pin definitions for the OLED graphical display
  #define OLED_DC 11
  #define OLED_RESET 13
  #define OLED_SS 0
  #define OLED_CLK 1
  #define OLED_MOSI 2
#else
  #define BUFF_SIZE_BITS 8
  #define PULSE_OUT_PIN 13
  #define TRIGGER_OUT_PIN 14
  #define TRIGGER_IN_PIN ?
  #define TRIGGER_IN_INT ?
  #define LED_RED_PIN 5
  #define LED_GRN_PIN 6
  #define LED_BLU_PIN 7
  // Pin definitions for the OLED graphical display
  #define OLED_DC 11
  #define OLED_RESET 13
  #define OLED_SS 12
  #define OLED_CLK 10
  #define OLED_MOSI 9
#endif
#define BUFF_SIZE (1<<BUFF_SIZE_BITS)

SSD1306 oled(OLED_MOSI, OLED_CLK, OLED_DC, OLED_RESET, OLED_SS);

// GE Physio data come in as 6 int16s in little endian format.
// We use a union so we can load each byte and then access the
// int16 data.
typedef union dataPacket{
  byte byteArray[12];
  struct{
    unsigned int tic;
    int ecg1;
    int ecg2;
    int resp;
    int ppg;
    int unknown;
  };
}dataPacket;

dataPacket g_data;

int g_dataBuffer[BUFF_SIZE];
int g_diffBuffer[BUFF_SIZE];
long g_dataSum;
unsigned long g_sumSquares;
int g_thresh;
unsigned int g_curBuffIndex;
unsigned int g_lastPulseTic;
unsigned int g_refractoryTics;
byte g_displayUpdateInterval;
byte g_numConsecutiveZscores;

// The approximate duration of the output pulse, in milliseconds.
// Note that we don't account for delays in the code, so it will 
// always be a bit longer than this.
unsigned int g_outPinDuration = DEFAULT_OUT_PULSE_MSEC;
byte g_physioOutFlag = DEFAULT_PHYSIO_OUT_STATE;
byte g_inPulseEdge = DEFAULT_IN_EDGE;

// Flags to remember if the output pins are currently turned on
byte g_triggerPinOn, g_pulsePinOn;
// We also need to remember when they were turned on
unsigned long g_triggerOutStart, g_pulseOutStart;

// Object to access the hardware serial port:
HardwareSerial g_Uart = HardwareSerial();

// Instantiate Messenger object used for serial port communication.
Messenger g_message = Messenger(',','[',']');

// Create the Message callback function. This function is called whenever a complete 
// message is received on the serial port.
void messageReady() {
  int val[16];
  byte i;
  char command, paramName;
  if(g_message.available()) {
    // get the command byte
    command = g_message.readChar();
    switch(command) {
    
    case '?': // display help text
      Serial << F("CNI PhysMon: Monitor GE physio data stream on UART Rx pin.\n");
      Serial << F("Pulse pulses are output on pin ") << (int)PULSE_OUT_PIN << F(".\n");
      Serial << F("Trigger pulses are output on pin ") << (int)TRIGGER_OUT_PIN << F(".\n");
      Serial << F("\nCommands:\n");
      Serial << F("[s,X,N] Set parameter X to value N. (Omit N to echo the current value.)\n");
      Serial << F("  [s,d,N] Set the display update interval.\n");     
      Serial << F("  [s,n,N] Set the number of consecutive z-scores needed to pulse.\n");
      Serial << F("  [s,o,N] Set the output pulse duration to N milliseconds.\n");
      Serial << F("  [s,p] Enable scan timing pulse detection.\n");
      Serial << F("  [s,r,N] Set the pulse refractory period to N milliseconds.\n");
      Serial << F("  [s,z,N] Set the z-score threshold, scaled by ") << ZSCALE << F(".\n");
      Serial << F("[p,F] Set physio output format flag. Valid values:\n");
      Serial << F("         0 for no physio data\n");
      Serial << F("         1 for text: tic,resp,ppg,z-score,pulse bit\n");
      Serial << F("         2 for binary tic(uint16) resp(int16) ppg(int16) z-score(uint8) pulse(uint8)\n");
      Serial << F("         3 for GE binary format\n");
      Serial << F("[r]   Reset default state.\n\n");
      Serial << F("[t]   Send a trigger pulse.\n");
      break;
    
    case 's': // Set parameter value
      if(g_message.available())
        paramName = g_message.readChar();
      else
        Serial << F("ERROR: Set parameter requires a parameter name.\n");
      while(g_message.available()) val[i++] = g_message.readInt();
      switch(paramName) {
        
        case 'd':
          if(i==1)
            g_displayUpdateInterval = val[0];
          else
            Serial << F("Display update interval is set to ") << (int)g_displayUpdateInterval << F(" tics.\n");
        break;

        case 'n':
          if(i==1)
            g_numConsecutiveZscores = val[0];
          else
            Serial << F("Num consecutive z-scores is set to ") << (int)g_numConsecutiveZscores << F(".\n");
        break;
        
        case 'o': // Set out-pulse duration (msec)
          if(i==1)
            g_outPinDuration = val[0];
          else
            Serial << F("Output pulse duration is set to ") << g_outPinDuration << F(" msec.\n");
          break;

        case 'p': // enable/disable input pulse detection
          if(i==1){
            if(val[0]==0 || val[0]==1)
              setInTriggerState(val[0]);
            else
              Serial << F("ERROR: in trigger state = [0|1].");
          }else
            Serial << F("ERROR: Set in trigger state requires one param.\n");
        break;
        
        case 'r': // Set pulse refractory period (msec)
          if(i==1)
            g_refractoryTics = val[0]/DATA_INTERVAL_MILLISEC;
          else
            Serial << F("Pulse refractory period is set to ") << g_refractoryTics*DATA_INTERVAL_MILLISEC 
                   << F(" msec (") << g_refractoryTics << F(" tics).\n");
          break;

        case 'z': // Set z-score threshold
          if(i==1)
            g_thresh = val[0];
          else
            Serial << F("Z-score threshold is set to ") << g_thresh << F(". (ZSCALE is ") << ZSCALE << F(")\n");
        break;

        default:
          Serial << F("ERROR: Unknown parameter name '") << paramName << F("'.\n\n");
        } // end switch paramName
      break;
    
    case 'p': // enable/disable physio output
      while(g_message.available()) val[i++] = g_message.readInt();
      if(i>1){
        Serial << F("ERROR: Set physio output state requires no more than one param.\n");
      }else if(i==1){
        if(val[0]<0||val[0]>3)
          Serial << F("ERROR: Physio output state = [0|1|2|3].\n");
        else
          g_physioOutFlag = val[0];
      }else{
        Serial << F("Physio state is set to ") << (int)g_physioOutFlag << F(".\n");
      }
      break;

    case 'r': // reset
      g_physioOutFlag = DEFAULT_PHYSIO_OUT_STATE;
      g_outPinDuration = DEFAULT_OUT_PULSE_MSEC;
      g_thresh = DEFAULT_ZSCALE;
      g_numConsecutiveZscores = DEFAULT_NUM_CONSEC_Z;
      g_refractoryTics = DEFAULT_REFRACTORY_TICS;
      g_displayUpdateInterval = DEFAULT_REFRESH_INTERVAL;
      digitalWrite(TRIGGER_OUT_PIN, LOW);
      break;
      
    case 't': // force output trigger
      // First force the pin low, in case it was already on. This will ensure that
      // we get a change on the pin no matter what state we were in.
      if(g_triggerPinOn) digitalWrite(TRIGGER_OUT_PIN, LOW);
      triggerOut();
      break;

    default:
      Serial << F("ERROR: Unknown command '") << command << F("'.\n\n");
    } // end switch command
  } // end while
}


void setup(){
  // TO DO: load these from eeprom
  g_thresh = DEFAULT_ZSCALE;
  g_numConsecutiveZscores = DEFAULT_NUM_CONSEC_Z;
  g_refractoryTics = DEFAULT_REFRACTORY_TICS;
  g_displayUpdateInterval = DEFAULT_REFRESH_INTERVAL;
  
  g_curBuffIndex = 0;
  
  // initialize output pins.
  pinMode(PULSE_OUT_PIN, OUTPUT);
  digitalWrite(PULSE_OUT_PIN, LOW);
  pinMode(TRIGGER_OUT_PIN, OUTPUT);
  digitalWrite(TRIGGER_OUT_PIN, LOW);
  
  // Initialize input pin
  // This probably isn't necessary- external interrupts work even in OUTPUT mode. 
  pinMode(TRIGGER_IN_PIN, INPUT);
  setInTriggerState(DEFAULT_IN_STATE);

  // Initialize the OLED display
  // Configure it to generate the high voltage from 3.3v
  oled.ssd1306_init(SSD1306_SWITCHCAPVCC);
  oled.display(); // show splashscreen
  for(byte i=0; i<255; i++){
    analogWrite(LED_RED_PIN, i);
    analogWrite(LED_GRN_PIN, i);
    analogWrite(LED_BLU_PIN, i);
    delay(2);
  }
  for(byte i=255; i>0; i--){
    analogWrite(LED_RED_PIN, i);
    analogWrite(LED_GRN_PIN, i);
    analogWrite(LED_BLU_PIN, i);
    delay(2);
  }

  g_Uart.begin(115200);
    
  Serial.begin(115200);
  Serial << F("*********************************************************\n");
  Serial << F("* CNI Physmon version ") << VERSION << F("\n");
  Serial << F("* Copyright 2011 Bob Dougherty <bobd@stanford.edu>\n");
  Serial << F("* http://cniweb.stanford.edu/wiki/CNI_widgets\n");
  Serial << F("*********************************************************\n\n");
  Serial << F("Initialized with ") << BUFF_SIZE << F(" element buffers. (") << freeRam() << F(" SRAM bytes free).\n\n");

  // Attach the callback function to the Messenger
  g_message.attach(messageReady);
  Serial << F("CNI PhysMon Ready. Send the ? command ([?]) for help.\n\n");
  analogWrite(LED_RED_PIN, 0);
  analogWrite(LED_GRN_PIN, 0);
  analogWrite(LED_BLU_PIN, 0);
}

void loop(){
  // Need a buffer for the line of text that we show.
  static char stringBuffer[SSD1306_LCDLINEWIDTH+1];

  // We only fire an output when we get N consecutive zscores above threshold.
  static byte curNumZscores;
  static byte bpm;
 
  // ticDiff is the increment in the tic count between this data packet and the previous one.
  // This function sets the data in the global g_data.
  // MAYBE PASS IN THE GLOBAL FOR CLARITY? OR JUST INLINE THE CODE?
  byte ticDiff = getDataPacket();
  
  if(ticDiff==255){
    // This means that we had to resync. We don't process anything, just show status.
    snprintf(stringBuffer,SSD1306_LCDLINEWIDTH+1,"* resyncing packets...");
    refreshDisplay(0, stringBuffer, 255, 0);
  }else if(ticDiff>0){
    // Got something, now process it.
    unsigned int pulseIntervalTics = g_data.tic - g_lastPulseTic;
    int zscore = processDataPacket();
    if(zscore>g_thresh && pulseIntervalTics>g_refractoryTics)
      curNumZscores++;
    
    boolean pulseNow = false;
    if((curNumZscores>g_numConsecutiveZscores)){
      // PULSE DETECTED!
      bpm = pulseOut(pulseIntervalTics);
      snprintf(stringBuffer, SSD1306_LCDLINEWIDTH+1, "%d z*%d=%02d bpm=%03d     ",ticDiff,ZSCALE,zscore,bpm);
      curNumZscores = 0;
      pulseNow = true;
    }

    if(g_physioOutFlag==1){
      // 1 for text: tic,resp,ppg,z-score,pulse bit
      Serial << g_data.tic << F(",") << g_data.resp << F(",") << g_data.ppg << F(",") 
             << zscore << F(",") << (int)pulseNow << F("\n");
    }else if(g_physioOutFlag==2){
      // 2 for binary tic(uint16) resp(int16) ppg(int16) z-score(uint8) pulse(uint8)
      // *** IMPLEMENT ME ***
    }else if(g_physioOutFlag==3){
      // 3 for GE binary format (just echo the byte array)
      Serial << (char *)g_data.byteArray;
    }
    refreshDisplay(zscore, stringBuffer, ticDiff, pulseNow);
  }
  
  updateOutPins();
  
  // Handle Messenger's callback:
  if(Serial.available())  g_message.process(Serial.read());
}

// Get the incomming data packet.
// Returns 0 if no data were ready, 255 if we received corrupt data and had to resync,
// or the actual tic difference between this packet and the previous packet (should
// be 1, but might be 2 if our main loop is running slow).
byte getDataPacket(){
  // Keep track of the tic value of the last data packet that we received.
  static unsigned int prevTic;
  static boolean resynced;
  unsigned int ticDiff;
  
  // If we don't have our bytes, we return immediately.
  if(g_Uart.available()<12){
    ticDiff = 0;
  }else{
    // Read the 12 bytes
    for(byte i=0; i<12; i++)
      g_data.byteArray[i] = g_Uart.read();
    
    // Check for valid data by comparing current tic to last tic (should increment by 1).
    ticDiff = g_data.tic - prevTic;
    if((ticDiff<1 || ticDiff>2) && !resynced){
      // To avoid getting stuck after a resync, we only check ticDiff for a valid increment if
      // the previous cycle was NOT a resync.
      ticDiff = 255;
      resync();
      resynced = true;
    }else{
      resynced = false; 
    }
  }
  prevTic = g_data.tic;
  return(ticDiff);
}

// Resync to the incomming serial stream by waiting for 
// the dead time between data packets.
// 
// flush to clear input buffer
// loop, timing the interval between bytes until it exceeds the threshold.
// Note that this function will block until it hits a silent period in the serial stream.
void resync(){
  // Wait for the next silent period:
  do{
    g_Uart.flush();
    delay(DATA_SILENCE_MILLISEC);
  }while(g_Uart.available()>0);
}

// Process the current data packet. This involves loading up the buffers and
// computing the z-score for the current data point.
int processDataPacket(){
  int curDataVal = g_data.ppg;
  // Remove the oldest value from the sum:
  g_dataSum -= g_dataBuffer[g_curBuffIndex];
  // And store the current (newest) value in that buffer position:
  g_dataBuffer[g_curBuffIndex] = curDataVal;
  // And update the sum with the newest value:
  g_dataSum += g_dataBuffer[g_curBuffIndex];
  // Now compute the mean using a bit-shift to do integer division:
  int dataMean = g_dataSum>>BUFF_SIZE_BITS;
  // Now we can do the diff:
  // *** TO DO: check for overflow!
  g_sumSquares -= g_diffBuffer[g_curBuffIndex];
  g_diffBuffer[g_curBuffIndex] = SQUARE(curDataVal - dataMean);
  g_sumSquares += g_diffBuffer[g_curBuffIndex];
  // Now check to see if the output should be pulsed:
  int diff = (g_data.ppg-dataMean)*ZSCALE;
  int stdev = isqrt(g_sumSquares>>BUFF_SIZE_BITS);
  int zscore = diff/stdev;
  // Limit to 9 SDs
  if(zscore>9*ZSCALE) zscore = 9*ZSCALE;
  g_curBuffIndex++;
  if(g_curBuffIndex==BUFF_SIZE)
    g_curBuffIndex = 0;
  return(zscore);
}

// Refresh the display, if we are due for a refresh. Otherwise, return immediately.
void refreshDisplay(int zscore, char *stringBuffer, byte ticDiff, byte pulseOut){
  // We need to keep track of the current x,y data value. 0,0 is at the
  // upper left, so we want to flip Y and thus initialize by the height,
  // which is the bottom of the display.
  static byte curX;
  static byte curY = SSD1306_LCDHEIGHT;
  // The current data frame. Used to know when we are due for a display refresh.
  static byte curRefreshFrame;
  
  curRefreshFrame += ticDiff;

  // CurY will contain an average z-score across the g_displayUpdateInterval number of frames.
  curY -= zscore/g_displayUpdateInterval;
  if(pulseOut)
    oled.drawline(curX, 10, curX, 14, WHITE);
  if(curRefreshFrame>g_displayUpdateInterval){ 
    // Update the running plot
    // Clear the graph just in front of the current x position:
    oled.fillrect(curX+1, 8, 16, SSD1306_LCDHEIGHT-8, BLACK);
    // Clip the y-values to the plot area (SSD1306_LCDHEIGHT-1 at the bottom to 16 at the top)
    if(curY>SSD1306_LCDHEIGHT-1) curY = SSD1306_LCDHEIGHT-1;
    else if(curY<16) curY = 16;
    // Plot the pixel for the current data point
    oled.setpixel(curX, curY, WHITE);
    // Update the blue LED
    analogWrite(LED_BLU_PIN, curY<SSD1306_LCDHEIGHT ? (SSD1306_LCDHEIGHT-1-curY)*2 : 0);
    // Reset the y-position accumulator:
    curY = SSD1306_LCDHEIGHT-1;
    // Increment x, and check for wrap-around
    curX++;
    if(curX>=SSD1306_LCDWIDTH){
      curX = 0;
      // If we have wrapped around, we need to clear the first row.
      oled.drawrect(0, 8, 1, SSD1306_LCDHEIGHT-8, BLACK);
    }
    // Draw the status string at the top:
    oled.drawstring(0, 0, stringBuffer);
    // Finished drawing the the buffer; copy it to the device:
    oled.display();
    curRefreshFrame = 0;
  }
}

void updateOutPins(){
  // Turn off the output pins after the requested duration.
  unsigned long curMillis = millis();
  
  if(g_triggerPinOn){
    // Detect and correct counter wrap-around:
    if(curMillis<g_triggerOutStart) g_triggerOutStart += 4294967295UL;
    // Pull it low if the duration has passed
    if(curMillis-g_triggerOutStart > g_outPinDuration){
      digitalWrite(TRIGGER_OUT_PIN, LOW);
      analogWrite(LED_RED_PIN, 0);
    }
  }
  
  if(g_pulsePinOn){
    // Detect and correct counter wrap-around:
    if(curMillis<g_pulseOutStart) g_pulseOutStart += 4294967295UL;
    // Pull it low if the duration has passed
    if(curMillis-g_pulseOutStart > g_outPinDuration){
      digitalWrite(PULSE_OUT_PIN, LOW);
      analogWrite(LED_GRN_PIN, 0);
    }
  }
}

void triggerOut(){
    digitalWrite(TRIGGER_OUT_PIN, HIGH);
    analogWrite(LED_RED_PIN, 255);
    g_triggerPinOn = true;
    g_triggerOutStart = millis();
}

byte pulseOut(unsigned int pulseIntervalTics){
  digitalWrite(PULSE_OUT_PIN, HIGH);
  analogWrite(LED_GRN_PIN, 80);
  g_pulsePinOn = true;
  g_pulseOutStart = millis();
  g_lastPulseTic = g_data.tic;
  return(BPM_SCALE / pulseIntervalTics);
}

void setInTriggerState(byte state){
  // Turn on the internal pull-up resistor if we want to detect falling edges.
  if(g_inPulseEdge==FALLING)
    digitalWrite(TRIGGER_IN_PIN, HIGH);
  else
    digitalWrite(TRIGGER_IN_PIN, LOW);
  // Attach or detach the interrupt
  if(state)
    attachInterrupt(TRIGGER_IN_INT, triggerIn, g_inPulseEdge);
  else
    detachInterrupt(TRIGGER_IN_INT);
}

// The following is an interrupt routine that is run each time the 
// a pulse is detected on the trigger input pin.
void triggerIn(){
  // *** WORK HERE ***
  // Do we want to just spit out chars with each pulse, or somehow incorporate this info in the
  // physio data str
  Serial << F("p");
}



/*
 * Integer square-root approximation, by Jim Ulery. 
 * from http://www.azillionmonkeys.com/qed/sqroot.html
 */
unsigned int isqrt(unsigned long val) {
  unsigned long temp, g=0, b = 0x8000, bshft = 15;
  do {
    if(val >= (temp = (((g << 1) + b)<<bshft--))) {
      g += b;
      val -= temp;
    }
  } while(b >>= 1);
  return g;
}


extern unsigned int __data_start;
extern unsigned int __data_end;
extern unsigned int __bss_start;
extern unsigned int __bss_end;
extern unsigned int __heap_start;
extern void *__brkval;

unsigned int freeRam(){
  int free_memory;
  if((int)__brkval == 0)
     free_memory = ((int)&free_memory) - ((int)&__bss_end);
  else
    free_memory = ((int)&free_memory) - ((int)__brkval);
  return free_memory;
}

