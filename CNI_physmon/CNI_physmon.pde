/*
 * Sketch to read serial data from the GE MR750 physiological monitoring 
 * port in the PGR cabinet. 
 *
 * The data come in 12-byte (96-bit) packets delivered at 115200 bps in bursts
 * of about .8ms (96bits / 115.2bits/ms = .8333ms) with a burst coming every 5ms.
 * (This timing pattern was confirmed with a scope.) So we can detect the start 
 * of a data packet by waiting for 4ms of silence (signal high) on the serial 
 * line. There might be a way to do this more cleverly, but the approach taken 
 * here is to send the serial signal into Rx and pin 2, which will be used to
 * trigger our data-read interrupt.
 *
 * Beware of overflow! The data sum and sum-of-squares are maintained in long 
 * and unsigned long (respectively) vars. So, if your data values are high,
 * then the sum can overflow its container with larger buffer sizes. E.g.,
 * with a 1024 buffer and the full int16 range, you can safely capture the 
 * full range of int16 values. However, the sum-of-squares will limit you to
 * sqrt(2^32/1024) = 2048 as your average deviation from the mean. 
 * (CHEKCK THIS)
 * 
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
#include <SSD1306.h>

#define VERSION "0.1"

#define SQUARE(a) ((a)*(a))

// Everything is integer math, so we scale the z-scores to allow finer
// precision. A 10 here will allow tenths precision for z-scores.
#define ZSCALE 10

#define DATA_INTERVAL_MILLISEC 5
#define DATA_SILENCE_MILLISEC 2

// We need to be careful that our two buffers will fit. We also want 
// them to be a power of two so that we can use bit-shifting for division. 
// If the update rate is 5ms, then 256 samples will give us a temporal 
// window of ~1.3 sec, 512 = 2.6 sec, and 1024 just over 5 sec.
#if defined(__AVR_AT90USB1286__)
  // the teensy 2.0++ (1286) has 8092 bytes of SRAM
  #define BUFF_SIZE_BITS 10
  // Teensy2.0++ has LED on D6
  #define LEDPIN 6
  // uart rx is D2, tx is D3
  // Pin definitions for the OLED graphical display
  #define OLED_DC 11
  #define OLED_RESET 13
  #define OLED_SS 20
  #define OLED_CLK 21
  #define OLED_MOSI 22
#elif defined(__AVR_ATmega32U4__)
  // teensy 2.0 (mega32) has 2560 bytes of SRAM. Enough for 2048 in buffers.
  #define BUFF_SIZE_BITS 9
  // Teensy2.0 has LED on pin 11
  #define LEDPIN 11
  // uart rx is D2, tx is D3
    // Pin definitions for the OLED graphical display
  #define OLED_DC 11
  #define OLED_RESET 13
  #define OLED_SS 0
  #define OLED_CLK 1
  #define OLED_MOSI 2
#else
  #define BUFF_SIZE_BITS 8
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
unsigned long g_diffSum;
int g_thresh;
unsigned int g_curBuffIndex;
byte g_verbose;
unsigned int g_lastPulseTic;
unsigned int g_refractoryMillis;
byte g_refreshFrames;
byte g_numConsecutiveZscores;

// Object to access the hardware serial port:
HardwareSerial g_Uart = HardwareSerial();

void setup()
{
  // The threshold is in scaled z-score units
  g_thresh = 1.5*ZSCALE;
  g_numConsecutiveZscores = 3;
  g_refractoryMillis = 300;
  g_refreshFrames = 3;
  g_curBuffIndex = 0;
  g_verbose = 0;
  
  // initialize the digital LED pin as an output.
  pinMode(LEDPIN, OUTPUT);
  digitalWrite(LEDPIN, LOW);
  
  // Initialize the OLED display
  // Configure to generate the high voltage from 3.3v
  oled.ssd1306_init(SSD1306_SWITCHCAPVCC);
  oled.display(); // show splashscreen
  delay(3000);

  g_Uart.begin(115200);
    
  Serial.begin(115200);
  Serial << F("*********************************************************\n");
  Serial << F("* CNI Physmon version ") << VERSION << F("\n");
  Serial << F("* Copyright 2011 Bob Dougherty <bobd@stanford.edu>\n");
  Serial << F("* http://cniweb.stanford.edu/wiki/CNI_widgets\n");
  Serial << F("*********************************************************\n\n");
  Serial << F("Initialized with ") << BUFF_SIZE << F(" element buffers. (") << freeRam() << F(" SRAM bytes free).\n\n");

  // Keep the splash screen and add a little animation until we get some data
  while(g_Uart.available()<1){
    oled.drawchar(120, 7, ' ');
    oled.drawchar(120, 7, '\\');
    oled.display();
    delay(100);
    oled.drawchar(120, 7, ' ');
    oled.drawchar(120, 7, '/');
    oled.display();
    delay(100);
  }
  oled.clear();
  oled.display();
}


void loop(){
  // We need to keep track of the current x,y data value. 0,0 is at the
  // upper left, so we want to flip Y and thus initialize by the height,
  // which is the bottom of the display.
  static byte curX;
  static byte curY = SSD1306_LCDHEIGHT;
  // Need a buffer for the line of text that we show at the top.
  static char stringBuffer[SSD1306_LCDLINEWIDTH+1];
  // Also keep track of the tic value of the last data packet that we received.
  static unsigned int prevTic;
  // The current data frame. Used to know when we are due for a display refresh.
  static byte curRefreshFrame;
  // We only fire an output when we get some number of zscores above threshold.
  static int curNumZscores;
  
  unsigned int bpm;
  
  // Make sure the output pin is low.
  digitalWrite(LEDPIN, LOW);
 
  getDataPacket();
  // Compare current tic to last tic-- it should be incremented by 1 or wrapped 
  // to 0. If not, then something went wrong; discard this packet and try to resync.
  byte resyncNow = 0;
  unsigned int ticDiff = g_data.tic-prevTic;
  if(ticDiff<1||ticDiff>2){
    Serial << F("Resync...\n");
    resyncNow = 1;
    resync();
    getDataPacket();
    snprintf(stringBuffer,SSD1306_LCDLINEWIDTH+1,"* resyncing packets...");
  }
  
  // Got something, now process it.
  unsigned int pulseIntervalMillisec = (g_data.tic - g_lastPulseTic)*DATA_INTERVAL_MILLISEC;
  int zscore = processDataPacket();
  if(zscore>g_thresh && pulseIntervalMillisec>g_refractoryMillis)
    curNumZscores++;
  
  if((curNumZscores>g_numConsecutiveZscores)){
    digitalWrite(LEDPIN, HIGH);   // set the LED on
    // instantaneous bps = 1000/pulseDiffMillisec, bpm = 60000/pulseIntervalMillisec
    bpm = 60000 / pulseIntervalMillisec;
    g_lastPulseTic = g_data.tic;
    //Serial << F("ticDiff=") << ticDiff << F(", ") << bpm << F(" BPM\n");
    // Display is 21 chars wide
    if(resyncNow)
      snprintf(stringBuffer, SSD1306_LCDLINEWIDTH+1, "* z*%d=%02d bpm=%03d     ",ZSCALE,zscore,bpm);
    else
      snprintf(stringBuffer, SSD1306_LCDLINEWIDTH+1, "%d z*%d=%02d bpm=%03d     ",ticDiff,ZSCALE,zscore,bpm);
    oled.drawline(curX, 10, curX, 14, WHITE);
    // Reset the consec counter
    curNumZscores = 0;
  }
  
  // CurY will contain an average z-score across the g_refreshFrames number of frames.
  curY -= zscore/g_refreshFrames;
  if(curRefreshFrame>g_refreshFrames){ 
    // Update the running plot
    // Clear the graph just in front of the current x position:
    oled.fillrect(curX+1, 8, 16, SSD1306_LCDHEIGHT-8, BLACK);
    // Clip the y-values to the plot area (SSD1306_LCDHEIGHT-1 at the bottom to 16 at the top)
    if(curY>SSD1306_LCDHEIGHT-1) curY = SSD1306_LCDHEIGHT-1;
    else if(curY<16) curY = 16;
    // Plot the pixel for the current data point
    oled.setpixel(curX, curY, WHITE);
    // Reset the y-position accumulator:
    curY = SSD1306_LCDHEIGHT-1;
    // Increment x, and check for wrap-around
    curX++;
    if(curX>=SSD1306_LCDWIDTH){
      curX = 0;
      // If we have wrapped around, we need to clear the first row.
      oled.drawrect(0, 8, 1, SSD1306_LCDHEIGHT-8, BLACK);
    }
    curRefreshFrame = 0;
  }
  // Draw the status string at the top:
  oled.drawstring(0, 0, stringBuffer);
  // Finished drawing the the buffer; copy it to the device:
  oled.display();
  
  if(g_verbose){
    Serial << g_data.tic << F(", ") << g_data.ppg << F(", ") << g_data.resp << F(", z=") << zscore << F("/") << ZSCALE;
    if(zscore>g_thresh) Serial << F("****");
    Serial << F("\n");
  }
  curRefreshFrame += ticDiff;
  prevTic = g_data.tic;
}

// Get the incomming data packet.
// Blocks until a full data packet is received.
void getDataPacket(){
  // Wait until all the bytes have arrived:
  while(g_Uart.available()<12) 
    delay(1);
  for(byte i=0; i<12; i++){
    g_data.byteArray[i] = g_Uart.read();
  }
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
  g_diffSum -= g_diffBuffer[g_curBuffIndex];
  g_diffBuffer[g_curBuffIndex] = SQUARE(curDataVal - dataMean);
  g_diffSum += g_diffBuffer[g_curBuffIndex];
  // Now check to see if the output should be pulsed:
  int diff = (g_data.ppg-dataMean)*ZSCALE;
  int stdev = isqrt(g_diffSum>>BUFF_SIZE_BITS);
  int zscore = diff/stdev;
  // Limit to 9 SDs
  if(zscore>9*ZSCALE) zscore = 9*ZSCALE;
  g_curBuffIndex++;
  if(g_curBuffIndex==BUFF_SIZE)
    g_curBuffIndex = 0;
  return(zscore);
}

// Resync to the incomming serial stream by waiting for 
// the dead time between data packets.
// 
// flush to clear input buffer
// loop, timing the interval between bytes until it exceeds the threshold (2ms?)
void resync(){
  unsigned long startUsec;
  // Start fresh:
  g_Uart.flush();
  // Wait for the next silent period. Doing this on every iteration will ensure 
  // that we stay in sync with data packets. 
  startUsec = micros();
  while(micros()-startUsec < DATA_SILENCE_MILLISEC && g_Uart.available()){
    byte junk = g_Uart.read();
  }
  // If we got here, then there was a silent period > DATA_SILENCE_MILLISEC, 
  // so we should be all set for the next data packet.
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

