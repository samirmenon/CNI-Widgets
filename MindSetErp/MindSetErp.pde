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
#include <SSD1306.h>
#include <Flash.h>

#define VERSION "0.5"

#define DEFAULT_REFRESH_INTERVAL 3

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
  #define LED_ERR 6
  HardwareSerial btSerial = HardwareSerial();
  // Pin definitions for the OLED graphical display
  #define OLED_DC 24
  #define OLED_RESET 25
  #define OLED_SS 20
  #define OLED_CLK 21
  #define OLED_MOSI 22
#elif defined(__AVR_ATmega32U4__)
  // Teensy2.0 has LED on pin 11
  #define LED_ERR 11
  HardwareSerial btSerial = HardwareSerial();
  // Pin definitions for the OLED graphical display
  #define OLED_DC 11
  #define OLED_RESET 13
  #define OLED_SS 0
  #define OLED_CLK 1
  #define OLED_MOSI 2
#else
  // Assume Arduino (LED on pin 13)
  #define LED_ERR 13
  Serial btSerial = Serial();
  // Pin definitions for the OLED graphical display
  #define OLED_DC 11
  #define OLED_RESET 13
  #define OLED_SS 12
  #define OLED_CLK 10
  #define OLED_MOSI 9
#endif

SSD1306 oled(OLED_MOSI, OLED_CLK, OLED_DC, OLED_RESET, OLED_SS);

#define BAUDRATE 115200

#define LED_RED 9
#define LED_BLU 4
#define LED_GRN 5

#define SQUARE(a) ((a)*(a))

byte g_displayUpdateInterval;
MindSet g_mindSet;

void setup() {
  // Set up the serial port on the USB interface
  Serial.begin(BAUDRATE);
  Serial << F("*********************************************************\n");
  Serial << F("* MindSet ERP version ") << VERSION << F("\n");
  Serial << F("*********************************************************\n\n");
  Serial << "Starting up...\n";
  btSerial.begin(BAUDRATE);
  
  g_displayUpdateInterval = DEFAULT_REFRESH_INTERVAL;

  // Configure LED pins
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GRN, OUTPUT);
  pinMode(LED_BLU, OUTPUT);
  pinMode(LED_ERR, OUTPUT);
  
  oled.ssd1306_init(SSD1306_SWITCHCAPVCC);
  oled.display(); // show splashscreen
  for(int i=0; i<255; i+=4){
    analogWrite(LED_RED, i);
    analogWrite(LED_GRN, i);
    analogWrite(LED_BLU, i);
    delay(2);
  }
  for(int i=255; i>=0; i-=4){
    analogWrite(LED_RED, i);
    analogWrite(LED_GRN, i);
    analogWrite(LED_BLU, i);
    delay(2);
  }
  
  // Attach the callback function to the MindSet packet processor
  g_mindSet.attach(dataReady);

  Serial << F("MindSet ERP Ready.\n\n");

}

//
// Main program loop. 
//
void loop() {
  // Need a buffer for the line of text that we show.
  static char stringBuffer[SSD1306_LCDLINEWIDTH+1];
  
  // We just feed bytes to the MindSet object as they come in. It will
  // call our callback whenever a complete data packet has been received and parsed.
  if(btSerial.available()) 
    g_mindSet.process(btSerial.read());
  
  // Turn off the LED if the counter has expired
  if((millis()-flickDutyStartMs)>flickDutyMs){
    analogWrite(LED_RED,0);
    analogWrite(LED_GRN,0);
    analogWrite(LED_BLU,0);
  }
  
  if(repCount>=128){
    for(byte i=0; i<flickPeriod; i++){
      // bit-shift division, with rounding:
      Serial.print((buffer[i]+32)>>6);
      Serial.print(',');
      //buffer[i] = 0;
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
  //if(g_mindSet.errorRate()<127 && g_mindSet.attention()>0)
  //  analogWrite(LED_RED, g_mindSet.attention()*2);
  //if(g_mindSet.errorRate()<127 && g_mindSet.meditation()>0)
  //  analogWrite(LED_BLU, g_mindSet.meditation()*2);
     
  if(g_mindSet.errorRate() == 0)
    digitalWrite(LED_ERR, HIGH);
  else
    digitalWrite(LED_ERR, LOW);
  
  buffer[flickCount] += g_mindSet.raw();
  
  flickCount++;
  if(flickCount>=flickPeriod){
    flickCount = 0;
    repCount++;
  }
  if(flickCount==0){
    analogWrite(LED_RED, 255);
    analogWrite(LED_GRN, 255);
    analogWrite(LED_BLU, 255);
    flickDutyStartMs = millis();
  }
}

void refreshDisplay(char *stringBuffer, byte rawVal){
  // We need to keep track of the current x,y data value. 0,0 is at the
  // upper left, so we want to flip Y and thus initialize by the height,
  // which is the bottom of the display.
  static byte curX;
  static byte curY = SSD1306_LCDHEIGHT;
  // The current data frame. Used to know when we are due for a display refresh.
  static byte curRefreshFrame;
  
  curRefreshFrame++;

  curY -= rawVal;
  //oled.drawline(curX, 10, curX, 14, WHITE);
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
    analogWrite(LED_BLU, curY<SSD1306_LCDHEIGHT ? (SSD1306_LCDHEIGHT-1-curY)*2 : 0);
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

