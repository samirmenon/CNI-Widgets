// some of this code was written by <cstone@pobox.com> originally; 
// it is in the public domain.
//
// 2011.05.18 Bob Dougherty added hardware SPI support and other optimizations.

//#include <Wire.h>
#include <avr/pgmspace.h>
#include <WProgram.h>
#include <util/delay.h>
#include <stdlib.h>

#include "SSD1306.h"
#include "glcdfont.h"

static uint8_t is_reversed = 0;

// a handy reference to where the pages are on the screen
const uint8_t pagemap[] = { 0, 1, 2, 3, 4, 5, 6, 7};

// a 5x7 font table
extern uint8_t PROGMEM font[];

// the memory buffer for the LCD
#include "glcdbuffer.h"
/*
To initialize the memory buffer with a custom image, create a grayscale
image that is 64x128 pixels and contains just black and white (i.e., 0 and 255).
Then run the following python code on your image file (e.g., buffer.png):

import scipy, sys
im = scipy.misc.pilutil.imread('buffer.png')
im[im>0] = 1
# loop over pages (8 rows) columns and convert each page-column to a byte
bytes = scipy.zeros(im.shape[1]*im.shape[0]/8,'uint8')
curByte = 0
for r in range(0,im.shape[0],8):
  for c in range(im.shape[1]):
    for b in range(0,8):
      bytes[curByte] = bytes[curByte] | im[r+b,c]<<b
    curByte = curByte+1

fp = open('glcdbuffer.h', 'w')
fp.write('#ifndef FGLCDBUFFER_H\n#define FGLCDBUFFER_H\n\n')
fp.write("static uint8_t buffer[%d] = {\n" % bytes.shape[0])

for i in range(bytes.shape[0]):
    fp.write("0x%02x" % bytes[i])
    if i != bytes.shape[0]-1:
      fp.write(", ")
    if i%16 == 15:
      fp.write("\n")

fp.write("};\n\n#endif\n")
fp.close()

*/


void SSD1306::drawbitmap(uint8_t x, uint8_t y, 
			const uint8_t *bitmap, uint8_t w, uint8_t h,
			uint8_t color) {
  for (uint8_t j=0; j<h; j++) {
    for (uint8_t i=0; i<w; i++ ) {
      if (pgm_read_byte(bitmap + i + (j/8)*w) & _BV(j%8)) {
        setpixel(x+i, y+j, color);
      }
    }
  }
}

void SSD1306::drawstring(uint8_t x, uint8_t line, char *c) {
  while (c[0] != 0) {
    drawchar(x, line, c[0]);
    c++;
    x += 6; // 6 pixels wide
    if (x + 6 >= SSD1306_LCDWIDTH) {
      x = 0;    // ran out of this line
      line++;
    }
    if (line >= (SSD1306_LCDHEIGHT/8))
      return;        // ran out of space :(
  }

}

void  SSD1306::drawchar(uint8_t x, uint8_t line, uint8_t c) {
  for (uint8_t i =0; i<5; i++ ) {
    buffer[x + (line*128) ] = pgm_read_byte(font+(c*5)+i);
    x++;
  }
}


// bresenham's algorithm - thx wikpedia
void SSD1306::drawline(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, 
		      uint8_t color) {
  uint8_t steep = abs(y1 - y0) > abs(x1 - x0);
  if (steep) {
    swap(x0, y0);
    swap(x1, y1);
  }

  if (x0 > x1) {
    swap(x0, x1);
    swap(y0, y1);
  }

  uint8_t dx, dy;
  dx = x1 - x0;
  dy = abs(y1 - y0);

  int8_t err = dx / 2;
  int8_t ystep;

  if (y0 < y1) {
    ystep = 1;
  } else {
    ystep = -1;}

  for (; x0<x1; x0++) {
    if (steep) {
      setpixel(y0, x0, color);
    } else {
      setpixel(x0, y0, color);
    }
    err -= dy;
    if (err < 0) {
      y0 += ystep;
      err += dx;
    }
  }
}

// filled rectangle
void SSD1306::fillrect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, 
		      uint8_t color) {
  // 2011.05.18 Bob Dougherty (bobd@stanford.edu): added the following code to do 
  // faster filled rects by filling whole bytes at a time.
  if(h==0 || w==0) return;
  uint8_t firstPage = y/8;
  uint8_t startBits = y%8;
  uint8_t yEnd = y+h-1;
  if(yEnd>=SSD1306_LCDHEIGHT)
    yEnd = SSD1306_LCDHEIGHT-1;
  uint8_t lastPage = yEnd/8;
  uint8_t endBits = 7-yEnd%8;
  uint8_t xEnd = x+w-1;
  if(xEnd>=SSD1306_LCDWIDTH)
    xEnd = SSD1306_LCDWIDTH-1;

  for(uint8_t c=x; c<=xEnd; c++){
    if(color == WHITE){
      if(firstPage==lastPage){
        buffer[c+firstPage*128] |= (255<<startBits & 255>>endBits);
      }else{
        buffer[c+firstPage*128] |= 255<<startBits;
        buffer[c+ lastPage*128] |= 255>>endBits;
        for(uint8_t p=firstPage+1; p<lastPage; p++)
          buffer[c+ p*128] = 255;
      }
    }else{
      if(firstPage==lastPage){
        buffer[c+firstPage*128] &= ~(255<<startBits & 255>>endBits);
      }else{
        buffer[c+firstPage*128] &= ~(255<<startBits);
        buffer[c+ lastPage*128] &= ~(255>>endBits);
        for(uint8_t p=firstPage+1; p<lastPage; p++)
          buffer[c+ p*128] = 0;
      }
    }
  }
  /*
  // stupidest version - just pixels - but fast with internal buffer!
  for (uint8_t i=x; i<x+w; i++) {
    for (uint8_t j=y; j<y+h; j++) {
      setpixel(i, j, color);
    }
  }
  */
}

void SSD1306::fillrectsmall(uint8_t x, uint8_t y, uint8_t w, uint8_t h, 
		      uint8_t color) {
  // stupidest version - just pixels - but fast with internal buffer!
  for (uint8_t i=x; i<x+w; i++) {
    for (uint8_t j=y; j<y+h; j++) {
      setpixel(i, j, color);
    }
  }
}


// draw a rectangle
void SSD1306::drawrect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, 
		      uint8_t color) {
  // stupidest version - just pixels - but fast with internal buffer!
  for (uint8_t i=x; i<x+w; i++) {
    setpixel(i, y, color);
    setpixel(i, y+h-1, color);
  }
  for (uint8_t i=y; i<y+h; i++) {
    setpixel(x, i, color);
    setpixel(x+w-1, i, color);
  } 
}

// draw a circle outline
void SSD1306::drawcircle(uint8_t x0, uint8_t y0, uint8_t r, 
			uint8_t color) {
  int8_t f = 1 - r;
  int8_t ddF_x = 1;
  int8_t ddF_y = -2 * r;
  int8_t x = 0;
  int8_t y = r;

  setpixel(x0, y0+r, color);
  setpixel(x0, y0-r, color);
  setpixel(x0+r, y0, color);
  setpixel(x0-r, y0, color);

  while (x<y) {
    if (f >= 0) {
      y--;
      ddF_y += 2;
      f += ddF_y;
    }
    x++;
    ddF_x += 2;
    f += ddF_x;
  
    setpixel(x0 + x, y0 + y, color);
    setpixel(x0 - x, y0 + y, color);
    setpixel(x0 + x, y0 - y, color);
    setpixel(x0 - x, y0 - y, color);
    
    setpixel(x0 + y, y0 + x, color);
    setpixel(x0 - y, y0 + x, color);
    setpixel(x0 + y, y0 - x, color);
    setpixel(x0 - y, y0 - x, color);
    
  }
}

void SSD1306::fillcircle(uint8_t x0, uint8_t y0, uint8_t r, 
			uint8_t color) {
  int8_t f = 1 - r;
  int8_t ddF_x = 1;
  int8_t ddF_y = -2 * r;
  int8_t x = 0;
  int8_t y = r;

  for (uint8_t i=y0-r; i<=y0+r; i++) {
    setpixel(x0, i, color);
  }

  while (x<y) {
    if (f >= 0) {
      y--;
      ddF_y += 2;
      f += ddF_y;
    }
    x++;
    ddF_x += 2;
    f += ddF_x;
  
    for (uint8_t i=y0-y; i<=y0+y; i++) {
      setpixel(x0+x, i, color);
      setpixel(x0-x, i, color);
    } 
    for (uint8_t i=y0-x; i<=y0+x; i++) {
      setpixel(x0+y, i, color);
      setpixel(x0-y, i, color);
    }    
  }
}

// the most basic function, set a single pixel
inline void SSD1306::setpixel(uint8_t x, uint8_t y, uint8_t color) {
  if ((x >= SSD1306_LCDWIDTH) || (y >= SSD1306_LCDHEIGHT))
    return;

  // x is which column
  if (color == WHITE) 
    buffer[x+ (y/8)*128] |= _BV((y%8));  
  else
    buffer[x+ (y/8)*128] &= ~_BV((y%8)); 
}


void SSD1306::ssd1306_init(uint8_t vccstate) {
  // set pin directions
  pinMode(mosi, OUTPUT);
  pinMode(sclk, OUTPUT);
  pinMode(dc, OUTPUT);
  pinMode(rst, OUTPUT);
  pinMode(cs, OUTPUT);

  digitalWrite(rst, HIGH);
  // VDD (3.3V) goes high at start, lets just chill for a ms
  delay(1);
  // bring0xset low
  digitalWrite(rst, LOW);
  // wait 10ms
  delay(10);
  // bring out of reset
  digitalWrite(rst, HIGH);
  // turn on VCC (9V?)
  
  digitalWrite(sclk, LOW);
  digitalWrite(mosi, LOW);
  digitalWrite(cs, HIGH);
  
#ifdef HARDWARE_SPI
  /*
    The SPI control register (SPCR) has 8 bits, each of which control a particular SPI setting.

    SPCR
    | 7    | 6    | 5    | 4    | 3    | 2    | 1    | 0    |
    | SPIE | SPE  | DORD | MSTR | CPOL | CPHA | SPR1 | SPR0 |

    SPIE - Enables the SPI interrupt when 1
    SPE - Enables the SPI when 1
    DORD - Sends data least Significant Bit First when 1, most Significant Bit first when 0
    MSTR - Sets the Arduino in master mode when 1, slave mode when 0
    CPOL - Sets the data clock to be idle when high if set to 1, idle when low if set to 0
    CPHA - Samples data on the falling edge of the data clock when 1, rising edge when 0
    SPR1 and SPR0 - Sets the SPI speed, 00 is fastest (4MHz) 11 is slowest (250KHz)
    */
  // Warning: if the SS pin ever becomes a LOW INPUT then SPI 
  // automatically switches to Slave, so the data direction of 
  // the SS pin MUST be kept as OUTPUT.
  SPCR = 0;
  SPCR |= _BV(MSTR);
  SPCR |= _BV(SPE);
  
  // To end SPI: SPCR &= ~_BV(SPE);
  
  // Set SPI mode, if default mode doesn't work
  //uint8_t spiMode = 0;
  //SPCR = (SPCR & ~SPI_MODE_MASK) | spiMode;
  
  // set clock divider
  uint8_t divider = 0x04; // DIV2=0x04, DIV4=0x00 (default), DIV8=0x05
  SPCR = (SPCR & ~SPI_CLOCK_MASK) | (divider & SPI_CLOCK_MASK);
  SPSR = (SPSR & ~SPI_2XCLOCK_MASK) | ((divider >> 2) & SPI_2XCLOCK_MASK);

  // Set bit order:
  // LSB first:
  //SPCR |= _BV(DORD);
  // MSB first:
  //SPCR &= ~(_BV(DORD));
#endif

  ssd1306_command(SSD1306_DISPLAYOFF);  // 0xAE
  ssd1306_command(SSD1306_SETLOWCOLUMN | 0x0);  // low col = 0
  ssd1306_command(SSD1306_SETHIGHCOLUMN | 0x0);  // hi col = 0
  
  ssd1306_command(SSD1306_SETSTARTLINE | 0x0); // line #0

  ssd1306_command(SSD1306_SETCONTRAST);  // 0x81
  if (vccstate == SSD1306_EXTERNALVCC) {
    ssd1306_command(0x9F);  // external 9V
  } else {
    ssd1306_command(0xCF);  // chargepump
  }
    
  ssd1306_command(0xa1);  // setment remap 95 to 0 (?)

  ssd1306_command(SSD1306_NORMALDISPLAY); // 0xA6

  ssd1306_command(SSD1306_DISPLAYALLON_RESUME); // 0xA4

  ssd1306_command(SSD1306_SETMULTIPLEX); // 0xA8
  ssd1306_command(0x3F);  // 0x3F 1/64 duty
  
  ssd1306_command(SSD1306_SETDISPLAYOFFSET); // 0xD3
  ssd1306_command(0x0); // no offset
  
  ssd1306_command(SSD1306_SETDISPLAYCLOCKDIV);  // 0xD5
  ssd1306_command(0x80);  // the suggested ratio 0x80
  
  ssd1306_command(SSD1306_SETPRECHARGE); // 0xd9
  if (vccstate == SSD1306_EXTERNALVCC) {
    ssd1306_command(0x22); // external 9V
  } else {
    ssd1306_command(0xF1); // DC/DC
  }
  
  ssd1306_command(SSD1306_SETCOMPINS); // 0xDA
  ssd1306_command(0x12); // disable COM left/right remap
  
  ssd1306_command(SSD1306_SETVCOMDETECT); // 0xDB
  ssd1306_command(0x40); // 0x20 is default?

  ssd1306_command(SSD1306_MEMORYMODE); // 0x20
  ssd1306_command(0x00); // 0x0 act like ks0108
  
  // left to right scan
  ssd1306_command(SSD1306_SEGREMAP | 0x1);

  ssd1306_command(SSD1306_COMSCANDEC);

  ssd1306_command(SSD1306_CHARGEPUMP); //0x8D
  if (vccstate == SSD1306_EXTERNALVCC) {
    ssd1306_command(0x10);  // disable
  } else {
    ssd1306_command(0x14);  // disable    
  }
  ssd1306_command(SSD1306_DISPLAYON);//--turn on oled panel
}


void SSD1306::invert(uint8_t i) {
  if (i) {
    ssd1306_command(SSD1306_INVERTDISPLAY);
  } else {
    ssd1306_command(SSD1306_NORMALDISPLAY);
  }
}

inline void SSD1306::spiwrite(uint8_t c) {
#ifdef HARDWARE_SPI
  // Put the current byte in to be sent.
  SPDR = c;
  // wait for the byte to finish transferring
  while (!(SPSR & _BV(SPIF))) ;
#else
  shiftOut(mosi, sclk, MSBFIRST, c);
#endif
}

void SSD1306::ssd1306_command(uint8_t c) { 
  digitalWrite(dc, LOW);
  digitalWrite(cs, LOW);
  spiwrite(c);
  digitalWrite(cs, HIGH);
}

void SSD1306::ssd1306_data(uint8_t c) {
  digitalWrite(dc, HIGH);
  digitalWrite(cs, LOW);
  spiwrite(c);
  digitalWrite(cs, HIGH);
}

void SSD1306::ssd1306_data(uint8_t *c, uint16_t n) {
  digitalWrite(dc, HIGH);
  digitalWrite(cs, LOW);
  for(uint16_t i=0; i<n; i++)
    spiwrite(*c++);
  digitalWrite(cs, HIGH);
}

void SSD1306::ssd1306_set_brightness(uint8_t val) {
  ssd1306_command(SSD1306_SETCONTRAST);
  ssd1306_command(val);
}

void SSD1306::display(void) {
  //ssd1306_command(SSD1306_SETLOWCOLUMN | 0x0);  // low col = 0
  //ssd1306_command(SSD1306_SETHIGHCOLUMN | 0x0);  // hi col = 0
  //ssd1306_command(SSD1306_SETSTARTLINE | 0x0); // line #0
  ssd1306_data(buffer,1024);
}

// clear everything
void SSD1306::clear(void) {
  memset(buffer, 0, 1024);
}

void SSD1306::clear_display(void) {
 
}
