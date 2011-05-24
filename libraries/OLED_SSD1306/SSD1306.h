#include <WProgram.h>

#define swap(a, b) { uint8_t t = a; a = b; b = t; }

#define HARDWARE_SPI
#define SPI_MODE_MASK 0x0C  // CPOL = bit 3, CPHA = bit 2 on SPCR
#define SPI_CLOCK_MASK 0x03  // SPR1 = bit 1, SPR0 = bit 0 on SPCR
#define SPI_2XCLOCK_MASK 0x01  // SPI2X = bit 0 on SPSR

#define BLACK 0
#define WHITE 1

#define SSD1306_LCDWIDTH 128
#define SSD1306_LCDHEIGHT 64
// The number of 5x7 chars that fit on a line
#define SSD1306_LCDLINEWIDTH (SSD1306_LCDWIDTH/6)


#define SSD1306_SETCONTRAST 0x81
#define SSD1306_DISPLAYALLON_RESUME 0xA4
#define SSD1306_DISPLAYALLON 0xA5
#define SSD1306_NORMALDISPLAY 0xA6
#define SSD1306_INVERTDISPLAY 0xA7
#define SSD1306_DISPLAYOFF 0xAE
#define SSD1306_DISPLAYON 0xAF

#define SSD1306_SETDISPLAYOFFSET 0xD3
#define SSD1306_SETCOMPINS 0xDA

#define SSD1306_SETVCOMDETECT 0xDB

#define SSD1306_SETDISPLAYCLOCKDIV 0xD5
#define SSD1306_SETPRECHARGE 0xD9

#define SSD1306_SETMULTIPLEX 0xA8

#define SSD1306_SETLOWCOLUMN 0x00
#define SSD1306_SETHIGHCOLUMN 0x10

#define SSD1306_SETSTARTLINE 0x40

#define SSD1306_MEMORYMODE 0x20

#define SSD1306_COMSCANINC 0xC0
#define SSD1306_COMSCANDEC 0xC8

#define SSD1306_SEGREMAP 0xA0

#define SSD1306_CHARGEPUMP 0x8D

#define SSD1306_EXTERNALVCC 0x1
#define SSD1306_SWITCHCAPVCC 0x2

class SSD1306 {
 public:
  SSD1306(int8_t MOSI, int8_t SCLK, int8_t DC, int8_t RST, int8_t CS) :mosi(MOSI), sclk(SCLK), dc(DC), rst(RST), cs(CS) {}
  SSD1306(int8_t MOSI, int8_t SCLK, int8_t DC, int8_t RST) :mosi(MOSI), sclk(SCLK), dc(DC), rst(RST), cs(-1) {}


  void ssd1306_init(uint8_t switchvcc);
  void ssd1306_command(uint8_t c);
  void ssd1306_data(uint8_t c);
  void ssd1306_data(uint8_t *c, uint16_t n);
  void ssd1306_set_brightness(uint8_t val);
  void clear_display(void);
  void clear();
  void invert(uint8_t i);
  void display();

  void setpixel(uint8_t x, uint8_t y, uint8_t color);
  void fillcircle(uint8_t x0, uint8_t y0, uint8_t r, 
		  uint8_t color);
  void drawcircle(uint8_t x0, uint8_t y0, uint8_t r, 
		  uint8_t color);
  void drawrect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, 
		uint8_t color);
  void fillrect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, 
		uint8_t color);
  void fillrectsmall(uint8_t x, uint8_t y, uint8_t w, uint8_t h, 
		uint8_t color);
  void drawline(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, 
		uint8_t color);
  void drawchar(uint8_t x, uint8_t line, uint8_t c);
  void drawstring(uint8_t x, uint8_t line, char *c);

  void drawbitmap(uint8_t x, uint8_t y, 
		  const uint8_t *bitmap, uint8_t w, uint8_t h,
		  uint8_t color);

 private:
  int8_t mosi, sclk, dc, rst, cs;
  void spiwrite(uint8_t c);

  //uint8_t buffer[128*64/8]; 
};
