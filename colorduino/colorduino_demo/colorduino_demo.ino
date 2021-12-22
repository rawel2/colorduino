/*
  arduino I2C to Colorduino demo
   
  based on 
  -arduino firmware by michael vogt <michu@neophob.com>
  -blinkm firmware by thingM
  -"daft punk" firmware by Scott C / ThreeFN 
   
  libraries to patch:
  Wire: 
   	    utility/twi.h: #define TWI_FREQ 400000L (was 100000L) or Wire.setClock(400000L)
                       #define TWI_BUFFER_LENGTH 70 (was 32)
      	wire.h:        #define BUFFER_LENGTH 70 (was 32)
*/

#include <Wire.h>
#include "fonts.h"


#define START_OF_DATA 0x10       //data markers
#define END_OF_DATA 0x20         //data markers
#define DEST_I2C_ADDR 0xA          //set destination I2C address (must match firmware in Colorduino module)
#define DEST_I2C_COUNT 4          //set destination I2C address (must match firmware in Colorduino module)

#define SCREENSIZEX 8            //num of LEDs accross
#define SCREENSIZEY 8            //num of LEDs down
#define DS3231_ADDR 0x68

byte display_byte[3][DEST_I2C_COUNT][64];        //display array - 64 bytes x 3 colours 

typedef union {
	uint8_t bytes[7];
	struct {
		uint8_t ss;
		uint8_t mm;
		uint8_t hh;
		uint8_t dayofwek;
		uint8_t day;
		uint8_t month;
		uint8_t year;
	};
} TDATETIME;


typedef struct {
	int8_t cel;
	uint8_t fract;
} TTEMP;


//setup for plasma
typedef struct
{
  unsigned char r;
  unsigned char g;
  unsigned char b;
} ColorRGB;

//a color with 3 components: h, s and v
typedef struct 
{
  unsigned char h;
  unsigned char s;
  unsigned char v;
} ColorHSV;

void DS3231_get_datetime( TDATETIME * dt );
void DS3231_get_temp( TTEMP * tmp );

long paletteShift;
TDATETIME datetime;
TTEMP temperature;

 
void setup() 
{
  Wire.begin(); // join i2c bus (address optional for master)  
  Wire.setClock(400000L);
  Serial.begin(9600);
  paletteShift = 0;
  DS3231_init();
  //DS3231_set_time(23,26,0);
  DS3231_get_datetime(&datetime );
  DS3231_get_temp(&temperature);
//  SendGammaAll(6,44,60);

//  SendI2C(11,15);
}

void loop()
{
  if (Serial.available() > 0) {

    // look for the next valid integer in the incoming serial stream:
    int v_godz = Serial.parseInt();
    // do it again:
    int v_min = Serial.parseInt();
     while(Serial.available() > 0) {
      Serial.read();
     }

    if ((v_godz >=0) and (v_godz <=23) and (v_min >=0) and (v_min <=59)){
      DS3231_set_time(v_godz,v_min,0);
    }

    
  }
  
   plasma_morph();
}


void plasma_morph()
{
  unsigned char x,y;
  float value;
  ColorRGB colorRGB;
  ColorHSV colorHSV;
  unsigned char x0; 
  int y0; 
  unsigned char neg=0;

  for(x = 0; x <SCREENSIZEX; x++) {
    for(y = 0; y < SCREENSIZEY; y++)
      {
	value = sin(dist(x + paletteShift, y, 128.0, 128.0) / 8.0)
	  + sin(dist(x, y, 64.0, 64.0) / 8.0)
	  + sin(dist(x, y + paletteShift / 7, 192.0, 64) / 7.0)
	  + sin(dist(x, y, 192.0, 100.0) / 8.0);

//  value = sin(dist(x + paletteShift, y, 128.0, 512.0) / 32.0)
//    + sin(dist(x, y, 64.0, 256.0) / 32.0)
//    + sin(dist(x, y + paletteShift / 7, 192.0, 64) / 7.0)
//    + sin(dist(x, y, 192.0, 400.0) / 32.0);
  
	colorHSV.h=(unsigned char)((value) * 128)&0xff;

  if (datetime.hh < 9 )
    { colorHSV.s=100; 
      colorHSV.v=10;}
  else
    {	colorHSV.s=255; 
	    colorHSV.v=200;}
  
	HSVtoRGB(&colorRGB, &colorHSV);
	
	display(x, y, colorRGB.r, colorRGB.g, colorRGB.b);
  //display(x, y, 0,0,200);
      }
  }
  paletteShift++;

//  Serial.println(paletteShift);
  
  x0 = 0;
//  y0 = paletteShift%300 - 8;
//  if (y0 > 0) {y0=0;}
  
  y0 = paletteShift%300;
  
  if (y0 < 9)
    {y0 = y0-8;}
  else if (y0 <241)
    {y0 = 0;}
  else if (y0 <250)  
    {y0 = y0 - 241;}
  else if (y0 <259)  
    {y0 = y0 - 258;}
  else if (y0 <291)  
    {y0 = 0;}
  else
    {y0 = y0 - 291;}  
    
  //neg = paletteShift / 600 %2;

  
  if(paletteShift % 300 < 250)
  {
    if (paletteShift % 30 == 0)
    {DS3231_get_datetime(&datetime );};
  
    DispShowChar((datetime.hh/10)+48,  x0,  x0,  x0, y0+2, 3, neg, 0);
    DispShowChar((datetime.hh%10)+48,  x0,  x0,  x0, y0+2, 2, neg, (paletteShift%12 > 2 ? 2 : 0));
    DispShowChar((datetime.mm/10)+48,  x0,  x0,  x0, y0, 1, neg, (paletteShift%12 > 2 ? 1 : 0));
    DispShowChar((datetime.mm%10)+48,  x0,  x0,  x0, y0, 0, neg, 0);
  }
  else
  {
    if (paletteShift % 300 == 250)
    {DS3231_get_temp(&temperature);};
  
    DispShowChar((temperature.cel/10)+48,  x0,  x0,  x0, y0, 3, neg, 0);
    DispShowChar((temperature.cel%10)+48,  x0,  x0,  x0, y0, 2, neg, 0);
    DispShowChar('"',  x0,  x0,  x0, y0, 1, neg, 0);
    DispShowChar('_',  x0,  x0,  x0, y0+2, 0, neg, 0);
    
  
   
//  DispShowChar('K',  x0,  x0,  x0, y0, 3, 0,0);
//  DispShowChar('u',  x0,  x0,  x0, y0, 2, 0,0);
//  DispShowChar('b',  x0,  x0,  x0, y0, 1, 0,0);
//  DispShowChar('a',  x0,  x0,  x0, y0, 0, 0,0);
  }
  update_display(DEST_I2C_ADDR,0);
  update_display(DEST_I2C_ADDR,1);
  update_display(DEST_I2C_ADDR,2);
  update_display(DEST_I2C_ADDR,3);
 
}

void SendGammaAll(uint8_t R,uint8_t G, uint8_t B){
  for(uint8_t i=0;i<DEST_I2C_COUNT;i++){
    SendGamma(R, G, B,DEST_I2C_ADDR+i);
  }
}

void SendGamma(uint8_t R,uint8_t G, uint8_t B,uint8_t addr2){
    // BlinkM_sendBuffer(byte addr, byte col, byte* disp_data)
    uint8_t rob[64];
    rob[0] = R;
    rob[1] = G;
    rob[2] = B;
    BlinkM_sendBuffer(addr2, 4, rob);  
}

void SendI2C(uint8_t oldaddr,uint8_t newaddr){
    // BlinkM_sendBuffer(byte addr, byte col, byte* disp_data)
    uint8_t rob[64];
    rob[0] = 'P';
    rob[1] = 'R';
    rob[2] = newaddr;
    BlinkM_sendBuffer(oldaddr, 3, rob);  
}
void display1(byte x, byte y, byte r, byte g, byte b) {
  byte p1 = ((y%8)*8)+x;   //convert from x,y to pixel number in array
  byte p2 = y/8;
  
  
  display_byte[0][p2][p1] = r;
  display_byte[1][p2][p1] = g;
  display_byte[2][p2][p1] = b;
}
//update display buffer using x,y,r,g,b format
void display(byte x, byte y, byte r, byte g, byte b) {
  byte p1 = (y*8)+x;   //convert from x,y to pixel number in array
  byte p2 = ((y)*8)+(7-x);
  byte p3 = ((7-y)*8)+x;
  byte p4 = ((7-y)*8)+(7-x);
  
  display_byte[0][0][p1] = r;
  display_byte[1][0][p1] = g;
  display_byte[2][0][p1] = b;

  display_byte[0][1][p2] = r;
  display_byte[1][1][p2] = g;
  display_byte[2][1][p2] = b;
  
  display_byte[0][2][p1] = r;
  display_byte[1][2][p1] = g;
  display_byte[2][2][p1] = b;
  
  display_byte[0][3][p2] = r;
  display_byte[1][3][p2] = g;
  display_byte[2][3][p2] = b;
//
//  display_byte[0][2][p3] = r;
//  display_byte[1][2][p3] = g;
//  display_byte[2][2][p3] = b;
//
//  display_byte[0][3][p4] = r;
//  display_byte[1][3][p4] = g;
//  display_byte[2][3][p4] = b;
  
 }


//send display buffer to display 
void update_display(byte addr, byte addr2) {   
  BlinkM_sendBuffer(addr+addr2, 0, display_byte[0][addr2]);
  BlinkM_sendBuffer(addr+addr2, 1, display_byte[1][addr2]);   
  BlinkM_sendBuffer(addr+addr2, 2, display_byte[2][addr2]);  
}


//send data via I2C to a client
static byte BlinkM_sendBuffer(byte addr, byte col, byte* disp_data) {
  Wire.beginTransmission(addr);
  Wire.write(START_OF_DATA);
  Wire.write(col);
  Wire.write(disp_data, 64);
  Wire.write(END_OF_DATA);
  return Wire.endTransmission();
}


//plasma convert
//Converts an HSV color to RGB color
void HSVtoRGB(void *vRGB, void *vHSV) 
{
  float r, g, b, h, s, v; //this function works with floats between 0 and 1
  float f, p, q, t;
  int i;
  ColorRGB *colorRGB=(ColorRGB *)vRGB;
  ColorHSV *colorHSV=(ColorHSV *)vHSV;

  h = (float)(colorHSV->h / 256.0);
  s = (float)(colorHSV->s / 256.0);
  v = (float)(colorHSV->v / 256.0);

  //if saturation is 0, the color is a shade of grey
  if(s == 0.0) {
    b = v;
    g = b;
    r = g;
  }
  //if saturation > 0, more complex calculations are needed
  else
  {
    h *= 6.0; //to bring hue to a number between 0 and 6, better for the calculations
    i = (int)(floor(h)); //e.g. 2.7 becomes 2 and 3.01 becomes 3 or 4.9999 becomes 4
    f = h - i;//the fractional part of h

    p = (float)(v * (1.0 - s));
    q = (float)(v * (1.0 - (s * f)));
    t = (float)(v * (1.0 - (s * (1.0 - f))));

    switch(i)
    {
      case 0: r=v; g=t; b=p; break;
      case 1: r=q; g=v; b=p; break;
      case 2: r=p; g=v; b=t; break;
      case 3: r=p; g=q; b=v; break;
      case 4: r=t; g=p; b=v; break;
      case 5: r=v; g=p; b=q; break;
      default: r = g = b = 0; break;
    }
  }
  colorRGB->r = (int)(r * 255.0);
  colorRGB->g = (int)(g * 255.0);
  colorRGB->b = (int)(b * 255.0);
}

 
void DispShowChar(char chr,unsigned char R,unsigned char G,unsigned char B,char bias,char disp, char neg, char dwukropek)
{
  unsigned char i,j,aa,bb,cc,temp;
  unsigned int Char;
  unsigned char chrtemp[8] = {0};
  unsigned char p1;
  
//  if ((bias > 8) || (bias < -8))
//  {}
//  else
//  {
    if ((bias > -8) && (bias < 8)){
      Char = (chr - 32)*8;
      aa = (bias>=0 ? 0 : -bias);
      bb = (bias>=0 ? bias :  0);
      cc = 8- (bias>=0 ? bias : -bias);
      
      for(i = 0;i< cc;i++){
        chrtemp[aa+i] = pgm_read_byte(&(Font8x8_[Char+bb+i]));    
      }    
    }
//  }
  
  for(i = 0;i < 8;i++)
  {
    if( ((dwukropek == 1) and (i==0) and (bias==0)) or ((dwukropek == 2) and (i==7) and (bias==2)))
      {temp = B01100110;}
    else
      {temp = chrtemp[i];}

    for(j = 0;j < 8;j++)
    {
      p1 = (j*8)+i;
      if((temp & 0x80) and (neg))
      {
        display_byte[0][disp][p1] = R;
        display_byte[1][disp][p1] = G;
        display_byte[2][disp][p1] = B;
      }
      
      if(!(temp & 0x80) and (!neg))
      {
        display_byte[0][disp][p1] = R;
        display_byte[1][disp][p1] = G;
        display_byte[2][disp][p1] = B;
      }
      temp = temp << 1;
    }
  }

}


float dist(float a, float b, float c, float d) 
{
  return sqrt((c-a)*(c-a)+(d-b)*(d-b));
}



void DS3231_init( void ) {
	uint8_t ctrl = 0;
	TWI_write_buf( DS3231_ADDR, 0x0e, 1, &ctrl );
}

void DS3231_get_datetime( TDATETIME * dt ) {
	uint8_t i;
	uint8_t buf[7];
	TWI_read_buf( DS3231_ADDR, 0x00, 7, buf );
	for( i=0; i<7; i++ ) dt->bytes[i] = bcd2dec( buf[i] );
}

void DS3231_set_time( uint8_t hh, uint8_t mm, uint8_t ss ) {
	uint8_t buf[3];
	buf[0]=dec2bcd(ss);
	buf[1]=dec2bcd(mm);
	buf[2]=dec2bcd(hh);
	TWI_write_buf( DS3231_ADDR, 0x00, 3, buf );
}

void DS3231_set_date( uint8_t year, uint8_t month, uint8_t day, uint8_t dayofweek ) {
	uint8_t buf[4];
	buf[0]=dayofweek;
	buf[1]=dec2bcd(day);
	buf[2]=dec2bcd(month);
	buf[3]=dec2bcd(year);
	TWI_write_buf( DS3231_ADDR, 0x03, 4, buf );
}


void DS3231_get_temp( TTEMP * tmp ) {
	uint8_t buf[2];
	TWI_read_buf( DS3231_ADDR, 0x11, 2, buf );
	tmp->cel = buf[0] ;
	tmp->fract = buf[1]>>6;

	TWI_read_buf( DS3231_ADDR, 0x0e, 1, buf );
	buf[0] |= (1<<5);
	TWI_write_buf( DS3231_ADDR, 0x0e, 1, buf );
}

// konwersja liczby dziesiętnej na BCD
uint8_t dec2bcd(uint8_t dec) {
return ((dec / 10)<<4) | (dec % 10);
}

// konwersja liczby BCD na dziesięną
uint8_t bcd2dec(uint8_t bcd) {
    return ((((bcd) >> 4) & 0x0F) * 10) + ((bcd) & 0x0F);
}


void TWI_write_buf( uint8_t SLA, uint8_t adr, uint8_t len, uint8_t *buf){
  Wire.beginTransmission(SLA);
  Wire.write(adr);
  Wire.write(buf,len);
  Wire.endTransmission();
 
};

void TWI_read_buf(uint8_t SLA, uint8_t adr, uint8_t len, uint8_t *buf){
    Wire.beginTransmission(SLA);
    Wire.write(adr);
    Wire.endTransmission();
    Wire.requestFrom(SLA, len);
    for(int i=0; i<len; i++)
      {
        buf[i]=Wire.read();
      }

};
