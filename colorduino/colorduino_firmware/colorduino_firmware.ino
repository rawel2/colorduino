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

#include "Arduino.h"
#include <EEPROM.h>
#include <Wire.h>
#include <avr/pgmspace.h> 

#define I2C_DEVICE_ADDRESS_DEF 0x01   //I2C address for this device 
#define START_OF_DATA 0x10            //data markers
#define END_OF_DATA 0x20              //data markers


/*****************************
define the IO
*****************************/
#define RST_BIT 0x04
#define LAT_BIT 0x02
#define SLB_BIT 0x01

#define SCL_BIT 0x40
#define SDA_BIT 0x80

#define RST PORTC
#define LAT PORTC
#define SLB PORTC
#define SDA PORTD
#define SCL PORTD

#define open_line0  {PORTB=0x01;}
#define open_line1  {PORTB=0x02;}
#define open_line2  {PORTB=0x04;}
#define open_line3  {PORTB=0x08;}
#define open_line4  {PORTB=0x10;}
#define open_line5  {PORTB=0x20;}
#define open_line6  {PORTD=0x08;}
#define open_line7  {PORTD=0x10;}
#define close_all_line  {PORTD=0x00;PORTB=0x00;}

uint8_t dots[2][8][8][3] = {0};

//dots matrix
//[2]:Page:one for display, one for receive data
//[8]:Row:8 row in LED plane
//[8]:Column:8 column in one row
//[3]:Color:RGB data: 0 for Red; 1 for green, 2 for Blue

uint8_t Gamma_Value[3] = {20,63,50};
        //Gamma correctly value, every LED plane is different.value range is 0~63
        //[3]:RGB data, 0 for Red; 1 for green, 2 for Blue

uint8_t Page_Index = 0; // the index of buffer
uint8_t Page_Write = 1;
uint8_t row = 0;//the value of row in LED plane, from 0~7
uint8_t column = 0;//the value of every row, from 0~7
uint8_t color = 0;//the value of every dots, 0 is Red, 1 is Green, 2 is Blue
uint8_t I2C_Device_Address = 0;
uint8_t line = 0;


extern const unsigned char font8_8[92][8];

void _IO_Init()
{
  DDRD = 0xff; // set all pins direction of PortD
  DDRC = 0xff; // set all pins direction of PortC
  DDRB = 0xff; // set all pins direction of PortB
  
  PORTD = 0x00; // set all pins output is low of PortD
  PORTC = 0x00; // set all pins output is low of PortC
  PORTB = 0x00; // set all pins output is low of PortB
}

void _LED_Init()
{
  LED_RST(1);
  LED_Delay(1);
  LED_RST(0);
  LED_Delay(1);
  LED_RST(1);
  LED_Delay(1);
  line = 0;
  
}

void _TC2_Init()
{
  TCCR2A |= (1 << WGM21) | (1 << WGM20);   
  TCCR2B |= ((1<<CS22)|(1<<CS20));   // by clk/64
  TCCR2B &= ~((1<<CS21));   // by clk/64
  TCCR2A &= ~((1<<WGM21) | (1<<WGM20));   // Use normal mode
  ASSR |= (1<<AS2);       // Use internal clock - external clock not used in Arduino
  TIMSK2 |= (1<<TOIE2) | (0<<OCIE2B);   //Timer2 Overflow Interrupt Enable
  TCNT2 = 0xff;
}


/****************************************************
the timer2 operate functions zone
****************************************************/
ISR(TIMER2_OVF_vect)          //Timer2  Service 
{ 
  cli();  
  TCNT2 = 0x64;      //flash a led matrix frequency is 100.3Hz,period is 9.97ms
  //TCNT2 = 0x63;      //flash a led matrix frequency is 99.66Hz,period is 10.034ms   
    if(line > 7) line = 0;    
    close_all_line;  
    run(line);
    open_line(line);
    line++;
    sei();
}
/****************************************************
the LED Hardware operate functions zone
****************************************************/
void LED_SDA(uint8_t temp)
{
  if (temp) 
    SDA|=SDA_BIT;
  else
    SDA&=~SDA_BIT;
}

void LED_SCL(uint8_t temp)
{
  if (temp) 
    SCL|=SCL_BIT;
  else
    SCL&=~SCL_BIT;
}

void LED_RST(uint8_t temp)
{
  if (temp) 
    RST|=RST_BIT;
  else
    RST&=~RST_BIT;
}

void LED_LAT(uint8_t temp)
{
  if (temp) 
    LAT|=LAT_BIT;
  else
    LAT&=~LAT_BIT;
}

void LED_SLB(uint8_t temp)
{
  if (temp) 
    SLB|=SLB_BIT;
  else
    SLB&=~SLB_BIT;
}
/***************************************************
the LED datas operate functions zone
***************************************************/

void SetGamma()
{
  cli();
  uint8_t i = 0;
  uint8_t j = 0;
  uint8_t k = 0;
  uint8_t temp = 0;
  LED_LAT(0);
  LED_SLB(0);
  for(k=0;k<8;k++)
    for(i = 3;i > 0 ;i--)
    {
      temp = Gamma_Value[i-1]<<2;
      for(j = 0;j<6;j++)
      {
        if(temp &0x80)
          LED_SDA(1);
        else
          LED_SDA(0);
        
        temp =temp << 1;
        LED_SCL(0);
        LED_SCL(1);
    }
  }
  LED_SLB(1);
  sei();
}

void run(uint8_t k)
{
  uint8_t i = 0;
  uint8_t j = 0;
  uint8_t p = 0;
  uint8_t temp = 0;
  LED_SLB(1);
  LED_LAT(0);
  for(i = 0;i<8;i++)
  {
    
    for(j=0;j<3;j++)
    {
      temp = dots[Page_Index][k][i][2-j];
      for(p=0;p<8;p++)
      {
         if(temp & 0x80)
           LED_SDA(1);
         else
           LED_SDA(0);
           
         temp = temp<<1;  
         LED_SCL(0);
         LED_SCL(1);
       }
     }
  }
  LED_LAT(1);
  LED_LAT(0);
}

void open_line(uint8_t x)
{
  switch (x)
  {  
    case 0 :open_line0;
            break;
    case 1 :open_line1;
            break;
    case 2 :open_line2;
            break;
    case 3 :open_line3;
            break;
    case 4 :open_line4;
            break;
    case 5 :open_line5;
            break;
    case 6 :open_line6;
            break;
    case 7 :open_line7;
            break;
    default: close_all_line;
            break;
  }
}


void DispShowCharBG(char chr,unsigned char R,unsigned char G,unsigned char B,char bias,unsigned char Rbk,unsigned char Gbk,unsigned char Bbk)
{
  unsigned char i,j,aa,bb,cc,temp;
  unsigned int Char;
  unsigned char chrtemp[8] = {0};
  
  if ((bias > 8) || (bias < -8))
    return;

  if ((bias > -8) && (bias < 8)){
    Char = chr - 32;
    aa = (bias>=0 ? 0 : -bias);
    bb = (bias>=0 ? bias :  0);
    cc = 8- (bias>=0 ? bias : -bias);
    
    for(i = 0;i< cc;i++){
      chrtemp[aa+i] = pgm_read_byte(&(font8_8[Char][bb+i]));    
    }    
  }
  
  for(i = 0;i < 8;i++)
  {
    temp = chrtemp[i];
    for(j = 0;j < 8;j++)
    {
      if(temp & 0x80)
      {
        dots[Page_Write][j][i][0] = R;
        dots[Page_Write][j][i][1] = G;
        dots[Page_Write][j][i][2] = B;
      }
      else
      {
        dots[Page_Write][j][i][0] = Rbk;
        dots[Page_Write][j][i][1] = Gbk;
        dots[Page_Write][j][i][2] = Bbk;
      }
      temp = temp << 1;
    }
  }
  FlipPage();
}
/********************************************************
Name:DispShowChar
Function:Display a English latter in LED matrix
Parameter:chr :the latter want to show
          R: the value of RED.   Range:RED 0~255
          G: the value of GREEN. Range:RED 0~255
          B: the value of BLUE.  Range:RED 0~255
          bias: the bias of a letter in LED Matrix.Range -7~7
********************************************************/
void DispShowChar(char chr,unsigned char R,unsigned char G,unsigned char B,char bias)
{
  DispShowCharBG(chr, R, G, B, bias, 0, 0, 0);
};

/********************************************************
Name:DispShowColor
Function:Fill a color in LED matrix
Parameter:R: the value of RED.   Range:RED 0~255
          G: the value of GREEN. Range:RED 0~255
          B: the value of BLUE.  Range:RED 0~255
********************************************************/
void DispShowColor(uint8_t R,uint8_t G,uint8_t B)
{
  uint8_t i,j;
  
  for (i = 0;i<8;i++)
    for(j = 0;j<8;j++)
    {
      dots[Page_Write][i][j][2] = R;
      dots[Page_Write][i][j][1] = G;
      dots[Page_Write][i][j][0] = B;
    }
  
  FlipPage();
}

/******************************************
the other operate functions zone
******************************************/
void LED_Delay(uint8_t i)
{
  uint16_t y;
  y = i * 10;
  while(y--);
}

void FlipPage(void)
{
  Page_Index = !Page_Index;
  Page_Write = !Page_Index;
}

void SetI2cAddrToEEPROM(uint8_t addr){
  //set i2c address in eeprom
  char rob;
  rob ='P';
  EEPROM.write(0, rob);
  rob ='R';
  EEPROM.write(1, rob);
  rob =addr;
  EEPROM.write(2, rob);
}

void GetI2CAddrFromEEPROM(void){
  if((EEPROM.read(0)=='P') && (EEPROM.read(1)=='R') )
  {
    I2C_Device_Address = EEPROM.read(2);
    Serial.print("Address from EEPROM: 0x");
    Serial.print(I2C_Device_Address,HEX);
    Serial.println(" .");
  }
  else
  {
    I2C_Device_Address = I2C_DEVICE_ADDRESS_DEF;
    Serial.print("ERROR: NOT Address in EEPROM: 0x");
    Serial.print(I2C_Device_Address,HEX);
    Serial.println(" .");
    Serial.println("SET Address from program!");
  }
}
void SetGammaToEEPROM(void){
  //set Gamm in eeprom
  char rob;
  rob ='P';
  EEPROM.write(3, rob);
  rob ='R';
  EEPROM.write(4, rob);
  rob = Gamma_Value[0];
  EEPROM.write(5, rob);
  rob = Gamma_Value[1];
  EEPROM.write(6, rob);
  rob = Gamma_Value[2];
  EEPROM.write(7, rob);
}

void GetGammaFromEEPROM(void){
  if((EEPROM.read(3)=='P') && (EEPROM.read(4)=='R') )
  {
    Gamma_Value[0] = EEPROM.read(5);
    Gamma_Value[1] = EEPROM.read(6);
    Gamma_Value[2] = EEPROM.read(7);
    Serial.print("Gamma from EEPROM: ");
  }
  else
  {
    Gamma_Value[0] = 20;
    Gamma_Value[1] = 63;
    Gamma_Value[2] = 50;
    Serial.print("ERROR: Gamma from default: ");
  }

  Serial.print(Gamma_Value[0],DEC);
  Serial.print(" ,");
  Serial.print(Gamma_Value[1],DEC);
  Serial.print(" ,");
  Serial.print(Gamma_Value[2],DEC);
  Serial.println(" .");
}

/****************************************************
Main Functions zone
****************************************************/
void setup()
{
  Serial.begin(9600);
  
  Serial.println();
  Serial.println("-------------------------");
  Serial.println("Colorduino Firmware 2.0 .");
  GetI2CAddrFromEEPROM();
  GetGammaFromEEPROM();
  Serial.print("TWI_BUFFER_LENGTH: ");
  Serial.print(BUFFER_LENGTH);
  Serial.println(" .");
  Serial.println("-------------------------");
  
  _IO_Init();           //Init IO
  _LED_Init();          //Init LED Hardware
  _TC2_Init();          //Init Timer/Count2
  sei();  
  SetGamma();

  /*
  DispShowColor(50,50,50);
  delay(1000);
  DispShowChar('P',100,100,100,0);
  delay(1000);
  DispShowCharBG('P',0,0,0,0,100,100,100);
  delay(1000);
  */

  Wire.begin(I2C_Device_Address); // join i2c bus as slave
  Wire.onReceive(receiveEvent);   // define the receive function for receiving data from master
  Wire.setClock(400000L);
}

void loop()
{
  //We send R then G then B bytes as 3 separate transfers
  //This is because if we make the I2C buffer too large, we run out of SRAM for other things on our master Arduino

  //uint8_t dots[2][8][8][3] = {0};
  //[2]:Page:one for display, one for receive data
  //[8]:Row:8 row in LED plane
  //[8]:Column:8 column in one row
  //[3]:Color:RGB data: 0 for Red; 1 for green, 2 for Blue
  
  if (Wire.available()>66) { //when buffer full, process data. 66 =  start byte + colour + 64 pixel data + end byte
    
    // error check - make sure our data starts with the right byte   
    if (Wire.read() != START_OF_DATA) {
      //else handle error by reading remaining data until end of data marker (if available)
      while (Wire.available()>0 && Wire.read()!=END_OF_DATA) {}      
      return;
    }

    uint8_t c = Wire.read(); //read our color byte so we know if these are the R, G or B pixels.
    
    //depeding on c read pixels in as R G or B
    //read red display data
        
    if ((c >= 0) and (c<=3)){
      for (byte x = 0; x < 8; x++){
        for (byte y = 0; y < 8; y++){
           dots[Page_Write][x][y][c]= Wire.read();
        }
      }
    }
    if (c == 3) {
      // change i2c address
      if(Wire.read()=='P'){
        if(Wire.read()=='R'){
          SetI2cAddrToEEPROM(Wire.read());
        }
      }
        
      for (byte y = 0; y < 61; y++){
        Wire.read();
      }
    }
    if (c == 4) {
      // SetGamma();
      Gamma_Value[0] = Wire.read();
      Gamma_Value[1] = Wire.read();
      Gamma_Value[2] = Wire.read();
      
      SetGamma();
      for (byte y = 0; y < 61; y++){
        Wire.read();
      }
      SetGammaToEEPROM();
    }
    //read end of data marker
    if (Wire.read()==END_OF_DATA) {
    //if colour is blue, then update display
      if (c == 2){FlipPage();
      }
    } 
  }
}


void receiveEvent(int numBytes) {
  //do nothing here
}
