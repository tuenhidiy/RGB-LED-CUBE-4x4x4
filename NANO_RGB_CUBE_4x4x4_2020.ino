/*
  RGB LED CUBE 4x4x4
*/

#include <SPI.h>
#define latch_pin       2   // PORTD bit 2 - PD2
#define blank_pin       3   // PORTD bit 3 - PD3
#define data_pin        11  // MOSI
#define clock_pin       13  // CLOCK

#define layer0          4   // PORTD bit 4 - PD4
#define layer1          5   // PORTD bit 5 - PD5
#define layer2          6   // PORTD bit 6 - PD6
#define layer3          7   // PORTD bit 7 - PD7

int layerArray[4] = {layer0, layer1, layer2, layer3};
int lastAnode;

byte red[4][8];
byte green[4][8];
byte blue[4][8];

#define BAM_RESOLUTION 4    // EG 4 bit colour = 15 variation of R, G & B (4096 colours)
byte myred, mygreen, myblue;
uint8_t R, G, B;

#define myPI    3.14159265358979323846
#define myDPI   1.2732395
#define myDPI2  0.40528473

#define COLOUR_WHEEL_LENGTH 256

uint8_t colourR[COLOUR_WHEEL_LENGTH];
uint8_t colourG[COLOUR_WHEEL_LENGTH];
uint8_t colourB[COLOUR_WHEEL_LENGTH];

int16_t ColPos = 0;
uint16_t colourPos;
int16_t pos;
float X, Y, Z; 


#define REDC    0x0F,0x00,0x00
#define ORANGEC 0x0F,0x04,0x00
#define YELLOWC 0x0F,0x09,0x00
#define GREENC  0x00,0x0F,0x00
#define TEALC   0x00,0x0F,0x04
#define BLUEC   0x00,0x00,0x0F
#define PURPLEC 0x0F,0x00,0x0F
#define WHITEC  0x0F,0x0F,0x0F
#define CLEARC  0x00,0x00,0x00

#define CUBE_SIZE   4
#define ARRAY_SIZE(a) (sizeof(a)/sizeof(a[0]))
#define AXIS_X      1
#define AXIS_Y      2
#define AXIS_Z      3

unsigned long start;

int colorCount;
int x3, y3, z3,count3, mywait=200; 
float x,y,z, z1;
byte upDown[8][8];
int counter, mycolor;
float count;
enum plane_t
  {
    XYPLANE,
    XZPLANE,
    YZPLANE
  };

typedef uint8_t     type_polarity;

enum polarity_t : uint8_t
  {
    POSITIVE,
    NEGATIVE,
  };

typedef struct{
float x;
float y;
float z;
} point;

uint8_t cube[4][4];

//************************************************************************************************************//

#define RGB(red, green, blue) (rgb) { red, green, blue }

int level=0;
int anodeLevel=0;
int BAM_Bit, BAM_Counter=0;

void setup()
{
SPI.setBitOrder(MSBFIRST);
SPI.setDataMode(SPI_MODE0);
SPI.setClockDivider(SPI_CLOCK_DIV2);
noInterrupts();
//We use Timer 1 to refresh the cube
TCCR1A = B00000000;
TCCR1B = B00001011;
TIMSK1 = B00000010;
OCR1A=12;
//  our clock runs at 250kHz, which is 1/250kHz = 4us
//  with OCR1A set to 12, this means the interrupt will be called every (12+1)x4us=52us, 

pinMode(latch_pin, OUTPUT); // turn off PWM and set PortD bit 4 as output
pinMode(data_pin, OUTPUT);  //MOSI DATA
pinMode(clock_pin, OUTPUT); //SPI Clock

pinMode(layer0, OUTPUT);
pinMode(layer1, OUTPUT);
pinMode(layer2, OUTPUT);
pinMode(layer3, OUTPUT);

SPI.begin();
interrupts();
fill_colour_wheel();
clearfast();
}

void loop()
{
for (byte x=0; x<4; x++)
  {
    for (byte y=0; y<4; y++)
    {
      for (byte z=0; z<4; z++)
      {
        LED(z,y,x,15,9,0);
        delay(300);
      }
    }
  }
delay(5000);
clearfast();
RandomFall();
clearfast();
RandomFall();
clearfast();
fillcube_hue(10);
clearfast();
rainboww();
clearfast();
sinwaveTwo_colorwheel();
clearfast();
clearfast();
crazy_straw_changecolor(1000, 50);
clearfast();
Cosine(60);
clearfast();
for(int i=0;i<4;i++)
{
  Droplets_colorwheel();
  clearfast();
}
clearfast();
sendvoxels_rand_z_colorwheel(100, 800, 2000);
clearfast();
EffectPlaneTwist();
clearfast();
RandomFleaJumps(random(46));
clearfast();
flagwave(0, 100);
clearfast();
sidewaves_colorwheel(800, 80);// OK - Đẹp
clearfast();
boingboing(200, 1500, 0x01, 0x01);
boingboing(200, 1500, 0x02, 0x03);
clearfast();
bomb(1000, 15);
clearfast();
squarespiral(100, 1000);
clearfast();

shrinkCube_random(16);  
clearfast();
effect_boxside_randsend_parallel_colorwheel (AXIS_Z, 0 , 1000, 1);
delay_ms(1500);
effect_boxside_randsend_parallel_colorwheel (AXIS_Z, 1 , 1000, 1);
delay_ms(1500);
    
effect_boxside_randsend_parallel_colorwheel (AXIS_X, 0 , 1000, 1);
delay_ms(1500);
effect_boxside_randsend_parallel_colorwheel (AXIS_X, 1 , 1000, 1);
delay_ms(1500);
      
effect_boxside_randsend_parallel_colorwheel (AXIS_Y, 0 , 1000, 1);
delay_ms(1500);
effect_boxside_randsend_parallel_colorwheel (AXIS_Y, 1 , 1000, 1);
delay_ms(1500);

effect_boxside_randsend_parallel_colorwheel (AXIS_Z, 0 , 1000, 2);
delay_ms(1500);
effect_boxside_randsend_parallel_colorwheel (AXIS_Z, 1 , 1000, 2);
delay_ms(1500);
   
effect_boxside_randsend_parallel_colorwheel (AXIS_X, 0 , 1000, 2);
delay_ms(1500);
effect_boxside_randsend_parallel_colorwheel (AXIS_X, 1 , 1000, 2);
delay_ms(1500);
      
effect_boxside_randsend_parallel_colorwheel (AXIS_Y, 0 , 1000, 2);
delay_ms(1500);
effect_boxside_randsend_parallel_colorwheel (AXIS_Y, 1 , 1000, 2);
delay_ms(1500);

clearfast();
rainbow1(20);
clearfast();
walk_through_walls(500, 4);
clearfast();
rainbow2(20);
clearfast();
RandomFall();
clearfast();
RandomFall();
clearfast();
effect_boxside_randsend_parallel (AXIS_Z, 0 , 2000, 1, 0);
delay_ms(1500);
effect_boxside_randsend_parallel (AXIS_Z, 1 , 2000, 1, 0);
delay_ms(1500);
      
effect_boxside_randsend_parallel (AXIS_X, 0 , 2000, 1, 30);
delay_ms(1500);
effect_boxside_randsend_parallel (AXIS_X, 1 , 2000, 1, 30);
delay_ms(1500);
      
effect_boxside_randsend_parallel (AXIS_Y, 0 , 2000, 1, 38);
delay_ms(1500);
effect_boxside_randsend_parallel (AXIS_Y, 1 , 2000, 1, 38);
delay_ms(1500);

effect_boxside_randsend_parallel (AXIS_Z, 0 , 2000, 2, 15);
delay_ms(1500);
effect_boxside_randsend_parallel (AXIS_Z, 1 , 2000, 2, 15);
delay_ms(1500);
      
effect_boxside_randsend_parallel (AXIS_X, 0 , 2000, 2, 28);
delay_ms(1500);
effect_boxside_randsend_parallel (AXIS_X, 1 , 2000, 2, 28);
delay_ms(1500);
      
effect_boxside_randsend_parallel (AXIS_Y, 0 , 2000, 2, 46);
delay_ms(1500);
effect_boxside_randsend_parallel (AXIS_Y, 1 , 2000, 2, 46);
delay_ms(1500);

clearfast();
effect_axis_updown_randsuspend(AXIS_Z, 2000,5000, 0, 15 );
effect_axis_updown_randsuspend(AXIS_Z, 2000,5000, 1, 15);
effect_axis_updown_randsuspend(AXIS_X, 2000,5000, 0, 25);
effect_axis_updown_randsuspend(AXIS_X, 2000,5000, 1, 25);
effect_axis_updown_randsuspend(AXIS_Y, 2000,5000, 0, 35);
effect_axis_updown_randsuspend(AXIS_Y, 2900,5000, 1, 35);
clearfast();

int  time = 20;
for(int red = 0; red <= 15; red++)
{
  fillCube(red, 0,0);
  delay(time);
}
colorMorph(time);
for(int red = 15; red >= 0; red--)
{
  fillCube(red, 0,0);
  delay(time);
  }
delay(500);
clearfast();
}

void LED(int CZ, int CY, int CX, int CR, int CG, int CB) 
{
  CR = constrain(CR, 0, (1 << BAM_RESOLUTION) - 1);
  CG = constrain(CG, 0, (1 << BAM_RESOLUTION) - 1);
  CB = constrain(CB, 0, (1 << BAM_RESOLUTION) - 1);

  int WhichByte = int(((CZ<<4) + (CY<<2) + CX) >> 3);
  int WhichBit = (((CZ<<4) + (CY<<2) + CX) - (WhichByte << 3));

  if (inrange(CZ, CY, CX))
  { 
    for (byte I = 0; I < BAM_RESOLUTION; I++) 
    {
      //*** RED ***
      bitWrite(red[I][WhichByte], WhichBit, bitRead(CR, I));

      //*** GREEN ***
      bitWrite(green[I][WhichByte], WhichBit, bitRead(CG, I));

      //*** BLUE ***
      bitWrite(blue[I][WhichByte], WhichBit, bitRead(CB, I));
    }
  }
}

ISR(TIMER1_COMPA_vect)
{
  PORTD |= 1<<blank_pin;
  if(BAM_Counter==6)
  BAM_Bit++;
  else
  if(BAM_Counter==18)
  BAM_Bit++;
  else
  if(BAM_Counter==42)
  BAM_Bit++;
  BAM_Counter++;
  switch (BAM_Bit)
  {
  case 0:
       //Blue       
        SPI.transfer(blue[0][level + 1]);
        SPI.transfer(blue[0][level]);
      //Green
        SPI.transfer(green[0][level + 1]);
        SPI.transfer(green[0][level]);      
      //Red
        SPI.transfer(red[0][level + 1]);
        SPI.transfer(red[0][level]);
      break;
    case 1:
      //Blue
        SPI.transfer(blue[1][level + 1]);
        SPI.transfer(blue[1][level]);       
      //Green
        SPI.transfer(green[1][level + 1]);
        SPI.transfer(green[1][level]);
      //Red
        SPI.transfer(red[1][level + 1]);
        SPI.transfer(red[1][level]);
      break;
    case 2:
      //Blue
        SPI.transfer(blue[2][level + 1]);
        SPI.transfer(blue[2][level]);
       //Green
        SPI.transfer(green[2][level + 1]);
        SPI.transfer(green[2][level]);              
      //Red
        SPI.transfer(red[2][level + 1]);
        SPI.transfer(red[2][level]);              

      break;
    case 3:
      //Blue
        SPI.transfer(blue[3][level + 1]);
        SPI.transfer(blue[3][level]);
      //Green
        SPI.transfer(green[3][level + 1]);
        SPI.transfer(green[3][level]);
      //Red
        SPI.transfer(red[3][level + 1]);
        SPI.transfer(red[3][level]);
    if(BAM_Counter==90)
    {
      BAM_Counter=0;
      BAM_Bit=0;
    }
  break;
  }
  lastAnode = (anodeLevel-1);
  if (anodeLevel == 0) 
    {
      lastAnode = 3;
      //PORTD &=~_BV(layerArray[lastAnode]);  // turn on the previous layer
      }            
  PORTD &= ~_BV(layerArray[lastAnode]);   // turn off the previous layer
  PORTD |=  _BV(layerArray[anodeLevel]);  // turn on the previous layer  

  PORTD |= 1<<latch_pin;
  //delayMicroseconds(3);
  PORTD &= ~(1<<latch_pin);
  delayMicroseconds(3);
  PORTD &= ~(1<<blank_pin);
  //delayMicroseconds(3);
  
  anodeLevel++;
  level = anodeLevel<<1;
  if(anodeLevel == 4) anodeLevel=0;
  if(level==8) level=0;
  DDRD |= 1<<blank_pin;

}

void clearfast()
{
   memset(red, 0, sizeof(red[0][0]) * 4 * 8);
   memset(green, 0, sizeof(green[0][0]) * 4 * 8);
   memset(blue, 0, sizeof(blue[0][0]) * 4 * 8);
}

void delay_ms(uint16_t x)
{
  uint8_t y, z;
  for ( ; x > 0 ; x--){
    for ( y = 0 ; y < 90 ; y++){
      for ( z = 0 ; z < 6 ; z++){
        asm volatile ("nop");
      }
    }
  }
}

void getColor(int z, int y, int x, byte colorArray[]) {
  // Check parameters
  x = checkConstrains(x, 0, 3);
  y = checkConstrains(y, 0, 3);
  z = checkConstrains(z, 0, 3);

  int wholebyte = (z << 4)+(y << 2) + x;
  int whichbyte = int(wholebyte >> 3);
  byte whichbit = wholebyte - (whichbyte << 3);

  // RED
  colorArray[0] = 0;
  bitWrite(colorArray[0], 0, bitRead(red[0][whichbyte], whichbit));
  bitWrite(colorArray[0], 1, bitRead(red[1][whichbyte], whichbit));
  bitWrite(colorArray[0], 2, bitRead(red[2][whichbyte], whichbit));
  bitWrite(colorArray[0], 3, bitRead(red[3][whichbyte], whichbit));

  // GREEN
  colorArray[1] = 0;
  bitWrite(colorArray[1], 0, bitRead(green[0][whichbyte], whichbit));
  bitWrite(colorArray[1], 1, bitRead(green[1][whichbyte], whichbit));
  bitWrite(colorArray[1], 2, bitRead(green[2][whichbyte], whichbit));
  bitWrite(colorArray[1], 3, bitRead(green[3][whichbyte], whichbit));

  // BLUE
  colorArray[2] = 0;
  bitWrite(colorArray[2], 0, bitRead(blue[0][whichbyte], whichbit));
  bitWrite(colorArray[2], 1, bitRead(blue[1][whichbyte], whichbit));
  bitWrite(colorArray[2], 2, bitRead(blue[2][whichbyte], whichbit));
  bitWrite(colorArray[2], 3, bitRead(blue[3][whichbyte], whichbit));
}

void argorder(int ix1, int ix2, int *ox1, int *ox2)
{
  if (ix1>ix2)
  {
    int tmp;
    tmp = ix1;
    ix1= ix2;
    ix2 = tmp;
  }
  *ox1 = ix1;
  *ox2 = ix2;
}

unsigned char inrange(int z, int y, int x)
{
  if (x >= 0 && x < 4 && y >= 0 && y < 4 && z >= 0 && z < 4)
  {
    return 1;
  } else
  {
    // One of the coordinates was outside the cube.
    return 0;
  }
}

void Swap(int8_t *x,int8_t *y)
  {
    int8_t t;
    t=*x;
    *x = *y;
    *y = t;
  }

int8_t Wrap(int8_t val)
{
  if (val>3)
    return 0;
  else if (val<0)
    return 3;
  else
    return val;
}

int8_t Crop(int8_t val)
{
  if (val>3)
    return 3;
  else if (val<0)
    return 0;
  else
    return val;
}

// Returns a byte with a row of 1's drawn in it.
// byteline(2,5) gives 0b00111100
char byteline (int start, int end)
{
  return ((0xff<<start) & ~(0xff<<(end+1)));
}
//*******************************************************MK4*****************************************************//

//FAST SINE APPROX
float mySin(float x){
  float sinr = 0;
  uint8_t g = 0;

  while(x > myPI){
    x -= 2*myPI; 
    g = 1;
  }

  while(!g&(x < -myPI)){
    x += 2*myPI;
  }

  sinr = myDPI*x - myDPI2*x*myAbs(x);
  sinr = 0.225*(sinr*myAbs(sinr)-sinr)+sinr;

  return sinr;
}

//FAST COSINE APPROX
float myCos(float x){
  return mySin(x+myPI/2);
}

float myTan(float x){
  return mySin(x)/myCos(x);
}

//SQUARE ROOT APPROX
float mySqrt(float in){
  int16_t d = 0;
  int16_t in_ = in;
  float result = 2;
  
  for(d = 0; in_ > 0; in_ >>= 1){
    d++;
  }
  
  for(int16_t i = 0; i < d/2; i++){
    result = result*2;
  }
  
  for(int16_t i = 0; i < 3; i++){
    result = 0.5*(in/result + result);
  }
  
  return result;
}

//MAP NUMBERS TO NEW RANGE
float myMap(float in, float inMin, float inMax, float outMin, float outMax){
  float out;
  out = (in-inMin)/(inMax-inMin)*(outMax-outMin) + outMin;
  return out;
}

//ROUND A NUMBER
int16_t myRound(float in){
  int8_t s = in/myAbs(in);
  return (int16_t)(s*(myAbs(in) + 0.5));
}

//ABSOLUTE VALUE
float myAbs(float in){
  return (in)>0?(in):-(in);
} 


void fill_colour_wheel(void) 
{
  float red, green, blue;
  float c, s;
  int32_t phase = 0;
  int16_t I = 0;

  while (phase < COLOUR_WHEEL_LENGTH) 
  {
    s = (1 << BAM_RESOLUTION)*mySin(myPI*(3 * phase - I*COLOUR_WHEEL_LENGTH) / (2 * COLOUR_WHEEL_LENGTH));
    c = (1 << BAM_RESOLUTION)*myCos(myPI*(3 * phase - I*COLOUR_WHEEL_LENGTH) / (2 * COLOUR_WHEEL_LENGTH));

    red = (I == 0 ? 1 : 0)*s + (I == 1 ? 1 : 0)*c;
    green = (I == 1 ? 1 : 0)*s + (I == 2 ? 1 : 0)*c;
    blue = (I == 2 ? 1 : 0)*s + (I == 0 ? 1 : 0)*c;

    colourR[phase] = red;
    colourG[phase] = green;
    colourB[phase] = blue;

    if (++phase >= (1 + I)*COLOUR_WHEEL_LENGTH / 3) 
      I++;
  }
}

void get_colour(int16_t p, uint8_t *R, uint8_t *G, uint8_t *B)
{
  if (p >= COLOUR_WHEEL_LENGTH)
    p -= COLOUR_WHEEL_LENGTH;

  *R = colourR[p];
  *G = colourG[p];
  *B = colourB[p];
}

void get_next_colour(uint8_t *R, uint8_t *G, uint8_t *B)
{
  if (++ColPos >= COLOUR_WHEEL_LENGTH)
    ColPos -= COLOUR_WHEEL_LENGTH;

  *R = colourR[ColPos];
  *G = colourG[ColPos];
  *B = colourB[ColPos];
}

void increment_colour_pos(uint8_t i)
{
  colourPos += i;
  while (colourPos >= COLOUR_WHEEL_LENGTH)
  {
    colourPos -= COLOUR_WHEEL_LENGTH;
  }
}

int checkConstrains(int value, int min, int max) {
  if(value < min) {
    return min;
  } else if (value > max) {
    return max;
  } else {
    return value;
  }
}

void getColor(int color, int intensity) {
  if (color<16){
    mygreen = color;
    myred = 15-mygreen;
    myblue = 0;
  }
  if ((color>15)&& (color<31)){
    myblue = color-15;
    mygreen = 15-myblue;
    myred = 0;
  }
  if ((color>30)&& (color<46)){
    myred = color-30;
    myblue = 15-myred;
    mygreen = 0;
  }
  if (color==46){
    myred = 15;
    myblue = 15;
    mygreen = 15;
  }
  if (color==47){
    myred = 0;
    myblue = 0;
    mygreen = 0;
  }
  if (color==48){//
    myred = 15;
    mygreen = 4;
    myblue = 0;
  }
  if (color==49){
    myred = 15;
    mygreen = 9;
    myblue = 0;
  }
   if (color==50){
    myred = 9;
    mygreen = 0;    
    myblue = 15;   
  } 

  if (intensity <5){
    myred=myred >> 1;
    mygreen=mygreen >> 1;
    myblue=myblue >> 1;
  }
  if (intensity <4){
    myred=myred >> 1;
    mygreen=mygreen >> 1;
    myblue=myblue >> 1;
  }
  if (intensity <3){
    myred=myred >> 1;
    mygreen=mygreen >> 1;
    myblue=myblue >> 1;
  }
  if (intensity <2){
    myred=myred >> 1;
    mygreen=mygreen >> 1;
    myblue=myblue >> 1;
  }
  if (intensity <1){
    myred=0;
    mygreen=0;
    myblue=0;
  }
}

void hueToRGB(int hue, int brightness, uint8_t *oR, uint8_t *oG, uint8_t *oB)
{
  unsigned int scaledHue = (hue * 6); 
  unsigned int segment = scaledHue / 16;
  unsigned int segmentOffset = scaledHue - (segment * 16);
  unsigned int complement = 0;
  unsigned int prev = (brightness * (15 - segmentOffset)) / 16;
  unsigned int next = (brightness *  segmentOffset) / 16;

  switch (segment) 
  {
    case 0:      // red
      *oR = brightness;
      *oG = next;
      *oB = complement;
      break;
    case 1:     // yellow
      *oR = prev;
      *oG = brightness;
      *oB = complement;
      break;
    case 2:     // green
      *oR = complement;
      *oG = brightness;
      *oB = next;
      break;
    case 3:    // cyan
      *oR = complement;
      *oG = prev;
      *oB = brightness;
      break;
    case 4:    // blue
      *oR = next;
      *oG = complement;
      *oB = brightness;
      break;
    case 5:      // magenta
    default:
      *oR = brightness;
      *oG = complement;
      *oB = prev;
      break;
  }
}

void colorMorph(int time) {
  int red, green, blue;
  int keepColorTime = time * 150;
  
  delay(keepColorTime);
  // RED + GREEN
  for(int green = 0; green <= 15; green++) {
    fillCube(15, green, 0);
    delay(time);
  }
  delay(keepColorTime);
  // GREEN - RED
  for(int red = 15; red >= 0; red --) {
    fillCube(red, 15, 0);
    delay(time);
  }
  delay(keepColorTime);
  // GREEN + BLUE
  for(int blue = 0; blue <= 15; blue++) {
    fillCube(0, 15, blue);
    delay(time);
  }
  delay(keepColorTime);
  // BLUE - GREEN
  for(int green = 15; green >= 0; green --) {
    fillCube(0, green, 15);
    delay(time);
  }
  delay(keepColorTime);
  // BLUE + RED
  for(int red = 0; red <= 15; red++) {
    fillCube(red, 0,15);
    delay(time);
  }
  delay(keepColorTime);
  // RED - BLUE + GREEN
  green = 0;
  for(int blue = 15; blue >= 0; blue --) {
    fillCube(15, green, blue);
    delay(time);
    green++;
  }
  delay(keepColorTime);
  // GREEN - RED + BLUE
  blue = 0;
  for(int red = 15; red >= 0; red --) {
    fillCube(red, 15, blue);
    delay(time);
    blue++;
  }
  delay(keepColorTime);
  // GREEN + RED + BLUE
  for(int red = 0; red <= 15; red++) {
    fillCube(red, 15, 15);
    delay(time);
  }
  delay(keepColorTime);
  // RED - GREEN - BLUE
  blue = 15;
  for(int green = 15; green >= 0; green --) {
    fillCube(15, green, blue);
    delay(time);
    blue--;
  }
}

void fillCube(byte R, byte G, byte B)
{
  for (byte z=0; z<4; z++)
  {
    for (byte y=0; y<4; y++)
    {
      for (byte x=0; x<4; x++)
      {
        LED(z, y, x, R, G, B);
      }
    }
  }
}

void draw_positions_axis (char axis, unsigned char positions[16], int invert, int xColor)
{
  int x, y, z, p;
  byte tempcolor[3]; 
 
  getColor(xColor, 4);
  clearfast();
  for (x=0; x<4; x++)
  {
    for (y=0; y<4; y++)
    {
      if (invert)
      {
        p = (3-positions[(x*4)+y]);
      } else
      {
        p = positions[(x*4)+y];
      }
    
      if (axis == AXIS_Z)
        LED(p,y,x, myred, mygreen, myblue);
        
      if (axis == AXIS_Y)
        LED(y,p,x, myred, mygreen, myblue);
        
      if (axis == AXIS_X)
        LED(x,y,p, myred, mygreen, myblue);
    }
  }
  
}

void effect_axis_updown_randsuspend (char axis, int delay, int sleep, int invert, int xColor)
{
  unsigned char positions[16];
  unsigned char destinations[16];

  int i,px;
  
  // Set 16 random positions
  for (i=0; i<16; i++)
  {
    positions[i] = 0; // Set all starting positions to 0
    destinations[i] = rand()%4;
  }

  // Loop 4 times to allow destination 3 to reach all the way
  for (i=0; i<4; i++)
  {
    // For every iteration, move all position one step closer to their destination
    for (px=0; px<16; px++)
    {
      if (positions[px]<destinations[px])
      {
        positions[px]++;
      }
    }
    // Draw the positions and take a nap
    draw_positions_axis (axis, positions,invert,xColor);
    delay_ms(delay);
  }
  
  // Set all destinations to 3 (opposite from the side they started out)
  for (i=0; i<16; i++)
  {
    destinations[i] = 3;
  }
  
  // Suspend the positions in mid-air for a while
  delay_ms(sleep);
  
  // Then do the same thing one more time
  for (i=0; i<4; i++)
  {
    for (px=0; px<16; px++)
    {
      if (positions[px]<destinations[px])
      {
        positions[px]++;
      }
      if (positions[px]>destinations[px])
      {
        positions[px]--;
      }
    }
    draw_positions_axis (axis, positions,invert,xColor);
    delay_ms(delay);
  }
}

void effect_boxside_randsend_parallel (char axis, int origin, int delay, int mode, int xColor)
{
  int i;
  int done;
  unsigned char cubepos[16];
  unsigned char pos[16];
  int notdone = 1;
  int notdone2 = 1;
  int sent = 0;
  
  for (i=0;i<16;i++)
  {
    pos[i] = 0;
  }
  
  while (notdone)
  {
    if (mode == 1)
    {
      notdone2 = 1;
      while (notdone2 && sent<16)
      {
        i = rand()%16;
        if (pos[i] == 0)
        {
          sent++;
          pos[i] += 1;
          notdone2 = 0;
        }
      }
    } else if (mode == 2)
    {
      if (sent<16)
      {
        pos[sent] += 1;
        sent++;
      }
    }
    
    done = 0;
    for (i=0;i<16;i++)
    {
      if (pos[i] > 0 && pos[i] <3)
      {
        pos[i] += 1;
      }
        
      if (pos[i] == 3)
        done++;
    }
    
    if (done == 16)
      notdone = 0;
    
    for (i=0;i<16;i++)
    {
      if (origin == 0)
      {
        cubepos[i] = pos[i];
      } else
      {
        cubepos[i] = (3-pos[i]);
      }
    }
    
    
    delay_ms(delay);
    draw_positions_axis(axis,cubepos,0,xColor);
    //LED_PORT ^= LED_RED;
  }
  
}

void RandomFall() {
  
    for (int x=0;x<4;x++){
      for (int y=0;y<4;y++){
        LED (3, y, x, 15,0,0); 
      }
  } 
  delay(500);
  while(count3<16){
    x3= random(4); 
    y3= random(4);
    if (upDown[x3][y3]==0){
      upDown[x3][y3]=1;
      count3++;
      mycolor=10;
      getColor(mycolor, 4);
      for (z=3; z>0; z--){
        LED(z, y3, x3, 0,0,0);
        LED(z-1, y3, x3, myred, mygreen, myblue);
        delay((z*mywait)/5);
        mycolor=mycolor+5;
        getColor(mycolor, 4);
      }
    }
  }
  delay(500);
  count3=0;

  while(count3<16){
    x3= random(4); 
    y3= random(4);
    if (upDown[x3][y3]==1){
      upDown[x3][y3]=0;
      count3++;
      mycolor=45;
      getColor(mycolor, 4);
      for (z=0; z<3; z++){
        LED(z, y3, x3, 0,0,0);
        LED(z+1, y3, x3, myred, mygreen, myblue);
        delay(mywait/5);
        mycolor=mycolor-5;
        getColor(mycolor, 4);
      }
    }
  }
   delay(500);
  count3=0;
  delay(500);
  clearfast();
}

void line_3d (int x1, int y1, int z1, int x2, int y2, int z2, int xColor)
{
  int i, dx, dy, dz, l, m, n, x_inc, y_inc, z_inc,
  err_1, err_2, dx2, dy2, dz2;
  int pixel[3];
  getColor(xColor,4);
  pixel[0] = x1;
  pixel[1] = y1;
  pixel[2] = z1;
  dx = x2 - x1;
  dy = y2 - y1;
  dz = z2 - z1;
  x_inc = (dx < 0) ? -1 : 1;
  l = abs(dx);
  y_inc = (dy < 0) ? -1 : 1;
  m = abs(dy);
  z_inc = (dz < 0) ? -1 : 1;
  n = abs(dz);
  dx2 = l << 1;
  dy2 = m << 1;
  dz2 = n << 1;
  if ((l >= m) && (l >= n)) {
  err_1 = dy2 - l;
  err_2 = dz2 - l;
  for (i = 0; i < l; i++) {
  LED(pixel[2],pixel[1],pixel[0], myred, mygreen, myblue);
  if (err_1 > 0) {
  pixel[1] += y_inc;
  err_1 -= dx2;
  }
  if (err_2 > 0) {
  pixel[2] += z_inc;
  err_2 -= dx2;
  }
  err_1 += dy2;
  err_2 += dz2;
  pixel[0] += x_inc;
  }
  } else if ((m >= l) && (m >= n)) {
  err_1 = dx2 - m;
  err_2 = dz2 - m;
  for (i = 0; i < m; i++) {
  LED(pixel[2],pixel[1],pixel[0], myred, mygreen, myblue);
  if (err_1 > 0) {
  pixel[0] += x_inc;
  err_1 -= dy2;
  }
  if (err_2 > 0) {
  pixel[2] += z_inc;
  err_2 -= dy2;
  }
  err_1 += dx2;
  err_2 += dz2;
  pixel[1] += y_inc;
  }
  } else {
  err_1 = dy2 - n;
  err_2 = dx2 - n;
  for (i = 0; i < n; i++) {
  LED(pixel[2],pixel[1],pixel[0], myred, mygreen, myblue);
  if (err_1 > 0) {
  pixel[1] += y_inc;
  err_1 -= dz2;
  }
  if (err_2 > 0) {
  pixel[0] += x_inc;
  err_2 -= dz2;
  }
  err_1 += dy2;
  err_2 += dx2;
  pixel[2] += z_inc;
  }
  }
  LED(pixel[2],pixel[1],pixel[0], myred, mygreen, myblue);
}

void line_3d_colorwheel (int x1, int y1, int z1, int x2, int y2, int z2)
{
  int i, dx, dy, dz, l, m, n, x_inc, y_inc, z_inc,
  err_1, err_2, dx2, dy2, dz2;
  int pixel[3];
  uint8_t R, G, B;
  pixel[0] = x1;
  pixel[1] = y1;
  pixel[2] = z1;
  dx = x2 - x1;
  dy = y2 - y1;
  dz = z2 - z1;
  x_inc = (dx < 0) ? -1 : 1;
  l = abs(dx);
  y_inc = (dy < 0) ? -1 : 1;
  m = abs(dy);
  z_inc = (dz < 0) ? -1 : 1;
  n = abs(dz);
  dx2 = l << 1;
  dy2 = m << 1;
  dz2 = n << 1;
  if ((l >= m) && (l >= n)) {
  err_1 = dy2 - l;
  err_2 = dz2 - l;
  for (i = 0; i < l; i++) {
  get_colour(colourPos+10*pixel[2], &R, &G, &B);
  LED(pixel[2],pixel[1],pixel[0], R, G, B);
  if (err_1 > 0) {
  pixel[1] += y_inc;
  err_1 -= dx2;
  }
  if (err_2 > 0) {
  pixel[2] += z_inc;
  err_2 -= dx2;
  }
  err_1 += dy2;
  err_2 += dz2;
  pixel[0] += x_inc;
  }
  } else if ((m >= l) && (m >= n)) {
  err_1 = dx2 - m;
  err_2 = dz2 - m;
  for (i = 0; i < m; i++) {
  get_colour(colourPos+10*pixel[2], &R, &G, &B);
  LED(pixel[2],pixel[1],pixel[0], R, G, B);
  if (err_1 > 0) {
  pixel[0] += x_inc;
  err_1 -= dy2;
  }
  if (err_2 > 0) {
  pixel[2] += z_inc;
  err_2 -= dy2;
  }
  err_1 += dx2;
  err_2 += dz2;
  pixel[1] += y_inc;
  }
  } else {
  err_1 = dy2 - n;
  err_2 = dx2 - n;
  for (i = 0; i < n; i++) {
  get_colour(colourPos+10*pixel[2], &R, &G, &B);
  LED(pixel[2],pixel[1],pixel[0], R, G, B);
  if (err_1 > 0) {
  pixel[1] += y_inc;
  err_1 -= dz2;
  }
  if (err_2 > 0) {
  pixel[0] += x_inc;
  err_2 -= dz2;
  }
  err_1 += dy2;
  err_2 += dx2;
  pixel[2] += z_inc;
  }
  }
  get_colour(colourPos+10*pixel[2], &R, &G, &B);
  LED(pixel[2],pixel[1],pixel[0], R, G, B);
}


void crazy_straw_changecolor(int delay, int iterations)
{
  int8_t  i, j, p=0, pp=0,
          xStart=0, yStart=0, zStart=0,
          xEnd=0, yEnd=0, zEnd=0,
          xDest=0, yDest=0, zDest=0,
          xx, yy, zz;

  //start point & straw random generation
  if(rand()%2) xStart=0;
  else xStart=CUBE_SIZE-1;
  
  if(rand()%2) yStart=0;
  else yStart=CUBE_SIZE-1;
  
  if(rand()%2) zStart=0;
  else zStart=CUBE_SIZE-1;
  
  if(xStart>0) xEnd=0;
  else xEnd=CUBE_SIZE-1;
  yEnd=yStart;
  zEnd=zStart;
  
  for(i=0;i<iterations;i++)
  {
    while(1)
    {
      colorCount=colorCount+2;  // change color with each pass
      if (colorCount>46)
        {
        colorCount=2;
        }
      xx=xEnd;
      yy=yEnd;
      zz=zEnd;
            
      do p=rand()%3;
      while(pp==p);
      pp=p;
      
      if(p==0)
      {
        if(xEnd==0) xx=CUBE_SIZE-1;
        else xx=0;
      }
      if(p==1)
      {
        if(yEnd==0) yy=CUBE_SIZE-1;
        else yy=0;
      }
      if(p==2)
      {
        if(zEnd==0) zz=CUBE_SIZE-1;
        else zz=0;
      }
      
      if((xx==xStart && yy==yStart && zz==zStart) || (xx==xEnd && yy==yEnd && zz==zEnd)) continue;
      else
      {
        xDest=xx;
        yDest=yy;
        zDest=zz;
        break;
      } 
    }
    
    for(j=0;j<CUBE_SIZE;j++)
    {
      if(xDest!=xEnd)
      {
        if(xEnd==0) xx=j;
        if(xEnd==CUBE_SIZE-1) xx=xEnd-j;
      }
      
      if(yDest!=yEnd)
      {
        if(yEnd==0) yy=j;
        if(yEnd==CUBE_SIZE-1) yy=yEnd-j;
      }
      
      if(zDest!=zEnd)
      {
        if(zEnd==0) zz=j;
        if(zEnd==CUBE_SIZE-1) zz=zEnd-j;
      }
      
      line_3d(xStart,yStart,zStart,xx,yy,zz, colorCount);
      delay_ms(delay);
      clearfast();
      
      if(j==CUBE_SIZE-1)
      {
        xEnd=xStart;
        yEnd=yStart;
        zEnd=zStart;
        xStart=xx;
        yStart=yy;
        zStart=zz;
      }
    }
  }
}

void sinwaveTwo()
{
  int sinewavearray[4], addr, sinemult[4], colselect, rr=0, gg=0, bb=15, addrt;
  int sinewavearrayOLD[4], select, subZ=-3, subT=3, multi=0;//random(-1, 2);
  sinewavearray[0]=0;
  sinemult[0]=1;
  sinewavearray[1]=1;
  sinemult[1]=1; 
  sinewavearray[2]=2;
  sinemult[2]=1;
  sinewavearray[3]=3;
  sinemult[3]=1;
     
  start=millis();
      
  while(millis()-start<15000)
  {
    for(addr=0; addr<4; addr++)
    {
      if(sinewavearray[addr]==3)
      {
        sinemult[addr]=-1;
      }
      if(sinewavearray[addr]==0)
      {
        sinemult[addr]=1;     
      }
      sinewavearray[addr] = sinewavearray[addr] + sinemult[addr];
    }//addr
  if(sinewavearray[0]==3)
  {
    select=random(3);
    if(select==0)
    {
      rr=random(1, 16);
      gg=random(1, 16);
      bb=0;
    } 
    if(select==1)
    {
      rr=random(1, 16);
      gg=0;
      bb=random(1, 16);
    }    
    if(select==2)
     {
      rr=0;
      gg=random(1, 16);
      bb=random(1, 16);
      }
  }
   
  for(addr=0; addr<4; addr++)
  {
    LED(sinewavearrayOLD[addr], addr, 0, 0, 0, 0);
    LED(sinewavearrayOLD[addr], 0, addr, 0, 0, 0);
    LED(sinewavearrayOLD[addr], subT-addr, 3, 0, 0, 0);
    LED(sinewavearrayOLD[addr], 3, subT-addr, 0, 0, 0);     
    LED(sinewavearray[addr], addr, 0, rr, gg, bb);
    LED(sinewavearray[addr], 0, addr, rr, gg, bb);
    LED(sinewavearray[addr], subT-addr, 3, rr, gg, bb);
    LED(sinewavearray[addr], 3, subT-addr, rr, gg, bb);
  }//}
    
  for(addr=1; addr<3; addr++)
  {   
    LED(sinewavearrayOLD[addr+multi*1], addr, 1, 0, 0, 0);
    LED(sinewavearrayOLD[addr+multi*1], 1, addr, 0, 0, 0);
    LED(sinewavearrayOLD[addr+multi*1], subT-addr, 2, 0, 0, 0);
    LED(sinewavearrayOLD[addr+multi*1], 2, subT-addr, 0, 0, 0);  
    LED(sinewavearray[addr+multi*1], addr, 1, rr, gg, bb);
    LED(sinewavearray[addr+multi*1], 1, addr, rr, gg, bb);
    LED(sinewavearray[addr+multi*1], subT-addr, 2, rr, gg, bb);
    LED(sinewavearray[addr+multi*1], 2, subT-addr, rr, gg, bb);
  }
 
  for(addr=2; addr<2; addr++)
  {   
    LED(sinewavearrayOLD[addr+multi*2], addr, 2, 0, 0, 0);
    LED(sinewavearrayOLD[addr+multi*2], 2, addr, 0, 0, 0);
    LED(sinewavearrayOLD[addr+multi*2], subT-addr, 1, 0, 0, 0);
    LED(sinewavearrayOLD[addr+multi*2], 1, subT-addr, 0, 0, 0);  
    LED(sinewavearray[addr+multi*2], addr, 2, rr, gg, bb);
    LED(sinewavearray[addr+multi*2], 2, addr, rr, gg, bb);
    LED(sinewavearray[addr+multi*2], subT-addr, 1, rr, gg, bb);
    LED(sinewavearray[addr+multi*2], 1, subT-addr, rr, gg, bb);
  }  
       
  for(addr=0; addr<4; addr++)
  sinewavearrayOLD[addr]=sinewavearray[addr];
  delay(100); 
  }//while 
}//SinewaveTwo

void sinwaveTwo_colorwheel()
{
  int sinewavearray[4], addr, sinemult[4], colselect, rr=0, gg=0, bb=15, addrt;
  int sinewavearrayOLD[4], select, subZ=-3, subT=3, multi=0;//random(-1, 2);
  sinewavearray[0]=0;
  sinemult[0]=1;
  sinewavearray[1]=1;
  sinemult[1]=1; 
  sinewavearray[2]=2;
  sinemult[2]=1;
  sinewavearray[3]=3;
  sinemult[3]=1;
    
  start=millis();
      
  while(millis()-start<15000){
  for(addr=0; addr<4; addr++){
    if(sinewavearray[addr]==3){
    sinemult[addr]=-1;
    }
    if(sinewavearray[addr]==0){
    sinemult[addr]=1;     
    }
    sinewavearray[addr] = sinewavearray[addr] + sinemult[addr];
  }//addr
   
  for(addr=0; addr<4; addr++){
    get_colour(colourPos + 20*sinewavearrayOLD[addr], &R, &G, &B);  
    LED(sinewavearrayOLD[addr], addr, 0, 0, 0, 0);
    LED(sinewavearrayOLD[addr], 0, addr, 0, 0, 0);
    LED(sinewavearrayOLD[addr], subT-addr, 3, 0, 0, 0);
    LED(sinewavearrayOLD[addr], 3, subT-addr, 0, 0, 0);     
    LED(sinewavearray[addr], addr, 0, R, G, B);
    LED(sinewavearray[addr], 0, addr, R, G, B);
    LED(sinewavearray[addr], subT-addr, 3, R, G, B);
    LED(sinewavearray[addr], 3, subT-addr, R, G, B);
    }//}
    
  for(addr=1; addr<3; addr++){
    get_colour(colourPos + 20*sinewavearrayOLD[addr+multi*1], &R, &G, &B);  
    LED(sinewavearrayOLD[addr+multi*1], addr, 1, 0, 0, 0);
    LED(sinewavearrayOLD[addr+multi*1], 1, addr, 0, 0, 0);
    LED(sinewavearrayOLD[addr+multi*1], subT-addr, 2, 0, 0, 0);
    LED(sinewavearrayOLD[addr+multi*1], 2, subT-addr, 0, 0, 0);  
    LED(sinewavearray[addr+multi*1], addr, 1, R, G, B);
    LED(sinewavearray[addr+multi*1], 1, addr, R, G, B);
    LED(sinewavearray[addr+multi*1], subT-addr, 2, R, G, B);
    LED(sinewavearray[addr+multi*1], 2, subT-addr, R, G, B);
       }
 
  for(addr=2; addr<2; addr++){ 
    get_colour(colourPos + 20*sinewavearrayOLD[addr+multi*2], &R, &G, &B);  
    LED(sinewavearrayOLD[addr+multi*2], addr, 2, 0, 0, 0);
    LED(sinewavearrayOLD[addr+multi*2], 2, addr, 0, 0, 0);
    LED(sinewavearrayOLD[addr+multi*2], subT-addr, 1, 0, 0, 0);
    LED(sinewavearrayOLD[addr+multi*2], 1, subT-addr, 0, 0, 0);  
    LED(sinewavearray[addr+multi*2], addr, 2, R, G, B);
    LED(sinewavearray[addr+multi*2], 2, addr, R, G, B);
    LED(sinewavearray[addr+multi*2], subT-addr, 1, R, G, B);
    LED(sinewavearray[addr+multi*2], 1, subT-addr, R, G, B);
       }        
  for(addr=0; addr<4; addr++)
    sinewavearrayOLD[addr]=sinewavearray[addr];
    delay(100);
    increment_colour_pos(5);   
  }//while
   
}//sinwaveTwo_colorwheel

void walk_through_walls(int delay, int iterations){
  int i,j,c,k;
  //getColor(xColor,4);
  for(i=0;i<iterations*2;i++){  
    for(c=0;c<CUBE_SIZE;c++){
      for(k=0;k<4;k++){      
        for(j=0;j<CUBE_SIZE;j++){
          get_colour(colourPos + 10*(c+ k + j), &R, &G, &B);
          if(k==0){
            if(!(i%2)) LED(c,0,j, R, G, B);
            else LED(c,0,j,0,0,0);
            delay_ms(delay);
          }
          else if(k==1){
            if(!(i%2))  LED(c,j+1,CUBE_SIZE-1, R, G, B);
            else LED(c,j+1,CUBE_SIZE-1,0,0,0);
            delay_ms(delay);
            if(j==CUBE_SIZE-2) break;
          }
          else if(k==2){
            if(!(i%2)) LED(c,CUBE_SIZE-1,CUBE_SIZE-2-j, R, G, B);
            else LED(c,CUBE_SIZE-1,CUBE_SIZE-2-j,0,0,0);
            delay_ms(delay);
            if(j==CUBE_SIZE-2) break;
          }
          else{
            if(!(i%2))  LED(c,CUBE_SIZE-2-j,0, R, G, B);
            else LED(c,CUBE_SIZE-2-j,0,0,0,0);
            delay_ms(delay);
            if(j==CUBE_SIZE-3) break;
          }
        }
      }
      increment_colour_pos(4); 
      
    }
  }
}

void HSBToRGB(unsigned int inHue, unsigned int inSaturation, unsigned int inBrightness, uint8_t *oR, uint8_t *oG, uint8_t *oB)
{
  if (inSaturation == 0) {
    *oR = inBrightness;
    *oG = inBrightness;
    *oB = inBrightness;
  } else {
    unsigned int scaledHue = (inHue * 6);
    unsigned int sector = scaledHue >> 4;
    unsigned int offsetInSector = scaledHue - (sector << 4);
    unsigned int p = (inBrightness * ( 15 - inSaturation )) >> 4;
    unsigned int q = (inBrightness * ( 15 - ((inSaturation * offsetInSector) >> 4) )) >> 4;
    unsigned int t = (inBrightness * ( 15 - ((inSaturation * ( 15 - offsetInSector )) >> 4) )) >> 4;

    switch(sector) {
    case 0:
      *oR = inBrightness;
      *oG = t;
      *oB = p;
      break;
    case 1:
      *oR = q;
      *oG = inBrightness;
      *oB = p;      
      break;
    case 2:
      *oR = p;
      *oG = inBrightness;
      *oB = t;   
      
      break;
    case 3:
      *oR = p;
      *oG = q;
      *oB = inBrightness;         
      break;
    case 4:
      *oR = t;
      *oG = p;
      *oB = inBrightness;        
      break;
    default:    // case 5:
      *oR = inBrightness;
      *oG = p;
      *oB = q;  
      break;
    }
  }
}

void rainbow2(int interactions) {
  byte hue, hueref, dir;
  byte fx, fy, fz;
  for (byte i = 0 ; i < interactions ; i++) {
    hueref = random(15);
    dir = random(16);
    for (byte x = 0 ; x < 4 ; x++) {
      for (byte y = 0 ; y < 4 ; y++) {
        for (byte z = 0 ; z < 4 ; z++) {        
          fx = x; fy = y; fz = z;
          if (dir & 0x1 == 1) {
            fx = 3-x;
          } else if (dir & 0x2 == 2) {
            fy = 3-y;
          } else if (dir & 0x4 == 4){
            fz = 3-z;
          }          
          hue = hueref;
          HSBToRGB(hue, 15, 15, &R, &G, &B);
          LED(fz, fy, fx, R, G, B);
          delay_ms(3);
        }
      }
      hueref += (dir & 0x8 == 0x8 ? 1 : -1);
    }
    delay(1000);    
  }
}


void rainbow1(int interactions) {
  byte dir;
  byte fx, fy, fz;
  for (byte i = 0 ; i < interactions ; i++) {
    dir = random(16);
    for (byte x = 0 ; x < 4 ; x++) {
      for (byte y = 0 ; y < 4 ; y++) {
        for (byte z = 0 ; z < 4 ; z++) {      
          fx = x; fy = y; fz = z;
          if (dir & 0x1 == 1) {
            fx = 3-x;
          } else if (dir & 0x2 == 2) {
            fy = 3-y;
          } else if (dir & 0x4 == 4){
            fz = 3-z;
          }                    
          get_colour(colourPos + 10*fz + 10*fy, &R, &G, &B);
          LED(fz, fy, fx, R, G, B);
          delay_ms(3);
        }
      }          
    increment_colour_pos(2);
    }
    delay(1000);
  }
}

void draw_positions_axis_colorwheel (char axis, unsigned char positions[16], int invert)
{
  int x, y, z, p; 

  clearfast();
  //manage_color();
  for (x=0; x<4; x++)
  {        
    for (y=0; y<4; y++)
    {
      if (invert)
      {
        p = (3-positions[(x*4)+y]);
      } else
      {
        p = positions[(x*4)+y];
      }
      get_colour(colourPos + 40*p, &R, &G, &B);
      if (axis == AXIS_Z)
         
        LED(p,y,x, R, G, B);
        
      if (axis == AXIS_Y)
        
        LED(y,p,x, R, G, B);
        
      if (axis == AXIS_X)
        
        LED(x,y,p, R, G, B);
    }    
  }  
}

void effect_boxside_randsend_parallel_colorwheel (char axis, int origin, int delay, int mode)
{
  int i;
  int done;
  unsigned char cubepos[16];
  unsigned char pos[16];
  int notdone = 1;
  int notdone2 = 1;
  int sent = 0;
  
  for (i=0;i<16;i++)
  {
    pos[i] = 0;
  }
  
  while (notdone)
  {
    if (mode == 1)
    {
      notdone2 = 1;
      while (notdone2 && sent<16)
      {
        i = rand()%16;
        if (pos[i] == 0)
        {
          sent++;
          pos[i] += 1;
          notdone2 = 0;
        }
      }
    } else if (mode == 2)
    {
      if (sent<16)
      {
        pos[sent] += 1;
        sent++;
      }
    }
    
    done = 0;
    for (i=0;i<16;i++)
    {
      if (pos[i] > 0 && pos[i] <3)
      {
        pos[i] += 1;
      }
        
      if (pos[i] == 3)
        done++;
    }
    
    if (done == 16)
      notdone = 0;
    
    for (i=0;i<16;i++)
    {
      if (origin == 0)
      {
        cubepos[i] = pos[i];
      } else
      {
        cubepos[i] = (3-pos[i]);
      }
    }   
    delay_ms(delay);
    draw_positions_axis_colorwheel(axis,cubepos,0);
  } 
}

void TranslateScroll(char axis, int8_t value)
{
  uint8_t i, j, k, l, x, y, z, prev_x, prev_y, prev_z, dx, dy, dz;
  byte tempcolor[3], start_state[3];
  int8_t positive, negative;
  int8_t increment_x[4];
  int8_t increment_y[4];
  int8_t increment_z[4];

  positive = (value > 0);
  negative = (value < 0);

  switch (axis)
  {
  default:
  case AXIS_X:
    increment_x[0] = 0;
    increment_x[1] = 0;
    increment_x[2] = 0;
    increment_x[3] = 0;

    increment_y[0] = positive;
    increment_y[1] = negative;
    increment_y[2] = -positive;
    increment_y[3] = -negative;

    increment_z[0] = negative;
    increment_z[1] = positive;
    increment_z[2] = -negative;
    increment_z[3] = -positive;

    dx = 1;
    dy = 0;
    dz = 0;
    break;

  case AXIS_Y:
    increment_x[0] = positive;
    increment_x[1] = negative;
    increment_x[2] = -positive;
    increment_x[3] = -negative;

    increment_y[0] = 0;
    increment_y[1] = 0;
    increment_y[2] = 0;
    increment_y[3] = 0;

    increment_z[0] = negative;
    increment_z[1] = positive;
    increment_z[2] = -negative;
    increment_z[3] = -positive;

    dx = 0;
    dy = 1;
    dz = 0;
    break;

  case AXIS_Z:
    increment_x[0] = negative;
    increment_x[1] = positive;
    increment_x[2] = -negative;
    increment_x[3] = -positive;

    increment_y[0] = positive;
    increment_y[1] = negative;
    increment_y[2] = -positive;
    increment_y[3] = -negative;

    increment_z[0] = 0;
    increment_z[1] = 0;
    increment_z[2] = 0;
    increment_z[3] = 0;

    dx = 0;
    dy = 0;
    dz = 1;
    break;
  }

  value -= ((value >= 0) - 1) & (value << 1);

  for (l = 0; l<value; l++)
  {
    x = 0;
    y = 0;
    z = 0;
    
    for (i = 0; i < 8; i++)
    {
     getColor(z, y, x, start_state); 
      
      for (j = 0; j < 4; j++)
      {
        for (k = 0; k < 7; k++)
        {
          prev_x = x;
          prev_y = y;
          prev_z = z;

          x += increment_x[j];
          y += increment_y[j];
          z += increment_z[j];
          getColor(z, y, x, tempcolor);
          LED(prev_z, prev_y, prev_x, tempcolor[0], tempcolor[1], tempcolor[2]);
        }
      }

      LED(prev_z, prev_y, prev_x, start_state[0], start_state[1], start_state[2]);

      x += dx;
      y += dy;
      z += dz;
    }
  }
}

void TransitionScroll(char axis, const type_polarity polarity, const uint8_t delayx)
{
  uint8_t x;

  for (x = 0; x < 3; x++)
  {
    TranslateScroll(axis, ((polarity == POSITIVE) << 1) - 1);
    delay_ms(delayx);
  }
}

void drawRPrism(uint8_t x, uint8_t y, uint8_t z, int8_t dx, int8_t dy, int8_t dz, int xColor)
{
  getColor(xColor,4);
  // top rectangle
  line_3d(x, y, z, x, y+dy, z, xColor);
  line_3d(x, y+dy, z, x+dx, y+dy, z, xColor);
  line_3d(x+dx, y+dy, z, x+dx, y, z, xColor);
  line_3d(x+dx, y, z, x, y, z, xColor);

  // bottom rectangle
  line_3d(x, y, z+dz, x, y+dy, z+dz, xColor);
  line_3d(x, y+dy, z+dz, x+dx, y+dy, z+dz, xColor);
  line_3d(x+dx, y+dy, z+dz, x+dx, y, z+dz, xColor);
  line_3d(x+dx, y, z+dz, x, y, z+dz, xColor);

  // joining verticals
  line_3d(x, y, z, x, y, z+dz, xColor);
  line_3d(x, y+dy, z, x, y+dy, z+dz, xColor);
  line_3d(x+dx, y+dy, z, x+dx, y+dy, z+dz, xColor);
  line_3d(x+dx, y, z, x+dx, y, z+dz, xColor);
}

void shrinkCube_random(int interactions)
{
  const uint16_t delayx = 150;
  uint8_t cur = 0;
  uint8_t x, y, z, dx, dy, dz;
  int8_t corners[8][6] =
  {
    { 0, 0, 0 , 1, 1, 1 },
    { 0, 3, 0 , 1, -1, 1 },
    { 3, 0, 0, -1, 1, 1 },
    { 0, 0, 3, 1, 1, -1 },
    { 3, 3, 0, -1, -1, 1 },
    { 0, 3, 3, 1, -1, -1 },
    { 3, 0, 3, -1, 1, -1 },
    { 3, 3, 3, -1, -1, -1 }
  };
  
  clearfast();

  for (int i=0; i<interactions; i++)
  {
    x = corners[cur][0];
    y = corners[cur][1];
    z = corners[cur][2];
    uint8_t xColor = rand()%46;
    for (uint8_t i = 1; i < 4; i++)
    {
      dx = i * corners[cur][3];
      dy = i * corners[cur][4];
      dz = i * corners[cur][5];
      drawRPrism(x, y, z, dx, dy, dz, xColor);
      delay(delayx);
      drawRPrism(x, y, z, dx, dy, dz, 47);
    }
    cur = random(ARRAY_SIZE(corners));
    x = corners[cur][0];
    y = corners[cur][1];
    z = corners[cur][2];

    for (uint8_t i = 3; i >= 1; i--)
    {
      dx = i * corners[cur][3];
      dy = i * corners[cur][4];
      dz = i * corners[cur][5];
      drawRPrism(x, y, z, dx, dy, dz, xColor);
      delay(delayx);
      drawRPrism(x, y, z, dx, dy, dz, 47);
    }
  }
}

void rainboww() {   
    for(int j=0; j<20; j++) 
    {
        for(int z=0; z<4; z++)
        {
          for(int y=0; y<4; y++)
          {
            for(int x=0; x<4; x++)
            {
              get_colour(colourPos + 2*z + 4*y + 8*x + j, &R, &G, &B);          
              LED(z, y, x, R, G, B);                                                    
              //delay_ms(1);             
            } 
            increment_colour_pos(5);                     
          }        
      } 
      delay(1000);                       
    }
}


void shift (char axis, int direction)
{
  int i, x ,y;
  int ii, iii;
  byte tempcolor[3];

  for (i = 0; i < 4; i++)
  {
    if (direction == -1)
    {
      ii = i;
    } else
    {
      ii = (3-i);
    } 
   
    for (x = 0; x < 4; x++)
    {
      for (y = 0; y < 4; y++)
      {
        if (direction == -1)
        {
          iii = ii+1;
        } else
        {
          iii = ii-1;
        }
        
        if (axis == AXIS_Z)
        {
          getColor(iii,y,x, tempcolor);
          LED(ii,y,x,tempcolor[0],tempcolor[1], tempcolor[2] );
        }
        
        if (axis == AXIS_Y)
        {
          getColor(y,iii,x, tempcolor);
          LED(y,ii,x,tempcolor[0],tempcolor[1], tempcolor[2]);
        }
        
        if (axis == AXIS_X)
        {
          getColor(x,y,iii,tempcolor );
          LED(x,y,ii,tempcolor[0],tempcolor[1], tempcolor[2]);

        }
      }
    }
  }
  
  if (direction == -1)
  {
    i = 3;
  } else
  {
    i = 0;
  } 
  
  for (x = 0; x < 4; x++)
  {
    for (y = 0; y < 4; y++)
    {
      if (axis == AXIS_Z)
        LED(i,y,x, 0,0,0);
        
      if (axis == AXIS_Y)
        LED(y,i,x,0,0,0);
      
      if (axis == AXIS_X)
        LED(x,y,i,0,0,0);
    }
  }
}
void manage_color() {
  mycolor=mycolor+3;
  if (mycolor>46){
    mycolor=0;
  } 
  getColor(mycolor, 4);
}
void squarespiral (int iterations, int Delay)//, int xColor)
{
int loc = 0;
int iter = 0;
while (iter <= iterations)
{

for (loc =0;loc < 3; loc ++)
{
manage_color();
shift (AXIS_Z,-1);
LED (3,loc,0, myred, mygreen, myblue);
LED (3,3,loc, myred, mygreen, myblue);
LED (3,3-loc,3, myred, mygreen, myblue);
LED (3,0,3-loc, myred, mygreen, myblue);

delay_ms (Delay);
iter++;
}
loc = 0;
}
}

void drawCircle(int x0, int y0, int z0, int radius, int xColor){
  int r = radius, t = 0;
  int radiusError = 1-r;
  getColor(xColor,4);
  while(r >= t){
    LED(z0, t + y0,r + x0, myred, mygreen, myblue );
    LED(z0, r + y0,t + x0, myred, mygreen, myblue);
    LED(z0, t + y0,-r + x0, myred, mygreen, myblue);
    LED(z0, r + y0,-t + x0, myred, mygreen, myblue);
    LED(z0, -t + y0,-r + x0, myred, mygreen, myblue);
    LED(z0, -r + y0,-t + x0, myred, mygreen, myblue);
    LED(z0, -t + y0,r + x0, myred, mygreen, myblue);
    LED(z0, -r + y0,t + x0, myred, mygreen, myblue);
    t++;
    if (radiusError<0){
      radiusError += 2 * t + 1;
    }
    else{
      r--;
      radiusError += 2 * (t - r + 1);
    }
  }
}
void bomb(int delayx, int iterations){
  int i;
  uint8_t j,x,y,r, bomb_color;
  for(i=0;i<iterations;i++){
    x = rand()%4;
    y = rand()%4;
    bomb_color = rand()%46;
    LED(CUBE_SIZE-1,y,x, rand()%15, rand()%15, rand()%15);
    for(j=0;j<CUBE_SIZE;j++){
      delay_ms(delayx);
      shift(AXIS_Z,-1);
    }
    for(r=1;r<=CUBE_SIZE;r++){
      drawCircle(x,y,0,r, bomb_color);
      delay_ms(delayx);
      clearfast();
    }
    delay(1000);
  }
}

void flpvoxel(int x, int y, int z)
{
  if (inrange(x, y, z))
    cube[z][y] ^= (1 << x);
}

void boingboing(uint16_t iterations, int delay, unsigned char mode, unsigned char drawmode)
{
  clearfast();

  int x, y, z;
  int dx, dy, dz;
  int lol, i;   // lol?
  unsigned char crash_x, crash_y, crash_z;

  y = rand()%4;
  x = rand()%4;
  z = rand()%4;

  int snake[4][3];
  for (i=0;i<4;i++)
  {
    snake[i][0] = x;
    snake[i][1] = y;
    snake[i][2] = z;
  }  
  dx = 1;
  dy = 1;
  dz = 1;
  
  while(iterations)
  {
    crash_x = 0;
    crash_y = 0;
    crash_z = 0;
  
    if (rand()%3 == 0)
    {
      lol = rand()%3;
      if (lol == 0)
        dx = rand()%3 - 1;
      
      if (lol == 1)
        dy = rand()%3 - 1;
        
      if (lol == 2)
        dz = rand()%3 - 1;
    }

    if (dx == -1 && x == 0)
    {
      crash_x = 0x01;
      if (rand()%3 == 1)
      {
        dx = 1;
      } else
      {
        dx = 0;
      }
    }
    
    if (dy == -1 && y == 0)
    {
      crash_y = 0x01;
      if (rand()%3 == 1)
      {
        dy = 1;
      } else
      {
        dy = 0;
      }
    }
    
    if (dz == -1 && z == 0)
    {
      crash_z = 0x01;
      if (rand()%3 == 1)
      {
        dz = 1;
      } else
      {
        dz = 0;
      }
    }
      
    if (dx == 1 && x == 3)
    {
      crash_x = 0x01;
      if (rand()%3 == 1)
      {
        dx = -1;
      } else
      {
        dx = 0;
      }
    }
    
    if (dy == 1 && y == 3)
    {
      crash_y = 0x01;
      if (rand()%3 == 1)
      {
        dy = -1;
      } else
      {
        dy = 0;
      }
    }
    
    if (dz == 1 && z == 3)
    {
      crash_z = 0x01;
      if (rand()%3 == 1)
      {
        dz = -1;
      } else
      {
        dz = 0;
      }
    }
    
    if (mode | 0x01)
    {
      if (crash_x)
      {
        if (dy == 0)
        {
          if (y == 3)
          {
            dy = -1;
          } else if (y == 0)
          {
            dy = +1;
          } else
          {
            if (rand()%2 == 0)
            {
              dy = -1;
            } else
            {
              dy = 1;
            }
          }
        }
        if (dz == 0)
        {
          if (z == 3)
          {
            dz = -1;
          } else if (z == 0)
          {
            dz = 1;
          } else
          {
            if (rand()%2 == 0)
            {
              dz = -1;
            } else
            {
              dz = 1;
            }
          } 
        }
      }
      
      if (crash_y)
      {
        if (dx == 0)
        {
          if (x == 3)
          {
            dx = -1;
          } else if (x == 0)
          {
            dx = 1;
          } else
          {
            if (rand()%2 == 0)
            {
              dx = -1;
            } else
            {
              dx = 1;
            }
          }
        }
        if (dz == 0)
        {
          if (z == 3)
          {
            dz = -1;
          } else if (z == 0)
          {
            dz = 1;
          } else
          {
            if (rand()%2 == 0)
            {
              dz = -1;
            } else
            {
              dz = 1;
            }
          } 
        }
      }
      
      if (crash_z)
      {
        if (dy == 0)
        {
          if (y == 3)
          {
            dy = -1;
          } else if (y == 0)
          {
            dy = 1;
          } else
          {
            if (rand()%2 == 0)
            {
              dy = -1;
            } else
            {
              dy = 1;
            }
          } 
        }
        if (dx == 0)
        {
          if (x == 3)
          {
            dx = -1;
          } else if (x == 0)
          {
            dx = 1;
          } else
          {
            if (rand()%2 == 0)
            {
              dx = -1;
            } else
            {
              dx = 1;
            }
          } 
        }
      }
    }
    
    if (mode | 0x02)
    {
      if (
        (x == 0 && y == 0 && z == 0) ||
        (x == 0 && y == 0 && z == 3) ||
        (x == 0 && y == 3 && z == 0) ||
        (x == 0 && y == 3 && z == 3) ||
        (x == 3 && y == 0 && z == 0) ||
        (x == 3 && y == 0 && z == 3) ||
        (x == 3 && y == 3 && z == 0) ||
        (x == 3 && y == 3 && z == 3)
      )
      {
        lol = rand()%3;
        if (lol == 0)
          dx = 0;
        
        if (lol == 1)
          dy = 0;
          
        if (lol == 2)
          dz = 0;
      }
    }

        if (x == 0 && dx == -1)
            dx = 1;
  
        if (y == 0 && dy == -1)
            dy = 1;
  
        if (z == 0 && dz == -1)
            dz = 1;
  
        if (x == 3 && dx == 1)
            dx = -1;
  
        if (y == 3 && dy == 1)
            dy = -1;
  
        if (z == 3 && dz == 1)
            dz = -1;
  
    x = x + dx;
    y = y + dy;
    z = z + dz;
    
    if (drawmode == 0x01) // show one voxel at time
    {
      get_colour(colourPos + 10*z, &R, &G, &B);
      LED(z, y, x, R, G, B);
      delay_ms(delay);
      LED(z, y, x, 0, 0, 0);  
    } else if (drawmode == 0x02) // flip the voxel in question
    {
      flpvoxel(z,y,x);
      delay_ms(delay);
    } if (drawmode == 0x03) // draw a snake
    {
      for (i=3;i>=0;i--)
      {
        snake[i][0] = snake[i-1][0];
        snake[i][1] = snake[i-1][1];
        snake[i][2] = snake[i-1][2];
      }
      snake[0][0] = x;
      snake[0][1] = y;
      snake[0][2] = z;
        
      for (i=0;i<4;i++)
      {
        get_colour(colourPos + 10*snake[i][2], &R, &G, &B);
        LED(snake[i][2], snake[i][1], snake[i][0], R, G, B);
      }
      delay_ms(delay);
      for (i=0;i<4;i++)
      {
        LED(snake[i][2], snake[i][1], snake[i][0], 0, 0, 0);
      }
    }
    
    increment_colour_pos(2);
    iterations--;
  }
}

float distance2d (float x1, float y1, float x2, float y2)
{ 
  float dist;
  dist = sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2));

  return dist;
}
void sidewaves_colorwheel(int iterations, int delay)
{
  float origin_x, origin_y, distance, height, ripple_interval;
  int x,y,i;
  uint8_t R, G, B;
  clearfast();
  for (i=0;i<iterations;i++)
  {

    origin_x = 1.5+sin((float)i/50)*4;
    origin_y = 1.5+cos((float)i/50)*4;
    
    for (x=0;x<4;x++)
    {
      for (y=0;y<4;y++)
      {
        distance = distance2d(origin_x,origin_y,x,y)/9.899495*8;
        ripple_interval =1.5;
        height = 2+sin(distance/ripple_interval+(float) i/5)*1.6;
        get_colour(colourPos + 10*height, &R, &G, &B);
        LED((int) height, y, x, R, G, B);
          
      }
    }
    increment_colour_pos(5);
    delay_ms(delay);
    clearfast();
  }
}

void fillcube_hue(int interactions)
{    
for (int i=0; i<interactions; i++)
  {
    
    for (uint8_t x=0; x<4; x++)
      {
      
      for (uint8_t y=0; y<4; y++)
        {
          hueToRGB(2*y+1, rand()%15+8, &R, &G, &B);
          for (uint8_t z=0; z<4; z++)
            {
              LED(z, y, x, R, G, B);
            }
          delay(100);
        }      
      }

    
    for (uint8_t z=0; z<4; z++)
      {
      
      for (uint8_t y=0; y<4; y++)
        {
          hueToRGB(15-2*y, rand()%15+8, &R, &G, &B);
        for (uint8_t x=0; x<4; x++)
          {            
            LED(z, y, x, R, G, B);
          }
        delay(100);
      }      
    }   
    for (uint8_t z=0; z<4; z++)
    {
      
      for (uint8_t x=0; x<4; x++)
        {
          hueToRGB(2*x+1, rand()%15+8, &R, &G, &B);
          for (uint8_t y=0; y<4; y++)
            {
              LED(z, y, x, R, G, B);
            }
          delay(100);
        }      
      }    
   }
}    



void setplane_x (int x, byte red, byte green, byte blue)
{
  int z;
  int y;
  if (x>=0 && x<4)
  {
    for (z=0;z<4;z++)
    {
      for (y=0;y<4;y++)
      {
        LED(z, y, x, red, green, blue);
      }
    }
  }
}

void clrplane_x (int x)
{
  int z;
  int y;
  if (x>=0 && x<4)
  {
    for (z=0;z<4;z++)
    {
      for (y=0;y<4;y++)
      {
        LED(z, y, x, 0, 0, 0);        
      }
    }
  }
}

void setplane_y (int y, byte red, byte green, byte blue)
{
  int z, x;
  if (y>=0 && y<4)
  {
    for (z=0;z<4;z++)
    {
      for (x=0;x<4;x++)
      {
    
      LED(z, y, x,red, green, blue);
      } 
    }
  }
}

void clrplane_y (int y)
{
  int z, x;
  if (y>=0 && y<4)
  {
    for (z=0;z<4;z++)
    {
      for (x=0;x<4;x++)
      {
    
      LED(z, y, x,0, 0, 0);
      } 
    }
  }
}


void setplane_z (int z, byte red, byte green, byte blue)
{
  int y,x;
  if (z>=0 && z<4)
  {
    for (y=0;y<4;y++)
    {
      for (x=0;x<4;x++)
        {  
          LED(z, y, x,red, green, blue);  
        }
     }
  }
}

// Clears voxels in the same manner as above
void clrplane_z (int z)
{
  int y,x;
  if (z>=0 && z<4)
  {
    for (y=0;y<4;y++)
    {
      for (x=0;x<4;x++)
        {  
          LED(z, y, x,0, 0, 0);  
        }
     }
  }
}

void setplane (char axis, unsigned char i, byte red, byte green, byte blue)
{
    switch (axis)
    {
        case AXIS_X:
            setplane_x(i, red, green, blue);
            break;
        
       case AXIS_Y:
            setplane_y(i, red, green, blue);
            break;

       case AXIS_Z:
            setplane_z(i, red, green, blue);
            break;
    }
}


void setplane_x_colorwheel (int x)
{
  int z;
  int y;  
  if (x>=0 && x<4)
  {
    for (z=0;z<4;z++)
    {
      for (y=0;y<4;y++)
      {
        get_colour(colourPos + 10*(z+y), &R, &G, &B);
        LED(z, y, x, R, G, B);
        increment_colour_pos(2);
      }
    }
  }
}

void setplane_y_colorwheel (int y)
{
  int z, x;
  if (y>=0 && y<4)
  {
    for (z=0;z<4;z++)
    {
      for (x=0;x<4;x++)
      {
      get_colour(colourPos + 10*(z+x), &R, &G, &B);
      LED(z, y, x, R, G, B);
      increment_colour_pos(2);
      } 
    }
  }
}

void setplane_z_colorwheel (int z)
{
  int y,x;
  if (z>=0 && z<4)
  {
    for (y=0;y<4;y++)
    {
      for (x=0;x<4;x++)
        {  
          get_colour(colourPos + 10*(y+x), &R, &G, &B);
          LED(z, y, x, R, G, B); 
          increment_colour_pos(2); 
        }
     }
  }
}


void setplane_colorwheel (char axis, unsigned char i)
{

    switch (axis)
    {
        case AXIS_X:
            setplane_x_colorwheel(i);
            break;
        
       case AXIS_Y:
            setplane_y_colorwheel(i);
            break;

       case AXIS_Z:
            setplane_z_colorwheel(i);
            break;
    }
}

void clrplane (char axis, unsigned char i)
{
    switch (axis)
    {
        case AXIS_X:
            clrplane_x(i);
            break;
        
       case AXIS_Y:
            clrplane_y(i);
            break;

       case AXIS_Z:
            clrplane_z(i);
            break;
    }
}


void fillPlane(plane_t plane, uint8_t coord, int xColor)
// Fill the specified plane at the coordinate value
{
  getColor(xColor, 4);
  switch (plane)
  {
    case XYPLANE: 
      for (uint8_t i = 0; i < 4; i++)
        for (uint8_t j = 0; j < 4; j++)
          LED(coord, j, i, myred, mygreen, myblue); 
      break;
    case XZPLANE:
      for (uint8_t i = 0; i < 4; i++)
        for (uint8_t j = 0; j < 4; j++)
          LED(j, coord, i, myred, mygreen, myblue); 
      break;
    case YZPLANE:
      for (uint8_t i = 0; i < 4; i++)
        for (uint8_t j = 0; j < 4; j++)
          LED(j, i, coord, myred, mygreen, myblue);
      break;
  }
}

void copyPlane(char axis, uint8_t cordFrom, uint8_t cordTo)
// copy the plane from level cordFrom to cordTo
{
  byte tempcolor[3];
  //getColor(xColor, 4);
  clrplane(axis, cordTo);    // clear the destination plane
  switch (axis)
  {
  case AXIS_Z:
    for (uint8_t i = 0; i < 4; i++)
    {
      for (uint8_t j = 0; j < 4; j++)
      {
        getColor(cordFrom,j,i, tempcolor);
        if ((tempcolor[0]>0)||(tempcolor[1]>0)||(tempcolor[2]>0))
        
          LED(cordTo, j, i, tempcolor[0], tempcolor[1], tempcolor[2]);
        else
          LED(cordTo, j, i, 0, 0, 0);
      }
    }
    break;
  case AXIS_Y:
    for (uint8_t i = 0; i < 4; i++)
    {
      for (uint8_t j = 0; j < 4; j++)
      {
        getColor(j, cordFrom, i, tempcolor);
        if ((tempcolor[0]>0)||(tempcolor[1]>0)||(tempcolor[2]>0))
        
          LED(j, cordTo, i, tempcolor[0], tempcolor[1], tempcolor[2]);
        else
          LED(j, cordTo, i, 0, 0, 0);
      }
    }
    break;
  case AXIS_X:
    for (uint8_t i = 0; i < 4; i++)
    {
      for (uint8_t j = 0; j < 4; j++)
      {
        getColor(j, i, cordFrom, tempcolor);
        if ((tempcolor[0]>0)||(tempcolor[1]>0)||(tempcolor[2]>0))
        
          LED(j, i, cordTo, tempcolor[0], tempcolor[1], tempcolor[2]);
        else
          LED(j, i, cordTo,0,0,0);
      }
    }
    break;
  }
}


void flagwave(int xColor, int interactions)
// Looks like a flag waving in the wind
{
  uint8_t curY = 0, dy = 1;
  clearfast();
  for (int i=0; i<interactions; i++)
  {
    // shift all the planes back by one
    for (uint8_t x = 3; x > 0; x--)
      copyPlane(AXIS_X, x - 1, x);
      clrplane(AXIS_X, 0);

    // draw the wave line on the YZ plane
    line_3d(0, curY, 0, 0, curY, 3, xColor);
    curY += dy;
    if (curY == 3 || curY == 0) dy = -dy;
    delay(200);
  }
}


void FleaJump(
  int8_t x0, int8_t y0,
  int8_t x1, int8_t y1, int8_t height, int xColor)
{
  int8_t range;
  int8_t rotation;
  int8_t bgn,enz,stp, strt, loopMax;
  if (y0==y1) {rotation=0; range=abs(x1-x0); loopMax=range; if (x1>x0) {strt=x0; bgn=0; enz=loopMax+1; stp=1;} else {strt=x1; bgn=loopMax; enz=-1; stp=-1;}};
  if (x0==x1) {rotation=1; range=abs(y1-y0); loopMax=range; if (y1>y0) {strt=y0; bgn=0; enz=loopMax+1; stp=1;} else {strt=y1; bgn=loopMax; enz=-1; stp=-1;}};

  float xMin = -range/2.0;
  float xMax = range/2.0;
  float xStp = 1.0;
  float c = height;
  float a = -c / pow(xMax, 2);
  float x = xMin;

  
  for (int8_t dx=bgn; dx!=enz; dx=dx+stp)
  {
    int8_t p = strt + dx;
    float z = a * pow(x,2) + c;
    getColor(xColor, 4);
    if (rotation==0)
    {
      LED(z,y0,p, myred, mygreen, myblue);
      delay(64);
      LED(z,y0,p,0,0,0);
    }
    else
    {
      LED(z,p,x0, myred, mygreen, myblue);
      delay(64);
      LED(z,p,x0,0,0,0);
    }
    x = x + xStp;
  }
}
void RandomFleaJumps(int xColor)
{
  int8_t x0;
  int8_t y0;
  int8_t x1 = random(8);
  int8_t y1 = random(8);
  int8_t height;
  int8_t nTimes = 20;
  getColor(xColor, 4);
  for (int8_t n=0; n<nTimes; n++)
  {
    int8_t jumps = random(1,3);
    for (int8_t j=0; j<jumps; j++)
    {
      x0=x1; y0=y1;
      if (random(2) == 0) {
        x1 = Crop(x1 + random(-3,4));
      }
      else {
        y1 = Crop(y1 + random(-3,4));
      }
      height = random(1,3);
      FleaJump(x0,y0, x1,y1, height, xColor);
    }
    LED(0,y1,x1, random(15), random(15), random(15));// Randomcolor
    delay(500+random(500));
  }
  LED(0, y1, x1,0,0,0);
}
void FleaParty(int xColor)
{
  byte n = 5;
  int8_t x0[n];
  int8_t y0[n];
  int8_t x1[n];
  int8_t y1[n];
  int8_t height;
  int8_t nTimes = 2;
  for (int8_t f=0; f<n; f++)
  {
    x1[f] = random(4);
    y1[f] = random(4);

  }

  for (int8_t r=0; r<nTimes; r++)
  {
    int8_t jumps = random(1,3);
    for (int8_t j=0; j<jumps; j++)
    {
      for (int8_t f=0; f<n; f++)
      {
        x0[f]=x1[f]; y0[f]=y1[f];
        getColor(xColor, 4);
        if (random(2) == 0) {
          x1[f] = Crop(x1[f] + random(-3,4));
        }
        else {
          y1[f] = Crop(y1[f] + random(-3,4));
        }
        height = random(3,6);
        FleaJump(x0[f],y0[f], x1[f],y1[f], height, xColor);
        LED(0,y1[f],x1[f], random(15), random(15), random(15));
      }
    }
  }
}

void EffectPlaneTwist()
{
  uint16_t i, j;
  float angle, increment, twist;

  angle = 0.0;
  twist = 0.0;

  for (i = 0; i < 1270; i++)
  {
    clearfast();
    increment = 0.0;
    for (j = 0; j < 4; j++)
    {
      float x = 2.375*cos(angle + increment);
      float y = 2.375*sin(angle + increment);
      get_colour(colourPos + 20*j, &R, &G, &B);
      if ((i-(4*j)) < (1270-(4*8)))
      {
        line_3d_colorwheel(2.0 + x, 2.0 + y, j, 2.0 - x, 2.0 - y, j);
      }
      
      increment += twist;
    }

    twist = (PI/10.0) * sin(i/200.0);
    angle += PI/30.0;
    delay_ms(150);
    increment_colour_pos(5);
  }
}

void sendvoxels_rand_z_colorwheel (int iterations, int delay, int wait)
{
  unsigned char x, y, last_x = 0, last_y = 0, i;
  uint8_t R, G, B;
  byte tempcolor1[3];
  byte tempcolor2[3];
  clearfast();
  for (x=0;x<4;x++)
  {
    for (y=0;y<4;y++)
    {
      get_colour(colourPos + 40*y, &R, &G, &B);
      LED(((rand()%2)*3), y, x, R, G, B);
    }
    increment_colour_pos(2);
  }

  for (i=0;i<iterations;i++)
  {
    x = rand()%4;
    y = rand()%4;
    if (y != last_y && x != last_x)
    {
      getColor(0,y,x,tempcolor1);
      getColor(3,y,x,tempcolor2);
      if (((tempcolor1[0]>0)||(tempcolor1[1]>0)||(tempcolor1[2]>0))&& ((tempcolor2[0]==0)&&(tempcolor2[1]==0)&&(tempcolor2[2]==0)))          
        sendvoxel_z(0,y,x,delay, tempcolor1[0], tempcolor1[1], tempcolor1[2]);
      if (((tempcolor2[0]>0)||(tempcolor2[1]>0)||(tempcolor2[2]>0))&& ((tempcolor1[0]==0)&&(tempcolor1[1]==0)&&(tempcolor1[2]==0)))         
        sendvoxel_z(3,y,x,delay, tempcolor2[0], tempcolor2[1], tempcolor2[2]);          
      delay_ms(wait);
      
      last_y = y;
      last_x = x;
    }    
  }
}
void sendvoxel_z (int z, int y, int x, int delay, byte red, byte green, byte blue)
{
  int i, ii;
  for (i=0; i<4; i++)
  {
    if (z == 3)
    {
      ii = 3-i;
      LED(ii+1,y,x,0,0,0);
    } else
    {
      ii = i;
      LED(ii-1,y,x,0,0,0);
    }
    LED(ii,y,x, red, green, blue);
    delay_ms(delay);
  }
}
void CalcRect(int8_t x1, int8_t y1, int8_t z1, int8_t x2, int8_t y2, int8_t z2, byte mode, int xColor)
{
  byte tangentAxis = 0; // 0=x, 1=y, 2=z
  int8_t a1, a2, b1, b2;
  getColor(xColor, 4);
  if (y1!=y2 && z1!=z2) {
    tangentAxis = 0; 
    a1=y1; 
    a2=y2; 
    b1=z1; 
    b2=z2;
  } // YZ Plane
  if (x1!=x2 && z1!=z2) {
    tangentAxis = 1; 
    a1=x1; 
    a2=x2; 
    b1=z1; 
    b2=z2;
  } // XZ plane
  if (x1!=x2 && y1!=y2) {
    tangentAxis = 2; 
    a1=x1; 
    a2=x2; 
    b1=y1; 
    b2=y2;
  } // XY Plane
  
  for (int8_t p=a1; p<=a2; p++) {
    for (int8_t q=b1; q<=b2; q++) {
      int8_t x,y,z;
      switch (tangentAxis) {
      case 0: 
        x=x1; 
        y=p; 
        z=q; 
        break; // YZ Plane
      case 1: 
        x=p; 
        y=y1; 
        z=q; 
        break; // XZ plane
      case 2: 
        x=p; 
        y=q; 
        z=z1; 
        break; // XY Plane
      }
      
      if (mode==1) LED(z,y,x, myred, mygreen, myblue); 
      else LED(z,y,x,0,0,0);
    }
  }
}

void CalcRect_colorwheel(int8_t x1, int8_t y1, int8_t z1, int8_t x2, int8_t y2, int8_t z2, byte mode)//, int xColor)
{
  byte tangentAxis = 0; // 0=x, 1=y, 2=z
  int8_t a1, a2, b1, b2;
  //getColor(xColor, 4);
  if (y1!=y2 && z1!=z2) {
    tangentAxis = 0; 
    a1=y1; 
    a2=y2; 
    b1=z1; 
    b2=z2;
  } // YZ Plane
  if (x1!=x2 && z1!=z2) {
    tangentAxis = 1; 
    a1=x1; 
    a2=x2; 
    b1=z1; 
    b2=z2;
  } // XZ plane
  if (x1!=x2 && y1!=y2) {
    tangentAxis = 2; 
    a1=x1; 
    a2=x2; 
    b1=y1; 
    b2=y2;
  } // XY Plane
  
  for (int8_t p=a1; p<=a2; p++) {
    for (int8_t q=b1; q<=b2; q++) {
      int8_t x,y,z;
      switch (tangentAxis) {
      case 0: 
        x=x1; 
        y=p; 
        z=q; 
        break; // YZ Plane
      case 1: 
        x=p; 
        y=y1; 
        z=q; 
        break; // XZ plane
      case 2: 
        x=p; 
        y=q; 
        z=z1; 
        break; // XY Plane
      }
      get_colour(colourPos + 10*(y+x), &R, &G, &B);
      if (mode==1) LED(z,y,x, R, G, B); 
      else LED(z,y,x,0,0,0);
    }
    increment_colour_pos(2);
  }
}

void DrawRect(int8_t x1, int8_t y1, int8_t z1, int8_t x2, int8_t y2, int8_t z2, int xColor)
{
  CalcRect(x1,y1,z1, x2,y2,z2, 1, xColor);
}

void DrawRect_colorwheel(int8_t x1, int8_t y1, int8_t z1, int8_t x2, int8_t y2, int8_t z2)
{
  CalcRect_colorwheel(x1,y1,z1, x2,y2,z2, 1);
}

void EraseRect(int8_t x1, int8_t y1, int8_t z1, int8_t x2, int8_t y2, int8_t z2)
{
  CalcRect(x1,y1,z1, x2,y2,z2, 0, 47);
}
void Droplets_colorwheel()
{
  for (int8_t waterLevel=0; waterLevel<4; waterLevel++)
  {
    for (int8_t z=3; z>waterLevel; z--)
    {
      DrawRect_colorwheel(1, 1, z, 2, 2, z); 
      delay(150);
      EraseRect(1, 1, z, 2, 2, z); 
    }
    
    for (int8_t i=0; i<2; i++)
    {
      DrawRect_colorwheel(1-i,1-i, waterLevel, 2+i, 2+i, waterLevel); 
      delay(250);
    }
  }
}
 
void Cosine(int myNumber){
  for (count=0; count<myNumber; count++){  
    for (int i=0; i<11; i++){
      getColor(i*4, 4);  
      for (byte xx=0; xx<4; xx++){ 
        for (byte yy=0; yy<4; yy++){
          z=((byte)(2+cos((xx/2.23)+(yy/2.23)+(float)3.6*i/6.28)*2));
          if (z>3) {
            z=3;
          }
          LED(z,yy, xx, myred, mygreen, myblue); 
        }
      } 
      delay(50);
      clearfast();
    }
  }
   delay(1000);
}

void intersectPlanes(int xColor1, int xColor2, int xColor3, int interactions)
// Vertical and horizontal intersecting planes moving end-to-end in unison
{
  const uint16_t  delayx = 150;
  byte color1[3], color2[3], color3[3];
  getColor(xColor1, 4);
  color1[0]=myred;
  color1[1]=mygreen;
  color1[2]=myblue;
  getColor(xColor2, 4);
  color2[0]=myred;
  color2[1]=mygreen;
  color2[2]=myblue;
  getColor(xColor3, 4);
  color3[0]=myred;
  color3[1]=mygreen;
  color3[2]=myblue;
  clearfast();
  for (int i=0; i<interactions; i++)
  {
    for (uint8_t p = 0; p < 4; p++)
    { // move plane one way ...
      setplane(AXIS_Z, p, color1[0], color1[1], color1[2]);
      setplane(AXIS_X, p, color2[0], color2[1], color2[2]);
      setplane(AXIS_Y, p, color3[0], color3[1], color3[2]);
      delay(delayx);
      clrplane(AXIS_Z, p);
      clrplane(AXIS_X, p);
      clrplane(AXIS_Y, p);
    }
    for (uint8_t p = 3; p>0; --p)
    { // ... reverse back
      setplane(AXIS_Z, p, color1[0], color1[1], color1[2]);
      setplane(AXIS_X, p, color2[0], color2[1], color2[2]);
      setplane(AXIS_Y, p, color3[0], color3[1], color3[2]);
      delay(delayx);
      clrplane(AXIS_Z, p);
      clrplane(AXIS_X, p);
      clrplane(AXIS_Y, p);
    }
  }          
}
