#include <Adafruit_NeoPixel.h>
#define RINGBANDPIN 11     //define pin for led band output 
#define RINGLED_NUM 12     //define number of led in led band
//#define RINGLED_NUM 16   //define 16 LED band
Adafruit_NeoPixel strip = Adafruit_NeoPixel(RINGLED_NUM,RINGBANDPIN, NEO_GRB + NEO_KHZ800 );//instance for lightning

uint32_t RED=strip.Color(255,0,0);
uint32_t ORA=strip.Color(255,143,0);
uint32_t YEL=strip.Color(255,255,0);
uint32_t GRE=strip.Color(127,255,0);
uint32_t CYA=strip.Color(0,255,255);
uint32_t BLU=strip.Color(0,127,255);
uint32_t PUR=strip.Color(127,0,255);
uint32_t WHI=strip.Color(255,255,255);
uint32_t RAI[7]={RED,ORA,YEL,GRE,CYA,BLU,PUR};


void setup() {
   Serial.begin(9600);
   strip.begin();
   strip.setBrightness(30);
}

void loop() {

  for(uint8_t i=1;i<=7;i++){
    delay(2000);
    for(uint8_t j=1;j<=RINGLED_NUM;j++){
      strip.setPixelColor(j,RAI[i]);
      strip.show();
    }
  }
}

