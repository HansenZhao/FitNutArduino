#include <SoftwareSerial.h> 
#include <Adafruit_NeoPixel.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif
//====================================================================================
#define PUSHBUTTON  8      //define pin for crash sensor 
#define TOUCH  8           //define pin for touch sensor 
#define TONE   10          //define pin for tone output
#define RINGBANDPIN 11     //define pin for led band output 
#define RINGLED_NUM 12     //define number of led in led band

#define  ADDSTATE 0        //define constant in add state for slleplight
#define  SUBSTATE 1        //define constant in sub state for sleeplight 

boolean LedState=ADDSTATE;

#define OUTPUT_READABLE_YAWPITCHROLL
#define SERIAL_BUFFER_SIZE 64
//====================================================================================
MPU6050 mpu;
Adafruit_NeoPixel strip = Adafruit_NeoPixel(RINGLED_NUM,RINGBANDPIN, NEO_GRB + NEO_KHZ800 );//instance for lightning
SoftwareSerial mySerial(4, 5); //instance for BlueTooth network 
#define my_Serial Serial1  //Core+ 
int GobalLoopTime=0; //Gobal variable for loop time
unsigned int MaxBrightness=20;  //TUNABLE: Max Brightness of led,CurBrightness should be in the same value initially
unsigned int CurBrightness=20;  //initiation of brightness in sleeplight

bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
uint16_t    allaa;       // total accel

int  Rnum=0;            //initiation of repeat number
int  cRnum=0;           //initiation of current repeat number
int  cGnum=0;           //initiation of current group number
int  Gnum=0;            //initiation of group number
int  tmpnum=0;          //
char tmp;               //initiation of BlueTooth Cache Data
String sGnum="";        //initiation of Group Number String
String sRnum="";        //initiation of Repeat Number String
unsigned long time1=0,time2=0; //initiation of Touch timer
boolean ifwork=false;   //indicate current work mode
boolean ifbt=false;     //indicate current bluetooth state
boolean ifalldone=false;//indicate if exercise plan done

boolean curmode=1;     // touch mode
uint8_t cishu=0;       // time of touch mode changes
//===============================Color Defination=====================================
uint32_t RED=strip.Color(255,0,0);
uint32_t ORA=strip.Color(255,143,0);
uint32_t YEL=strip.Color(255,255,0);
uint32_t GRE=strip.Color(127,255,0);
uint32_t CYA=strip.Color(0,255,255);
uint32_t BLU=strip.Color(0,127,255);
uint32_t PUR=strip.Color(127,0,255);
uint32_t WHI=strip.Color(255,255,255);
uint32_t RAI[7]={RED,ORA,YEL,GRE,CYA,BLU,PUR};
//====================================================================================
// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
    
    Serial.begin(9600);
    mySerial.begin(9600);
    pinMode(TOUCH,INPUT);
    pinMode(TONE,OUTPUT);
    
    strip.begin();
    while (!Serial); // wait for Leonardo enumeration, others continue immediately
    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3v or Ardunio
    // Pro Mini running at 3.3v, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    while (Serial.available() && Serial.read()); // empty buffer again

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
  //=============================touch sensor===========================================
    if (isDoubleClick(TOUCH,180)){  //TUNABLE: 100~200 larger the number is, the easier double touch be detected
        if(Gnum!=0&&Rnum!=0&&!ifalldone){
        ifwork=true;
        multiTone(TONE,600,500,1);  //TUNABLE: 600 indicate frequency;500 indicate duration;1 indicate repeat time
        clearAll();
        intiBrightness();
        showPercentage(ORA,cGnum,Gnum);
        }else if(ifalldone){
          Initialize();
        }
    }
  //=====================================================================================
  
  //================================bluetooth============================================
     while (Serial.available() > 0) {   //REPLACE mySerial.available()>0
         tmp=Serial.read();             //REPLACE mySerial.read()
         if(tmp=='G'){
           delay(2);
           Gnum=getGroupNumber();
           sGnum = "";
           Serial.println(Gnum);
         }else if(tmp=='N'){
           delay(2);
           Rnum=getRepeatNumber();
           sRnum = "";
           Serial.println(Rnum);
         }else if(tmp=='Y'){
           clearAll();
           delay(800);
           ifbt=true;
           intiBrightness();
           adFlash(BLU,100,2);
           delay(400);
         }else{
           delay(2);
         }  
     }  
  //===========================================================================================
  
  //===============================Led work module=============================================
    if(ifwork&&!ifalldone){
      //============================counting module============================================
        // if programming failed, don't try to do anything
      if (!dmpReady) return;
        // reset interrupt flag and get INT_STATUS byte
      mpuInterrupt = false;
      mpuIntStatus = mpu.getIntStatus();   
        // get current FIFO count
      fifoCount = mpu.getFIFOCount();
        // check for overflow (this should never happen unless our code is too inefficient)
      if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
            // reset so we can continue cleanly
            mpu.resetFIFO();
            loop();
        // otherwise, check for DMP data ready interrupt (this should happen frequently)
      } else if (mpuIntStatus & 0x02) {
            // wait for correct available data length, should be a VERY short wait
            while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
            // read a packet from FIFO
            mpu.getFIFOBytes(fifoBuffer, packetSize);        
            // track FIFO count here in case there is > 1 packet available
            // (this lets us immediately read more without waiting for an interrupt)
            fifoCount -= packetSize;
            // display real acceleration, adjusted to remove gravity
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
    
            allaa=abs(aaReal.x+aaReal.y+aaReal.z);  
            if(allaa>10000){   //TUNABLE: the larger the number is, the less sensitivity the device will have
              delay(400);   //TUNABLE: make sure every action counting once by decrease sensitivity or increse
                            //delay time
              Serial.println(allaa);
              cRnum++;
              // REPLACE String strbuff=String(cRnum);
              // char mes[3];
              // strbuff.toCharArray(mes,3);
              // mySerial.println(mes);
              Serial.println(cRnum);
            }
        }
        int restnum=Rnum-cRnum;
        //==============================================================================
        if(restnum<=0){
          // one group finished
          multiTone(TONE,1800,100,2);
          multiTone(TONE,1800,300,1);
          Serial.println("Finish");    // REPLACE mySerial.write("Finish"); 
          cGnum++;
          if(cGnum>=Gnum){
            // all group done
            ifwork=false;
            ifalldone=true;
            Serial.println("All Done!");    
          }else{
            // subgroup done
            ifwork=false;
            intiBrightness();
            adFlash(ORA,100,2);
            cRnum=0;
            tmpnum=0;
            multiTone(TONE,600,500,1);
          } 
        }else if (restnum>0 && restnum<=3&&tmpnum!=cRnum){
          // Almost done
          multiTone(TONE,1000+200*(3-restnum),50,2);
          tmpnum=cRnum;
        }
        intiBrightness();
        showPercentage(ORA,cGnum,Gnum);  
    }else if(!ifwork&&!ifalldone) {
  //=============================Led waiting module======================================
        delay(50);                 //control the speed of sleep light
        checkAndSetBrightness(MaxBrightness);
        if(ifbt&&!cGnum) simpleColorRun(GRE,0);
        else if(cGnum) {
          intiBrightness();
          showPercentage(ORA,cGnum,Gnum);
        }else     simpleColorRun(WHI,0);
    }else if(!ifwork&&ifalldone) showRainbow(100);  //TUNABLE: 100 indicate loop speed
}


//======================================================================================
void simpleColorRun(uint32_t color,int time){
  for(uint16_t i=0; i<RINGLED_NUM; i++){
    strip.setPixelColor(i,color);
    strip.show();
    delay(time);
  }
}
// light down all leds
void clearAll(){
    for(uint16_t i=0; i<RINGLED_NUM; i++){
    strip.setPixelColor(i,strip.Color(0,0,0));
    strip.show();
  }
 }

// flash whole band lasting "time"
void simpleFlash(uint32_t color, int time){
  for(uint8_t i=0;i<RINGLED_NUM;i++){
    strip.setPixelColor(i,color);
    strip.show();
  }
  delay(time);
  clearAll();
}
// flash for many times
void adFlash(uint32_t color,int time,int count){
  for(uint8_t i=0;i<count;i++){
    simpleFlash(color,time);
    delay(time);
  }
}
// show cur/total percentage
void showPercentage(uint32_t color,int cur,int total){
  int per=(cur*RINGLED_NUM)/total;
  for(uint8_t i=0;i<=per;i++){
        strip.setPixelColor(i,color);
        strip.show();
      }  
}

// check and set brightness in every loop for sleeplight or other effect
void checkAndSetBrightness(int MaxBrightness){
  if(CurBrightness>=MaxBrightness){
    LedState=SUBSTATE; 
  }else if(CurBrightness<=0){
    LedState=ADDSTATE;
  }
  if(LedState==SUBSTATE){
    CurBrightness--;
  }else{
    CurBrightness++;
  }
  strip.setBrightness(CurBrightness); 
}

void showRainbow(int time){
  for(uint16_t i=1;i<8;i++){
    strip.setPixelColor(((i+GobalLoopTime)%RINGLED_NUM),RAI[i]);
    strip.show();
  }
  delay(time);
  clearAll();
  GobalLoopTime=(GobalLoopTime+1)%RINGLED_NUM;
}

//Sensing if Double Click accur
boolean isDoubleClick(int pin,int interval){
  if(curmode!=digitalRead(pin)){
    curmode=!curmode;
    time2=millis();
    if((time2-time1)<interval){
      cishu++;
    }else{
      cishu=0;
    }
    time1=time2;
    if(cishu>=3){
      cishu=0;
      return true;  
    }else{
      return false;
    }
  }else{
    return false;
  }
}


void Initialize(){
  ifwork=false;
  ifbt=false;
  ifalldone=false;
  cRnum=0;
  cGnum=0;
  Rnum=0;
  Gnum=0;
  setup();
}

int getGroupNumber(){
  while(1){
    tmp=Serial.read();  //REPLACE mySerial.read();
    if(tmp=='D'){
      delay(2);
      return sGnum.toInt();
    }
    sGnum+=tmp;
    delay(2);
  }
}

int getRepeatNumber(){
  while(1){
    tmp=Serial.read(); //REPLACE tmp=mySerial.read();
    if(tmp=='D'){
      return sRnum.toInt();
    }
    sRnum+=tmp;
    delay(2);
  }
}

void multiTone(int pin,int freg,int durtime,int time){
  for(int i=0;i<time;i++){
    tone(pin,freg);
    delay(durtime);
    noTone(pin);
    delay(durtime);
  }
}

void intiBrightness(){
  strip.setBrightness(MaxBrightness);
  CurBrightness=MaxBrightness;
}
