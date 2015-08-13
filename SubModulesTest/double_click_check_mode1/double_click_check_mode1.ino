unsigned long time1=0,time2=0; //initiation of Touch timer
boolean curmode=1;
uint8_t cishu=0;

void setup(){
  Serial.begin(9600);
  pinMode(8,INPUT);
}
void loop(){
  if(isDoubleClick(8,180)){
    Serial.println("fsdfsdfs");
  }
}
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
    if(cishu==1){
      digitalWrite(MOTION,HIGH);
      delay(30);
      digitalWrite(MOTION,LOW);
    }else if(cishu>=3){
      digitalWrite(MOTION,HIGH);
      delay(30);
      digitalWrite(MOTION,LOW);
      cishu=0;
      return true;  
    }else{
      return false;
    }
  }else{
    return false;
  }
}


