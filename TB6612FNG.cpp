extern unsigned int __bss_end;
extern unsigned int __heap_start;
extern void *__brkval;

#include "TB6612FNG.h"
#include <stdint.h>

#define NO_LINE  0

TB6612FNG::TB6612FNG( int PinsIn[], int freq_PWM, uint8_t Conecction)
{


  this->conec=Conecction;

  switch (Conecction){
    case 2:

    for (int i=0;i < 4 ;i++){
     this->Pins[i]=PinsIn[i];
     pinMode(this->Pins[i],OUTPUT);
   }


   setPwmFrequency(this->Pins[0],freq_PWM);
   setPwmFrequency(this->Pins[3],freq_PWM);
   if (PinsIn[0]==5 || PinsIn[3]==6 || PinsIn[0]==6 || PinsIn[3]==5)
   {
    num=1;
  }else{
    num=2;
  }
  break;
  default:

  for (int i=0;i < 6 ;i++){
   this->Pins[i]=PinsIn[i];
   pinMode(this->Pins[i],OUTPUT);
 }

 setPwmFrequency(this->Pins[0],freq_PWM);
 setPwmFrequency(this->Pins[5],freq_PWM);
 if (PinsIn[0]==5 || PinsIn[5]==6 || PinsIn[0]==6 || PinsIn[5]==5)
 {
  num=1;
}else{
  num=2;
}
break;
}

Fast_PWM_frequency = (float)((16000000L)/ ((int)freq_PWM) / (255) / (num)); 
rotating = true;
direction1 = FORWARD;
direction2 = FORWARD;
speed1 = 255;
speed2 = 255;
speed_regulation=255;
lastspeed_regulation=255;
SpeedLine=255;
}
TB6612FNG::~TB6612FNG(){
  delete[] Pins;
  if(Pins)
   free(Pins);
  

}
void TB6612FNG::Stop(int S_Time)
{
  switch (conec){
   case 2:
   setOutput(this->Pins[0], 0);
   setOutput(this->Pins[1], 0);
   setOutput(this->Pins[2], 0);
   setOutput(this->Pins[3], 0);
   break;
   default:
   setOutput(this->Pins[2], 0);
   setOutput(this->Pins[1], 0);
   analogWrite(this->Pins[0], 0);
   setOutput(this->Pins[3], 0);
   setOutput(this->Pins[4], 0);
   analogWrite(this->Pins[5], 0);
   break;
 }
 if( S_Time != 0){
  delay(S_Time);
}
rotating = false;
}
void TB6612FNG::setSpeed(int n_spd1, int n_spd2)
{
 speed1=constrain(n_spd1,(speed_regulation*-1),speed_regulation);
 speed2=constrain(n_spd2,(speed_regulation*-1),speed_regulation);
 fem1 =(speed1 > 30 | speed1 < -30);
 fem2 =(speed2 > 30 | speed2 < -30);

 if (fem1 == false )
 {
   speed1=0;
 }else if (fem2 == false)
 {
  speed2=0;
}
if( (speed1 !=0) || (speed2 !=0) ){
  rotating = true;
  if (rotating){
    if(speed1 < 0) {
      direction1=REVERSE;
    }
    else{
     direction1=FORWARD;
   }

   if(speed2<0) {
    direction2=REVERSE;
  }
  else {
    direction2=FORWARD;
  }
  switch (conec){
   case 2:
   if (direction1==FORWARD){
    if (abs(speed1) == 255)
    {
      setOutput(this->Pins[0],1);
    }else{
      analogWrite(this->Pins[0], abs(speed1));
    }
    setOutput(this->Pins[1], 0);
  }else{
   setOutput(this->Pins[0], 0);
   if (speed1 == 255)
   {
    setOutput(this->Pins[1],1);
  }else{
    analogWrite(this->Pins[1], speed1);
  }
  
}
if(direction2==REVERSE){
 setOutput(this->Pins[2], 0);
 if (abs(speed2) == 255)
 {
  setOutput(this->Pins[3],1);
}else{
 analogWrite(this->Pins[3], abs(speed2));
}
}else{
  setOutput(this->Pins[3], 0);
  if(speed2 == 255)
  {
    setOutput(this->Pins[2],1);
  }
else{
     analogWrite(this->Pins[2], speed2);
   }
 }
 break;
 default:
 if (direction1==FORWARD){
  setOutput(this->Pins[2], 1);
  setOutput(this->Pins[1], 0);
  if (abs(speed1) == 255){
    setOutput(this->Pins[0],1);
  }else{
   analogWrite(this->Pins[0], abs(speed1));
 }


}else{
 setOutput(this->Pins[2], 0);
 setOutput(this->Pins[1], 1);
 if (speed1 == 255){
  setOutput(this->Pins[0],1);
}else{
  analogWrite(this->Pins[0], speed1);
}

}
if(direction2==REVERSE){
 setOutput(this->Pins[3], 1);
 setOutput(this->Pins[4], 0);
 if (abs(speed2)== 255){
  setOutput(this->Pins[5],1);
}else{
 analogWrite(this->Pins[5], abs(speed2));
}
}else{
  setOutput(this->Pins[3], 0);
  setOutput(this->Pins[4], 1);
  if (speed2== 255){
    setOutput(this->Pins[5],1);
  }else{
    analogWrite(this->Pins[5], speed2);
  }
}
break;
}
}
}else{

  switch (conec){
   case 2:
   if (speed1 < 30)
   {
    setOutput(this->Pins[0], 0);
    setOutput(this->Pins[1], 0);
  }else if (speed2 < 30 )
  {
    setOutput(this->Pins[2], 0);
    setOutput(this->Pins[3], 0);
  }
  break;
  default:
  if (speed1 < 30  )
  {
    setOutput(this->Pins[2], 0);
    setOutput(this->Pins[1], 0);
    setOutput(this->Pins[0], 0);
  }else if (speed2 < 30 )
  {
    setOutput(this->Pins[3], 0);
    setOutput(this->Pins[4], 0);
    setOutput(this->Pins[5], 0);
  }
  break;
}
rotating=false;
}
}
void TB6612FNG::ChangeSpeedRegulation(int n_spd_reg,int SpeedMax)
{
 speed_regulation = n_spd_reg;
 lastspeed_regulation = n_spd_reg;
 if(SpeedMax != NO_LINE){
  SpeedLine=SpeedMax;
} else{
  SpeedLine=n_spd_reg;
}

}
uint8_t TB6612FNG::GetSpeedRegulation()
{
 return speed_regulation;
}
void TB6612FNG::setPwmFrequency(uint8_t pin, int divisor) {

  if(pin == 5 || pin == 6 || pin == 9 || pin == 10) {
    switch(divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 64: mode = 0x03; break;
      case 256: mode = 0x04; break;
      case 1024: mode = 0x05; break;
      default: return;
    }
    if(pin == 5 || pin == 6) {
      TCCR0B = TCCR0B & 0b11111000 | mode;
      TCCR0A = _BV(COM0A1) | _BV(COM0B1) | _BV(WGM01) | _BV(WGM00); 
      TCCR0B = _BV(CS00); 
    } else {
      TCCR1B = TCCR1B & 0b11111000 | mode;

    }
  } else if(pin == 3 || pin == 11) {
    switch(divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 32: mode = 0x03; break;
      case 64: mode = 0x04; break;
      case 128: mode = 0x05; break;
      case 256: mode = 0x06; break;
      case 1024: mode = 0x7; break;
      default: return;
    }
    TCCR2B = TCCR2B & 0b11111000 | mode;
    TCCR2A = _BV(COM2A1) | _BV(COM2B1) | _BV(WGM21) | _BV(WGM20);
    TCCR2B = _BV(CS22);
  }

}
double TB6612FNG::GetFrequency(){
  return Fast_PWM_frequency;
}
void TB6612FNG::setOutput(uint8_t pin,boolean state){

  if(state){
    switch(pin){
     case 0:PORTD |= (1 << PORTD0);break;
     case 1:PORTD |= (1 << PORTD1);break;
     case 2:PORTD |= (1 << PORTD2);break;
     case 3:PORTD |= (1 << PORTD3);break;
     case 4:PORTD |= (1 << PORTD4);break;
     case 5:PORTD |= (1 << PORTD5);break;
     case 6:PORTD |= (1 << PORTD6);break;
     case 7:PORTD |= (1 << PORTD7);break;
     case 8:PORTB |= (1 << PORTB0);break;
     case 9:PORTB |= (1 << PORTB1);break;
     case 10:PORTB |= (1 << PORTB2);break;
     case 11:PORTB |= (1 << PORTB3);break;
     case 12:PORTB |= (1 << PORTB4);break;
     case 13:PORTB |= (1 << PORTB5);break;
     case 14:PORTC |= (1 << PORTC0);break;
     case 15:PORTC |= (1 << PORTC1);break;
     case 16:PORTC |= (1 << PORTC2);break;
     case 17:PORTC |= (1 << PORTC3);break;
     case 18:PORTC |= (1 << PORTC4);break;
     case 19:PORTC |= (1 << PORTC5);break;
     default: return;
   }
 }else{
  switch(pin){
   case 0:PORTD &= ~(1 << PORTD0);break;
   case 1:PORTD &= ~(1 << PORTD1);break;
   case 2:PORTD &= ~(1 << PORTD2);break;
   case 3:PORTD &= ~(1 << PORTD3);break;
   case 4:PORTD &= ~(1 << PORTD4);break;
   case 5:PORTD &= ~(1 << PORTD5);break;
   case 6:PORTD &= ~(1 << PORTD6);break;
   case 7:PORTD &= ~(1 << PORTD7);break;
   case 8:PORTB &= ~(1 << PORTB0);break;
   case 9:PORTB &= ~(1 << PORTB1);break;
   case 10:PORTB &= ~(1 << PORTB2);break;
   case 11:PORTB &= ~(1 << PORTB3);break;
   case 12:PORTB &= ~(1 << PORTB4);break;
   case 13:PORTB &= ~(1 << PORTB5);break;
   case 14:PORTC &= ~(1 << PORTC0);break;
   case 15:PORTC &= ~(1 << PORTC1);break;
   case 16:PORTC &= ~(1 << PORTC2);break;
   case 17:PORTC &= ~(1 << PORTC3);break;
   case 18:PORTC &= ~(1 << PORTC4);break;
   case 19:PORTC &= ~(1 << PORTC5);break;
   default: return;
 }
}

}

void TB6612FNG::FollowerLinePID(double setPoint, 
  double InputMax, 
  double Kp, 
  double Ki,
  double Kd, 
  int maxTime){

  this->_Kp=Kp;
  this->_Kd=Kd;
  this->_Ki=Ki;
  this->ref=InputMax;
  this->_setPoint = (double)map(setPoint, 0, ref, 0, 100);
  this->_setPoint = (double)constrain(this->_setPoint, 0, 100);
  lastError=0;
  integral=0;
  this->_maxTime=maxTime;
  followerLine=false;
  Flag=false;
  Flag2=false;
  
   if (maxTime == 0)
  {
    cont=-1;
  }else{
    cont=0;
  }
  Time = millis();
}

bool TB6612FNG::ControlPID(double input){

/*
   * Pseudo code (source Wikipedia)
   * 
     previous_error = 0
     integral = 0 
   start:
     error = setpoint – PV [actual_position]
     integral = integral + error*dt
     derivative = (error - previous_error)/dt
     output = Kp*error + Ki*integral + Kd*derivative
     previous_error = error
     wait(dt)
     goto start
   */
 
  input = (double)map(input, 0, this->ref, 0, 100);
  input = constrain(input,0,100);
  error = (double)(input - this->_setPoint );

 switch((int)error) {
       case 1:
              error=0;
              break;
      case -1:
             error=0;
             break;
 }
  integral =(long)(numpd((2/3) * integral + error)); //Saturate integral Value
  derivative = error - lastError;

  motorSpeed = (double)((this->_Kp * error) + (this->_Ki * integral) + (this->_Kd * derivative));

  if (followerLine != true){
    Flag=false;
    speed_regulation=lastspeed_regulation;

  }else{

    if(Flag==false){
      cont++;
      Flag=true;
    }

    speed_regulation=SpeedLine;
  }

  rightMotorSpeed = numpd((speed_regulation + motorSpeed));

  leftMotorSpeed = numpd((speed_regulation - motorSpeed));

  if (rightMotorSpeed > speed_regulation ) rightMotorSpeed = speed_regulation; 
  if (leftMotorSpeed > speed_regulation ) leftMotorSpeed = speed_regulation; 
  if (rightMotorSpeed < 0) rightMotorSpeed = 0; 
  if (leftMotorSpeed < 0) leftMotorSpeed = 0; 

  if(this->_maxTime != NO_LINE){
    if ( (error != lastError) && (millis() - Time > 0) ) {
      setOutput(13,0);
      followerLine=false;
      Time=millis();
      return false;
    } else{
     if ((millis() - Time > this->_maxTime ) && (error == lastError) && (lastError == 0)  ){
      setOutput(13,1);
      if( duration == lastduration){
        duration=_maxTime;
      }
      followerLine=true;
      return true;
      Time=millis();
    }else{
      duration= millis() - Time;
      lastduration=duration;
      followerLine=false;
      return false;
    }
  }
}else{
  if( error == lastError && lastError == 0){
    setOutput(13,1);
    duration= millis() - Time;
    Flag2=false;
    return true;
  }else{
    if (Flag2==false)
    {
      cont++;
      Flag2=true;
    }

    setOutput(13,0);
    Time=millis();
    return false;
  }
}

lastError = error;
}
int TB6612FNG::numpd(double x){  // ingresa un decimal con dos unidades despues del punto flotante para aproximarlo
  entero = int(x);
  decimal = int(abs((entero - x)) * 100); // EJ: 77.98 el decimal es 98 retorna un 1 para que sea sumado 
  if ( decimal >=50 ){
    return (entero+1); //Devuelve un 1 o un 0 si se cumple o no el rango de aproximación
  }
  else{
    return (entero); 
  }
}

int freeMemory() {
  int free_memory;

  if((int)__brkval == 0)
   free_memory = ((int)&free_memory) - ((int)&__bss_end);
 else
  free_memory = ((int)&free_memory) - ((int)__brkval);

return free_memory;
}


