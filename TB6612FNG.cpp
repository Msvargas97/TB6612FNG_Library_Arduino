#include "TB6612FNG.h"

#define NO_LINE    0

TB6612FNG::TB6612FNG( byte PinsIn[], int freq_PWM)
{

 for (byte i=0;i<6;i++){
   this->Pins[i]=PinsIn[i];
   pinMode(this->Pins[i],OUTPUT);
 }

 if( freq_PWM != PWM_FREQUENCY_DEFAULT ){
   setPwmFrequency(this->Pins[0],freq_PWM);
   setPwmFrequency(this->Pins[5],freq_PWM);
 }


 rotating = true;
 direction1 = FORWARD;
 direction2 = FORWARD;
 speed1 = 255;
 speed2 = 255;
 speed_regulation = 255;
}
TB6612FNG::~TB6612FNG(){
delete[] Pins;
if (Pins)
    free(Pins);
}
void TB6612FNG::Stop()
{

 setOutput(this->Pins[2], 0);
 setOutput(this->Pins[1], 0);
 analogWrite(this->Pins[0], 0);
 setOutput(this->Pins[3], 0);
 setOutput(this->Pins[4], 0);
 analogWrite(this->Pins[5], 0);
 rotating = false;
}
void TB6612FNG::setSpeed(int n_spd1, int n_spd2)
{
 speed1=constrain(n_spd1,(speed_regulation*-1),speed_regulation);
 speed2=constrain(n_spd2,(speed_regulation*-1),speed_regulation);
 if( speed1 > 30 || speed2 > 30 ){
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
else {direction2=FORWARD;
}

if (direction1==REVERSE){
  setOutput(this->Pins[2], 1);
  setOutput(this->Pins[1], 0);
  analogWrite(this->Pins[0], abs(speed1));
}else{
 setOutput(this->Pins[2], 0);
 setOutput(this->Pins[1], 1);
 analogWrite(this->Pins[0], speed1);
}
if(direction2==FORWARD){
 setOutput(this->Pins[3], 1);
 setOutput(this->Pins[4], 0);
 analogWrite(this->Pins[5], abs(speed2));

}else{
  setOutput(this->Pins[3], 0);
  setOutput(this->Pins[4], 1);
  analogWrite(this->Pins[5], speed2);
}
}

}else{
  rotating=false;
  if (speed1 <= 30)
  {
  setOutput(this->Pins[2], 0);
  setOutput(this->Pins[1], 0);
  analogWrite(this->Pins[0], 0);
  }else if (speed2 <= 30 )
  {
  setOutput(this->Pins[3], 0);
  setOutput(this->Pins[4], 0);
  analogWrite(this->Pins[5], 0);
  }
}
}
void TB6612FNG::ChangeSpeedRegulation(int n_spd_reg)
{
 speed_regulation = n_spd_reg;
}
byte TB6612FNG::GetSpeedRegulation()
{
 return speed_regulation;
}
void TB6612FNG::setPwmFrequency(byte pin, int divisor) {

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
  }
}
void TB6612FNG::setOutput(byte pin,boolean state){

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
  Time = millis();
}

bool TB6612FNG::ControlPID(double input){
  input = (double)map(input, 0, this->ref, 0, 100);
  input = (double)constrain(input, 0, 100);
  error = (double)(input - this->_setPoint);
  if(error <=1 && error >=-1){
    error=0;
  }
 motorSpeed = (double)((this->_Kp * error) + (this->_Ki*integral) + (this->_Kd * (error - lastError)));
rightMotorSpeed = numpd((255 + motorSpeed));
leftMotorSpeed = numpd((255 - motorSpeed));
integral = (long)(integral + error);

if (followerLine != true){
  Flag=false;
if (rightMotorSpeed > speed_regulation ) rightMotorSpeed = speed_regulation; 

if (leftMotorSpeed > speed_regulation ) leftMotorSpeed = speed_regulation; 
}else{
  if(Flag==false){
  cont++;
  Flag=true;
}
  rightMotorSpeed = 255; //Full Speed in the Line 
  leftMotorSpeed = 255;
}
if (rightMotorSpeed < 0) rightMotorSpeed = 0; 

if (leftMotorSpeed < 0) leftMotorSpeed = 0; 

if(_maxTime != NO_LINE){
if ( (error != lastError) && (millis() - Time > 0) ) {
     Time=millis();
} else{
 if ((millis() - Time >= _maxTime ) && (error == lastError) && (lastError == 0)  ){

followerLine=true;
return true;
Time=millis();
  }else{

    followerLine=false;
   return false;
 }
}
}else{
  if( error== lastError && lastError==0){
    return true;
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