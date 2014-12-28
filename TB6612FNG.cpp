#include "TB6612FNG.h"

TB6612FNG::TB6612FNG( byte PinsIn[], int freq_PWM)
{

 for (byte i=0;i<6;i++){
   Pins[i]=PinsIn[i];
   pinMode(Pins[i],OUTPUT);
 }

 if( freq_PWM != PWM_FREQUENCY_DEFAULT ){
   setPwmFrequency(Pins[0],freq_PWM);
   setPwmFrequency(Pins[5],freq_PWM);
 }


 rotating = true;
 direction1 = FORWARD;
 direction2 = FORWARD;
 speed1 = 255;
 speed2 = 255;
 speed_regulation = 255;
}

void TB6612FNG::Stop()
{

 setOutput(Pins[2], 0);
 setOutput(Pins[1], 0);
 analogWrite(Pins[0], 0);
 setOutput(Pins[3], 0);
 setOutput(Pins[4], 0);
 analogWrite(Pins[5], 0);
 rotating = false;
}
void TB6612FNG::setSpeed(int n_spd1, int n_spd2)
{
 speed1=constrain(n_spd1,(speed_regulation*-1),speed_regulation);
 speed2=constrain(n_spd2,(speed_regulation*-1),speed_regulation);
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
  setOutput(Pins[2], 1);
  setOutput(Pins[1], 0);
  analogWrite(Pins[0], abs(speed1));
}else{
 setOutput(Pins[2], 0);
 setOutput(Pins[1], 1);
 analogWrite(Pins[0], speed1);
}
if(direction2==REVERSE){
 setOutput(Pins[3], 1);
 setOutput(Pins[4], 0);
 analogWrite(Pins[5], abs(speed2));

}else{
  setOutput(Pins[3], 0);
  setOutput(Pins[4], 1);
  analogWrite(Pins[5], speed2);
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
     case 0:
     PORTD |= (1 << PORTD0);
     break;
     case 1:
     PORTD |= (1 << PORTD1);
     break;
     case 2:
     PORTD |= (1 << PORTD2);
     break;
     case 3:
     PORTD |= (1 << PORTD3);
     break;
     case 4:
     PORTD |= (1 << PORTD4);
     break;
     case 5:
     PORTD |= (1 << PORTD5);
     break;
     case 6:
     PORTD |= (1 << PORTD6);
     break;
     case 7:
     PORTD |= (1 << PORTD7);
     break;
     case 8:
     PORTB |= (1 << PORTB0);
     break;
     case 9:
     PORTB |= (1 << PORTB1);
     break;
     case 10:
     PORTB |= (1 << PORTB2);
     break;
     case 11:
     PORTB |= (1 << PORTB3);
     break;
     case 12:
     PORTB |= (1 << PORTB4);
     break;
     case 13:
     PORTB |= (1 << PORTB5);
     break;
     case 14:
     PORTC |= (1 << PORTC0);
     break;
     case 15:
     PORTC |= (1 << PORTC1);
     break;
     case 16:
     PORTC |= (1 << PORTC2);
     break;
     case 17:
     PORTC |= (1 << PORTC3);
     break;
     case 18:
     PORTC |= (1 << PORTC4);
     break;
     case 19:
     PORTC |= (1 << PORTC5);
     break;
   }
 }else{
  switch(pin){
   case 0:
   PORTD &= ~(1 << PORTD0);
   break;
   case 1:
   PORTD &= ~(1 << PORTD1);
   break;
   case 2:
   PORTD &= ~(1 << PORTD2);
   break;
   case 3:
   PORTD &= ~(1 << PORTD3);
   break;
   case 4:
   PORTD &= ~(1 << PORTD4);
   break;
   case 5:
   PORTD &= ~(1 << PORTD5);
   break;
   case 6:
   PORTD &= ~(1 << PORTD6);
   break;
   case 7:
   PORTD &= ~(1 << PORTD7);
   break;
   case 8:
   PORTB &= ~(1 << PORTB0);
   break;
   case 9:
   PORTB &= ~(1 << PORTB1);
   break;
   case 10:
   PORTB &= ~(1 << PORTB2);
   break;
   case 11:
   PORTB &= ~(1 << PORTB3);
   break;
   case 12:
   PORTB &= ~(1 << PORTB4);
   break;
   case 13:
   PORTB &= ~(1 << PORTB5);
   break;
   case 14:
   PORTC &= ~(1 << PORTC0);
   break;
   case 15:
   PORTC &= ~(1 << PORTC1);
   break;
   case 16:
   PORTC &= ~(1 << PORTC2);
   break;
   case 17:
   PORTC &= ~(1 << PORTC3);
   break;
   case 18:
   PORTC &= ~(1 << PORTC4);
   break;
   case 19:
   PORTC &= ~(1 << PORTC5);
   break;
 }
}

}

void TB6612FNG::FollowerLinePID(int setPoint,float Kp,float Kd, float Ki){

  _setPoint=setPoint;
  _Kp=Kp;
  _Kd=Kd;
  _Ki=Ki;
  lastError=0;
  integral=0;

}

void TB6612FNG::ControlPID(int input){
  error=input - _setPoint;
  
   motorSpeed = (_Kp * error) + ((_Ki*integral)/10000) + (_Kd * (error - lastError));


lastError = error;
integral+=error;

rightMotorSpeed = 255 + motorSpeed;
leftMotorSpeed = 255 - motorSpeed;

if (rightMotorSpeed > speed_regulation )rightMotorSpeed = speed_regulation; 

if (leftMotorSpeed > speed_regulation ) leftMotorSpeed = speed_regulation; 

if (rightMotorSpeed < 0) rightMotorSpeed= 0; 

if (leftMotorSpeed < 0) leftMotorSpeed = 0; 


}
