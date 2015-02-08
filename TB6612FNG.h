/*
A library to controll the TB6612FNG Toshiba H-bridge
-----------------------------------------------------------------

The pins must be connected accordingly (see the datasheet of the H-bridge).
    -the PWM pins must support PWM on the Arduino
    -the enable pin on the H-bridge should be connected either to 5V
     or to a pin on the Arduino, which is set to HIGH, for the H-bridge to function

contact me: msvargas97@gmail.com.

I hope this library will help you in your projects!

PWM setPwmFrequency reference=
http://forum.arduino.cc/index.php/topic,16612.0.html#4

*/
#ifndef TB6612FNG_h
#define TB6612FNG_h
#include <arduino.h>
#include <avr/io.h>

#define FORWARD true
#define REVERSE false
#define PWM_FREQUENCY_MAX 1
#define PWM_FREQUENCY_DEFAULT 64
#define PWM_FREQUENCY_MIN 1024
#define LIBRARY_VERSION 2.0.0

class TB6612FNG
{
 private:

   void setPwmFrequency(byte pin, int divisor);
   byte Pins[6];
   byte mode;
   boolean rotating;
   boolean direction1;
   boolean direction2;
   int speed1;
   int speed2;
   int speed_regulation;
   int entero;
   int decimal; 
   double _Kp;
   double _Kd;
   double _Ki;
   double _setPoint;
   double lastError;
   long integral;
   double ref;
   unsigned long _maxTime;
   boolean followerLine;
   boolean Flag;

 public:

   TB6612FNG( byte PinsIn[], int freq_PWM);
   ~TB6612FNG();
   void Stop();
   void setSpeed(int n_spd1, int n_spd2);
   void ChangeSpeedRegulation(int n_spd_reg);
   byte GetSpeedRegulation();
   bool ControlPID(double input);
   void setOutput(byte pin, boolean state);
   void FollowerLinePID(double setPoint, double InputMax, double Kp, double Ki, double Kd,int maxTime=0);
   int numpd(double x);
   int rightMotorSpeed;
   int leftMotorSpeed;
   double motorSpeed;
   double error;
   unsigned long Time;
   unsigned int cont;
};

#endif 