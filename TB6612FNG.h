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
#include <Arduino.h>
#include <avr/io.h>
#include <stdint.h>

#define FORWARD true
#define REVERSE false
#define PWM_FREQUENCY_MAX 1
#define PWM_FREQUENCY_DEFAULT 64
#define PWM_FREQUENCY_MIN 1024
#define LIBRARY_VERSION 2.0.0

#ifdef __cplusplus
extern "C" {
#endif
int freeMemory();

#ifdef  __cplusplus
}
#endif

class TB6612FNG
{
 private:

   void setPwmFrequency(uint8_t pin, int divisor);
   int Pins[6];
   uint8_t conec;
   byte num;
   boolean rotating;
   boolean fem1;
   boolean fem2;
   boolean direction1;
   boolean direction2;
   boolean followerLine;
   boolean Flag;
   boolean Flag2;
   int speed1;
   int speed2;
   int speed_regulation;
   int entero;
   int decimal;
   int lastspeed_regulation;
   int SpeedLine; 
   int mode;
   double _Kp;
   double _Kd;
   double _Ki;
   double _setPoint;
   double derivative;
   double lastError;
   double ref;
   int32_t integral;
   uint16_t _maxTime;
   uint32_t lastduration;
   double Fast_PWM_frequency;
 public:

   TB6612FNG( int PinsIn[], int freq_PWM, uint8_t Conecction=0);
   ~TB6612FNG();
   int rightMotorSpeed;
   int leftMotorSpeed;
   double error;
   unsigned int cont;
   double motorSpeed;
   uint32_t Time;
   uint32_t duration;
   void ChangeSpeedRegulation(int n_spd_reg,int SpeedMax=0);
   bool ControlPID(double input);
   int numpd(double x);
   void FollowerLinePID(double setPoint, double InputMax, double Kp, double Ki, double Kd,int maxTime=0);
   uint8_t GetSpeedRegulation();
   void setOutput(uint8_t pin, boolean state);
   void setSpeed(int n_spd1, int n_spd2);
   void Stop(int S_Time=0);
   double GetFrequency();

};

#endif 