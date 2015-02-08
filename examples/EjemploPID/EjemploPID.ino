#include <TB6612FNG.h>

#define PWM_FREQ PWM_FREQUENCY_MAX
#define Kp 2.23
#define Kd 1.16
#define Ki 0
#define setPoint 3500


TB6612FNG motores((byte[]) {
11, A0, A1, A3, A4, 10
},PWM_FREQ);
 int sensorValue; 

void setup(){
 Serial.begin(9600);
 pinMode(A2,OUTPUT);
 pinMode(13,OUTPUT);
 motores.setOutput(13,1);
 motores.setOutput(13,0);
 motores.ChangeSpeedRegulation(230);
 motores.FollowerLinePID(3500,7000,Kp,Ki,Kd,2500);
}

void loop(){
  motores.setOutput(16,HIGH);
  sensorValue = map(analogRead(A6),0,1023,0,7000);
Serial.print("  Linea recta->");
Serial.print(motores.ControlPID((double)sensorValue));
Serial.print("  # de veces->");
Serial.print(motores.cont);
motores.setSpeed(motores.leftMotorSpeed,motores.rightMotorSpeed);
Serial.print("  Sensor->");
Serial.print(sensorValue);
Serial.print("  Error->");
Serial.print(motores.error);
Serial.print("  Power Difference->");
Serial.print(motores.motorSpeed);
Serial.print("  Tiempo-->");
Serial.print(millis()-(motores.Time));
Serial.print("  Motor izquierdo->");
Serial.print(motores.leftMotorSpeed);
Serial.print("  Motor derecho->");
Serial.println(motores.rightMotorSpeed);


}


