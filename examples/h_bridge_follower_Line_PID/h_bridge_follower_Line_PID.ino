#include <QTRSensors.h>
#include <TB6612FNG.h>
#include <EEPROMex.h>
#include <EEPROMVar.h>

#define NUM_SENSORS  8     
#define TIMEOUT     2500 
#define EMITTER_PIN   QTR_NO_EMITTER_PIN    
#define PWM_FREQ PWM_FREQUENCY_MAX
#define Kp 1.45
#define Kd 16
#define Ki 0
#define setPoint 3500
#define BATTERY_PIN A7

QTRSensorsRC qtrrc((unsigned char[]) {  
  2, 3, 4, 5, 6, 7, 8, 9 } 
,NUM_SENSORS, TIMEOUT, EMITTER_PIN); 

TB6612FNG motores((byte[]) {
11, A0, A1, A3, A4, 10
},PWM_FREQ);

unsigned int sensorValues[NUM_SENSORS];

void setup(){
 //Serial.begin(9600);
 delay(500);
 pinMode(A2,OUTPUT);
 pinMode(13,OUTPUT);
 motores.setOutput(13,1);
 for (int i = 0; i < 250; i++) {
      qtrrc.calibrate();
    }
 guardarCalibracion();
 qtrrc.resetCalibration();
 restaurarCalibracion();
 //monitorSerial();
 motores.setOutput(13,0);
 motores.FollowerLinePID(setPoint,Kp,Kd,Ki);
delay(1000);
}
void loop(){
  int position = qtrrc.readLine(sensorValues);
  motores.ControlPID(position);
  motores.setOutput(16,HIGH);
  motores.setSpeed(motores.leftMotorSpeed,motores.rightMotorSpeed);


}
void guardarCalibracion(){

  for (int i = 0; i < NUM_SENSORS; i++)
  {                                                     //Gracias al uso de punteros por parte de la libreria se puede realizar lo siguiente
    EEPROM.writeInt(i*2,qtrrc.calibratedMinimumOn[i]);  //Guardar los valores en la EEPROM cada 2 slots gracias a la libreria 
    EEPROM.writeInt((i*2)+(NUM_SENSORS*2),qtrrc.calibratedMaximumOn[i]); //ya que en Arduino el int es de 2 Bytes y la EEMPROM almacena 1 byte por cada slot
  }

}
void monitorSerial(){
// Imprimir en pantalla el resultado
  Serial.println();
  Serial.print("Valores Minimos guardados en EEPROM-> ");
  for (int i = 0; i < NUM_SENSORS; i++){
    Serial.print(EEPROM.readInt(i*2));
    Serial.print(' ');
  }
  Serial.println(' ');
  Serial.print("Valores Maximos guardados en EEPROM-> ");

  for (int i = 0; i < NUM_SENSORS; i++){
    Serial.print(EEPROM.readInt((i*2)+(NUM_SENSORS*2)));
    Serial.print(' ');
  }

  Serial.println();
  Serial.println();

  Serial.print("Valores minimos de QTR-> ");
  for (int i = 0; i < NUM_SENSORS; i++){
    Serial.print(qtrrc.calibratedMinimumOn[i]);
    Serial.print(' ');
  }
  Serial.println(' ');
  Serial.print("Valores maximos de QTR-> ");

  for (int i = 0; i < NUM_SENSORS; i++){
    Serial.print(qtrrc.calibratedMaximumOn[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println();

}
void restaurarCalibracion(){  
  
  for (int i = 0; i < NUM_SENSORS; i++)
  {
    qtrrc.calibratedMinimumOn[i] = EEPROM.readInt(i*2); //Asignar los valores guardados en la EEPROM en los valores de calibracion
    qtrrc.calibratedMaximumOn[i] = EEPROM.readInt((i*2)+(NUM_SENSORS*2));
  }

}
float voltageBattery(){
 static int sensorValue = analogRead(BATTERY_PIN);
 static float voltage;
  voltage =(analogRead(A0))*(5.0/1023.0)*11.00; 
  return voltage;
}
