#include <ZumoBuzzer.h>
#include <ZumoMotors.h>
#include <Pushbutton.h>
#include <QTRSensors.h>
#include <ZumoReflectanceSensorArray.h>
#include <avr/pgmspace.h>
#include <Wire.h>
#include <LSM303.h>
#include <QTRSensors.h>

ZumoBuzzer buzzer;
#define NUM_SENSORS 2
byte pins[] = {4, 5};
unsigned int sensor_values[NUM_SENSORS];
// this might need to be tuned for different lighting conditions, surfaces, etc.
#define QTR_THRESHOLD  1500 // microseconds
ZumoReflectanceSensorArray sensors(pins, 2, 2000, QTR_NO_EMITTER_PIN);
ZumoMotors motors;
Pushbutton button(ZUMO_BUTTON);
#define LED 13

int sensorpin = 5;
int i;// analog pin used to connect the sharp sensor
int val = 0;                 // variable to store the values from sensor(initially zero)
float distance = 0; 
float voltage = 0;

void setup()
{
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  Serial.begin(9600);               // starts the serial monitor
  motors.flipLeftMotor(true);
  motors.flipRightMotor(true);
  
  //buzzer.play(">g32>>c32");
  /*
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  pinMode(A4, INPUT);
  pinMode(A5, INPUT);
  digitalWrite(A0, LOW);
  digitalWrite(A1, LOW);
  digitalWrite(A2, LOW);
  digitalWrite(A3, LOW);
  digitalWrite(A4, LOW);
  digitalWrite(A5, LOW);
  */
  button.waitForButton();
  digitalWrite(13, LOW);

}
 
void loop()
{
    
  for (int i = 0; i<6; i++)
  {
    //sensorpin = i;
    val = analogRead(sensorpin);   // reads the value of the sharp sensor
    voltage = val*(5/1023.0);
    distance = 27.0570*pow(voltage,-1.1811);//pow(((val*(5/1023.0)*0.001221)/16.251),1.1765);
    Serial.print("Sensor ");
    Serial.print(sensorpin);
    Serial.print(" analog ");
    Serial.print(val);
    Serial.print(" voltage ");
    Serial.print(voltage);
    Serial.print(" distance ");
    Serial.println(distance);            // prints the value of the sensor to the serial monitor
    delay(1);                    // wait for this much time before printing next value
  }
}
