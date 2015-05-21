#include <ZumoBuzzer.h>
#include <ZumoMotors.h>
#include <Pushbutton.h>
#include <QTRSensors.h>
#include <ZumoReflectanceSensorArray.h>
#include <avr/pgmspace.h>
#include <Wire.h>
#include <LSM303.h>

/*  NQRoS Sumo Bot code
    QUT Robotics Club
    Lachlan Robinson
    Natasha Moffat
    Ben Graham  
    uses some code from SumoCollisionDetect example
*/

#define LED 13
Pushbutton button(ZUMO_BUTTON); // pushbutton on pin 12
// Reflectance Sensor Settings
#define NUM_SENSORS 2
byte pins[] = {4, 5};
unsigned int sensor_values[NUM_SENSORS];
// this might need to be tuned for different lighting conditions, surfaces, etc.
#define QTR_THRESHOLD  1500 // microseconds
ZumoReflectanceSensorArray sensors(pins, 2, 2000, QTR_NO_EMITTER_PIN); 

// Motor Settings
ZumoMotors motors;

 // Timing
unsigned long loop_start_time = 0;
unsigned long last_turn_time;
unsigned long contact_made_time;
#define MIN_DELAY_AFTER_TURN          400  // ms = min delay before detecting contact event
#define MIN_DELAY_BETWEEN_CONTACTS   1000  // ms = min delay between detecting new contact event

// these might need to be tuned for different motor types
#define REVERSE_SPEED     200 // 0 is stopped, 400 is full speed
#define TURN_SPEED        200
#define SEARCH_SPEED      200
#define SUSTAINED_SPEED   400 // switches to SUSTAINED_SPEED from FULL_SPEED after FULL_SPEED_DURATION_LIMIT ms
#define FULL_SPEED        400
#define STOP_DURATION     100 // ms
#define REVERSE_DURATION  200 // ms
#define TURN_DURATION     300 // ms
#define DIST_LIMIT        30  // cm
#define SPIN_SPEED        300 //

#define RIGHT 1
#define LEFT -1

// Sound Effects
ZumoBuzzer buzzer;
const char sound_effect[] PROGMEM = "O4 T100 V15 L4 MS g12>c12>e12>G6>E12 ML>G2"; // "charge" melody
 // use V0 to suppress sound effect; v15 for max volume
 
int lVal;
int rVal;
float lVoltage;
float rVoltage;
float lDist;
float rDist;

void setup() {
  // put your setup code here, to run once:
  //motors.flipLeftMotor(true);
  //motors.flipRightMotor(true);
  
  pinMode(LED, OUTPUT);
  digitalWrite(13, HIGH);
  //Serial.begin(9600);    
  //pinMode(A4, INPUT);
  //pinMode(A5, INPUT);
  //digitalWrite(A4, LOW);
  //digitalWrite(A5, LOW);
  
  randomSeed((unsigned int) millis());

  // Play a little welcome song
  //buzzer.play(">g32>>c32");
  button.waitForButton();
  digitalWrite(13, LOW);
  // 5 second delay before starting
  delay(5000);
}

void loop() {
  // put your main code here, to run repeatedly:
  
  int mostRecentTurn = RIGHT;
  int direction = LEFT;
  
  if (button.isPressed()){
    // if button is pressed, stop and wait for another press to go again
    motors.setSpeeds(0, 0);
    buzzer.play(">g32>>c32");
    button.waitForRelease();
  }
  
  sensors.read(sensor_values);

  // Does robot detect line?
  if (sensor_values[0] < QTR_THRESHOLD){
    // note - if leftmost sensor detects line, reverse and turn to the right
    turn(RIGHT, false);
    mostRecentTurn = RIGHT;
  }
  else if (sensor_values[1] < QTR_THRESHOLD){
    // note - if rightmost sensor detects line, reverse and turn to the left
    turn(LEFT, false);
    mostRecentTurn = LEFT;
  }
  // If robot does not detect line, do normal behaviour - seek and destroy!
  else{
    lVal = analogRead(0);   // reads the value of the sharp sensor
    rVal = analogRead(1);
    lVoltage = lVal*(5/1023.0);
    rVoltage = rVal*(5/1023.0);
    lDist = 27.0570*pow(lVoltage,-1.1811);//pow(((val*(5/1023.0)*0.001221)/16.251),1.1765);
    rDist = 27.0570*pow(rVoltage,-1.1811);
      
    if (lDist<DIST_LIMIT && rDist<DIST_LIMIT){
      motors.setSpeeds(FULL_SPEED, FULL_SPEED);
      loop_start_time = millis();
    }
    else if (lDist<DIST_LIMIT && rDist>DIST_LIMIT){
      motors.setSpeeds(SEARCH_SPEED, FULL_SPEED);
    }
    else if (lDist>DIST_LIMIT && rDist<DIST_LIMIT){
      motors.setSpeeds(FULL_SPEED, SEARCH_SPEED);
    }
    else {
      if ((millis() - loop_start_time)>5000) {
        motors.setSpeeds(FULL_SPEED, FULL_SPEED);
      }
      else {
        if (mostRecentTurn = LEFT) {
          direction = LEFT;
        }
        else {
          direction = RIGHT;
        }
        motors.setSpeeds(SPIN_SPEED * direction, -SPIN_SPEED * direction);
      }
    }
  }
}

void turn(char direction, bool randomize)
{
  
  static unsigned int duration_increment = TURN_DURATION / 4;
  
  // motors.setSpeeds(0,0);
  // delay(STOP_DURATION);
      
  motors.setSpeeds(-FULL_SPEED, -FULL_SPEED);
  delay(REVERSE_DURATION);
  motors.setSpeeds(FULL_SPEED * direction, -FULL_SPEED * direction);
  delay(randomize ? TURN_DURATION + (random(8) - 2) * duration_increment : TURN_DURATION);
  motors.setSpeeds(0,0);
  last_turn_time = millis();
}

