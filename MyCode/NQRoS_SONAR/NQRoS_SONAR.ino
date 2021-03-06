#include <ZumoBuzzer.h>
#include <ZumoMotors.h>
#include <Pushbutton.h>
#include <QTRSensors.h>
#include <ZumoReflectanceSensorArray.h>
#include <avr/pgmspace.h>
#include <Wire.h>
#include <LSM303.h>

#define FLIPPED 1

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
#define QTR_THRESHOLD  1000 // microseconds
ZumoReflectanceSensorArray sensors(pins, 2, 2000, QTR_NO_EMITTER_PIN);

// Motor Settings
ZumoMotors motors;

// Timing
unsigned long loop_start_time = millis();
unsigned long last_line_time = millis();
unsigned long last_turn_time = millis();
unsigned long contact_made_time = millis();
#define MIN_DELAY_AFTER_TURN          400  // ms = min delay before detecting contact event
#define MIN_DELAY_BETWEEN_CONTACTS   1000  // ms = min delay between detecting new contact event

// these might need to be tuned for different motor types
#define CHASE_SPEED       100
#define REVERSE_SPEED     200 // 0 is stopped, 400 is full speed
#define TURN_SPEED        200
#define SEARCH_SPEED      200
#define SUSTAINED_SPEED   400 // switches to SUSTAINED_SPEED from FULL_SPEED after FULL_SPEED_DURATION_LIMIT ms
#define FULL_SPEED        400
#define STOP_DURATION     100 // ms
#define REVERSE_DURATION  200 // ms
#define TURN_DURATION     300 // ms
#define DIST_LIMIT        20  // cm
#define SPIN_SPEED        250 //

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
int lDist;
int rDist;
int lSpeed;
int rSpeed;
int offset = (analogRead(0) + analogRead(2)) / 2;
int baseline = 83;
int mostRecentTurn;
bool reversing;
int val = 0;                 // variable to store the values from sensor(initially zero)
float distance = 0;
float voltage = 0;

#define trig2 A2
#define echo2 A3
#define trig1 A0
#define echo1 11

void setup() {
  // put your setup code here, to run once:
  if (FLIPPED) {
    motors.flipLeftMotor(true);
    motors.flipRightMotor(true);
  }

  pinMode(LED, OUTPUT);
  digitalWrite(13, HIGH);
  Serial.begin(9600);
  //pinMode(A4, INPUT);
  //pinMode(A5, INPUT);
  //digitalWrite(A4, LOW);
  //digitalWrite(A5, LOW);
  pinMode(trig1, OUTPUT);
  pinMode(echo1, INPUT);
  digitalWrite(trig1, LOW);
  pinMode(trig2, OUTPUT);
  pinMode(echo2, INPUT);
  digitalWrite(trig2, LOW);

  // Play a little welcome song
  //buzzer.play(">g32>>c32");
  button.waitForButton();
  //randomSeed((unsigned int) millis());
  digitalWrite(13, LOW);
  // 5 second delay before starting
  delay(5000);
  loop_start_time = millis();
  last_line_time = millis();
}

void loop() {
  // put your main code here, to run repeatedly:


  int direction = LEFT;

  if (button.isPressed())
  {
    // if button is pressed, stop and wait for another press to go again
    motors.setSpeeds(0, 0);
    buzzer.play(">g32>>c32");
    button.waitForRelease();
  }

  sensors.read(sensor_values);

  // Does robot detect line on left sensor? If so, back up and turn right
  if (sensor_values[0] < QTR_THRESHOLD)
  {
    // note - if leftmost sensor detects line, reverse and turn to the right
    reverse(FULL_SPEED);
    mostRecentTurn = RIGHT;
    last_line_time = millis();
    loop_start_time = millis();
    reversing = true;
  }
  // Does robot detect line on right sensor? If so, back up and turn left
  else if (sensor_values[1] < QTR_THRESHOLD)
  {
    // note - if rightmost sensor detects line, reverse and turn to the left
    reverse(FULL_SPEED);
    mostRecentTurn = LEFT;
    last_line_time = millis();
    loop_start_time = millis();
    reversing = true;
  }
  // Start spinning if backed up enough.
  else if (((millis() - last_line_time) > 800) && (reversing == true)) {
    reversing = false;
    spin(mostRecentTurn);
    loop_start_time = millis();
  }
  // Cases for when the other robot is detected
  else {
    // Read sensors and convert to useful values
    lDist = measurement(trig1, echo1);
    rDist = measurement(trig2, echo2);

    // if both forward sensors detect, charge ahead
    if (lDist < DIST_LIMIT && rDist < DIST_LIMIT) {
      motors.setSpeeds(FULL_SPEED, FULL_SPEED);
      loop_start_time = millis();
    }
    // if left sensor only detects, turn left
    else if (lDist < DIST_LIMIT && rDist > DIST_LIMIT) {
      motors.setSpeeds(CHASE_SPEED, FULL_SPEED);
      mostRecentTurn = LEFT;
    }
    // if right sensor only detects, turn right
    else if (lDist > DIST_LIMIT && rDist < DIST_LIMIT) {
      motors.setSpeeds(FULL_SPEED, CHASE_SPEED);
      mostRecentTurn = RIGHT;
    }
    // if no line and no detection
    else {
      // if has been spinning for 5 seconds, go find the other robot
      if ((millis() - loop_start_time) > 5000) {
        motors.setSpeeds(FULL_SPEED, FULL_SPEED);
      }
      // if spinning for less than 5 seconds, search by spinning and scanning
      else {
        spin(mostRecentTurn);
      }
    }
  }
}

void spin(char direction) {
  motors.setSpeeds(SPIN_SPEED * direction, -SPIN_SPEED * direction);
}

void reverse(int speed) {
  motors.setSpeeds(-speed, -speed);
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
  motors.setSpeeds(0, 0);
  last_turn_time = millis();
}

long measurement(int trig, int echo) {
  long duration, distance;
  digitalWrite(trig, LOW);  // Added this line
  delayMicroseconds(2); // Added this line
  digitalWrite(trig, HIGH);
  delayMicroseconds(10); // Added this line
  digitalWrite(trig, LOW);
  duration = pulseIn(echo, HIGH);
  distance = (duration / 58);
  Serial.write("Distance: ");
  Serial.print(distance);
  Serial.println();
  return distance;
}

