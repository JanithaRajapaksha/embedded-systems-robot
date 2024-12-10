#include <Arduino.h>
#include <Wire.h>
#include "Adafruit_VL6180X.h"


Adafruit_VL6180X vl = Adafruit_VL6180X();

// PID Constants
#define Kp 2
#define Kd 40
#define MaxSpeed 255
#define BaseSpeed 230
#define speedturn 100

//Motor A
int PWMB = 2; //for speed control
int BIN1 = 24; //Direction
int BIN2 = 22; //Direction
//Motor B
int PWMA = 3; //for speed control
int AIN1 = 26; //Direction
int AIN2 = 28; //Direction

// Ultrasonic sensor pins
#define TRIG1_PIN 6 // Left ultrasonic trigger pin
#define ECHO1_PIN 7   // Left ultrasonic echo pin
#define TRIG2_PIN 8  // Right ultrasonic trigger pin
#define ECHO2_PIN 9  // Right ultrasonic echo pin

// Target distance to maintain from the left wall
const int targetDistance = 80;  // in mm

unsigned long previousMillis = 0;      // Store the last time condition was met
unsigned long turnStartMillis = 0;     // Store the start time of the turn
unsigned long straightStartMillis = 0; // Store the start time of the straight movement
const unsigned long turnDelay = 100;   // Delay before initiating turn (ms)
const unsigned long turnHoldTime = 100; // Hold time for the turn (ms)
const unsigned long straightHoldTime = 100; // Hold time for straight movement (ms)
bool isTurning = false;                // Flag to track if the robot is currently turning
bool goStraight = false;               // Flag to track if the robot should go straight after turning

int lastError_maze = 0;

void setup() {
  // Serial.begin(115200);

  // if (!vl.begin()) {
  //   // Serial.println("Failed to find ToF sensor");
  //   while (1);
  // }

  // Initialize ultrasonic sensor pins
  pinMode(TRIG1_PIN, OUTPUT);
  pinMode(ECHO1_PIN, INPUT);
  pinMode(TRIG2_PIN, OUTPUT);
  pinMode(ECHO2_PIN, INPUT);

  // Initialize motor control pins
  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
}

void loop() {
  // Read distance from the left and right ultrasonic sensors
  long distance1 = readUltrasonicDistance(TRIG1_PIN, ECHO1_PIN);
  long distance2 = readUltrasonicDistance(TRIG2_PIN, ECHO2_PIN);

  // uint8_t distance3 = vl.readRange();
  // uint8_t status = vl.readRangeStatus();

  // if (status != VL6180X_ERROR_NONE) {
  //   distance3 = 1000;
  // }

  // Handle straight movement hold after turning
  if (goStraight) {
    if (millis() - straightStartMillis >= straightHoldTime) {
      goStraight = false;  // End the straight movement hold
    } else {
      move(1, BaseSpeed, 1);  // Move both motors forward
      move(0, BaseSpeed, 1);
      return;  // Continue going straight without executing PID control
    }
  }

  // Handle turning hold
  if (isTurning) {
    if (millis() - turnStartMillis >= turnHoldTime) {
      isTurning = false;  // End the turning action
      goStraight = true;  // Start going straight
      straightStartMillis = millis();  // Begin timing the straight hold
    }
    return;  // Exit to continue holding the turn
  }

  // Check if the distance conditions for initiating a turn are met
  if (distance1 > 300 || distance2 > 300) {
    unsigned long currentMillis = millis();
    
    // Check if turnDelay time has passed since last distance check
    if (currentMillis - previousMillis >= turnDelay) {
      previousMillis = currentMillis;  // Update the previousMillis time
      turnStartMillis = millis();  // Start timing the turn hold duration
      isTurning = true;  // Set turning flag to true

      // Execute turn based on which sensor triggered it
      if (distance1 > 300) {
        move(0, speedturn, 1);  // Turn left
        move(1, speedturn, 0);
      } else if (distance2 > 300) {
        move(0, speedturn, 0);  // Turn right
        move(1, speedturn, 1);
      }
      return;  // Exit loop to allow holding the turn
    }
  }

  // PID control continues as usual if no turn condition is met or turn has ended
  float error = targetDistance - distance1;
  int motorSpeed = Kp * error + Kd * (error - lastError_maze);
  lastError_maze = error;

  int rightMotorSpeed = BaseSpeed + motorSpeed;
  int leftMotorSpeed = BaseSpeed - motorSpeed;

  rightMotorSpeed = constrain(rightMotorSpeed, 0, MaxSpeed);
  leftMotorSpeed = constrain(leftMotorSpeed, 0, MaxSpeed);

  move(1, rightMotorSpeed, 1);
  move(0, leftMotorSpeed, 1);

  // delay(100);  // Small delay to stabilize readings
}

// Function to control motor speed and direction
void move(int motor, int speed, int direction) {
  bool inPin1 = (direction == 1) ? HIGH : LOW;
  bool inPin2 = (direction == 1) ? LOW : HIGH;

  if (motor == 0) {
    digitalWrite(AIN1, inPin1);
    digitalWrite(AIN2, inPin2);
    analogWrite(PWMA, speed);
  } else if (motor == 1) {
    digitalWrite(BIN1, inPin1);
    digitalWrite(BIN2, inPin2);
    analogWrite(PWMB, speed);
  }
}

// Function to measure distance from an ultrasonic sensor
long readUltrasonicDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH);
  long distance = duration * 0.34 / 2;  // Convert to centimeters
  return distance;
}
