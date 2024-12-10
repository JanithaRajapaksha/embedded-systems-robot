#include <QTRSensors.h>
#include <Wire.h>
#include <Adafruit_VL6180X.h>
#include <ESP32Servo.h>

// QTR Sensors
QTRSensors qtr;
const uint8_t SensorCount = 5;
uint16_t sensorValues[SensorCount];

// Servo Motors
Servo servo1, servo2;
const int servoPin1 = 17;
const int servoPin2 = 16;
const int servoDefault1 = 90;
const int servoDefault2 = 120;

// Ultrasonic Sensor Pins
#define TRIG1_PIN 25
#define ECHO1_PIN 26
#define TRIG2_PIN 27
#define ECHO2_PIN 14

// IR Sensors
const int frontIR = 33;
const int IR = 22;

// Motor Configuration
struct MotorPins {
  int in1, in2, enable;
  int pwmChannel;
};
MotorPins motorA = {4, 0, 15, 0};
MotorPins motorB = {5, 18, 19, 1};
const int freq = 500;
const int resolution = 8;

// PID Parameters
const int Kp = 10;
const int Kd = 40;
const int BaseSpeed = 230;
const int MaxSpeed = 255;

// State Variables
bool isActionActiveLeft = false;
bool isBoxPicked = false;
unsigned long actionStartTime = 0;
const unsigned long actionDurationLeftTurn = 1000;
bool isTurning = false;
bool goStraight = false;
bool boxCompleted = false;

// Timing Constants
const unsigned long turnDelay = 100;
const unsigned long turnHoldTime = 100;
const unsigned long straightHoldTime = 100;
unsigned long previousMillis = 0;
unsigned long turnStartMillis = 0;
unsigned long straightStartMillis = 0;

// Target distance to maintain from the left wall
const int targetDistance = 100;  // in mm
int lastError = 0;
int lastErrorMaze = 0;

void setup() {
  Serial.begin(115200);
  setupServos();
  setupMotors();
  setupUltrasonicSensors();
  setupQTRSensor();
  setupIRSensors();
  calibrateQTRSensors();
}

void loop() {
  long distance1 = readUltrasonicDistance(TRIG1_PIN, ECHO1_PIN);
  long distance2 = readUltrasonicDistance(TRIG2_PIN, ECHO2_PIN);

  if (digitalRead(IR) == 0) {
    handleBoxPick();
  } else if (distance1 > 400) {
    lineFollowingLogic(distance1);
  } else if (boxCompleted) {
    mazeNavigationLogic(distance1, distance2);
  }
}

// Setup Functions
void setupServos() {
  servo1.setPeriodHertz(50);
  servo1.attach(servoPin1, 500, 2500);
  servo2.setPeriodHertz(50);
  servo2.attach(servoPin2, 500, 2500);
  resetServos();
}

void setupMotors() {
  setupMotorPins(motorA);
  setupMotorPins(motorB);
}

void setupMotorPins(MotorPins motor) {
  pinMode(motor.in1, OUTPUT);
  pinMode(motor.in2, OUTPUT);
  ledcSetup(motor.pwmChannel, freq, resolution);
  ledcAttachPin(motor.enable, motor.pwmChannel);
}

void setupUltrasonicSensors() {
  pinMode(TRIG1_PIN, OUTPUT);
  pinMode(ECHO1_PIN, INPUT);
  pinMode(TRIG2_PIN, OUTPUT);
  pinMode(ECHO2_PIN, INPUT);
}

void setupQTRSensor() {
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){36, 39, 34, 35, 32}, SensorCount);
}

void setupIRSensors() {
  pinMode(frontIR, INPUT);
  pinMode(IR, INPUT);
}

void calibrateQTRSensors() {
  for (int i = 0; i < 100; i++) {
    if (i < 25 || i >= 75) {
      moveMotor(motorA, 100, 1);
      moveMotor(motorB, 100, 0);
    } else {
      moveMotor(motorA, 100, 0);
      moveMotor(motorB, 100, 1);
    }
    qtr.calibrate();
    delay(20);
  }
}

void resetServos() {
  servo1.write(servoDefault1);
  servo2.write(servoDefault2);
}

// Movement Functions
void moveMotor(MotorPins motor, int speed, int direction) {
  digitalWrite(motor.in1, direction ? HIGH : LOW);
  digitalWrite(motor.in2, direction ? LOW : HIGH);
  ledcWrite(motor.pwmChannel, speed);
}

void moveMotors(int leftSpeed, int rightSpeed) {
  moveMotor(motorA, leftSpeed, leftSpeed > 0);
  moveMotor(motorB, rightSpeed, rightSpeed > 0);
}

void stopMotors() {
  moveMotor(motorA, 0, 0);
  moveMotor(motorB, 0, 0);
}

// Ultrasonic Functions
long readUltrasonicDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  return pulseIn(echoPin, HIGH) * 0.34 / 2;
}

// Behavior Functions
void handleBoxPick() {
  stopMotors();
  resetServos();
  // Simulate box picking actions
  servo2.write(150);
  delay(1000);
  servo1.write(180);
  delay(1000);
  servo2.write(120);
  delay(1000);
  servo1.write(90);
  delay(1000);
  boxCompleted = true;
}

void lineFollowingLogic(long distance1) {
  uint16_t position = qtr.readLineBlack(sensorValues);
  int error = position - 2000;
  int motorSpeed = Kp * error + Kd * (error - lastError);
  lastError = error;

  int rightSpeed = constrain(BaseSpeed + motorSpeed, 0, MaxSpeed);
  int leftSpeed = constrain(BaseSpeed - motorSpeed, 0, MaxSpeed);
  moveMotors(leftSpeed, rightSpeed);
}

void mazeNavigationLogic(long distance1, long distance2) {
  if (isTurning) {
    if (millis() - turnStartMillis >= turnHoldTime) {
      isTurning = false;
      goStraight = true;
      straightStartMillis = millis();
    }
    return;
  }

  if (goStraight) {
    if (millis() - straightStartMillis >= straightHoldTime) {
      goStraight = false;
    } else {
      moveMotors(BaseSpeed, BaseSpeed);
      return;
    }
  }

  if (distance1 > 300 || distance2 > 300) {
    turnStartMillis = millis();
    isTurning = true;
    if (distance1 > 300) {
      moveMotors(100, -100);  // Turn left
    } else {
      moveMotors(-100, 100);  // Turn right
    }
    return;
  }

  // Default PID control for maze navigation
  float error = targetDistance - distance1;
  int motorSpeed = Kp * error + Kd * (error - lastErrorMaze);
  lastErrorMaze = error;

  int rightSpeed = constrain(BaseSpeed + motorSpeed, 0, MaxSpeed);
  int leftSpeed = constrain(BaseSpeed - motorSpeed, 0, MaxSpeed);
  moveMotors(leftSpeed, rightSpeed);
}
