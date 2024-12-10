#include <QTRSensors.h>
#include <Wire.h>
#include <Adafruit_VL6180X.h>
#include <ESP32Servo.h>

// Components
QTRSensors qtr;
Servo servo1;  // Servo 1 object
Adafruit_VL6180X vl = Adafruit_VL6180X();

// Constants and Variables
const uint8_t SensorCount = 5;
uint16_t sensorValues[SensorCount];

bool isActionActive_left = false;
unsigned long actionStartTime = 0;
const unsigned long actionDurationLeftTurn = 1000;

const int Kp = 10;
const int Kd = 40;
const int MaxSpeed = 255;
const int BaseSpeed = 230;
const int speedturn = 100;

// Motor A Pins
const int AIN1 = 4;
const int AIN2 = 0;
const int enable1Pin = 15;

// Motor B Pins
const int BIN1 = 5;
const int BIN2 = 18;
const int enable2Pin = 19;

// Ultrasonic Sensor Pins
#define TRIG1_PIN 25
#define ECHO1_PIN 26
#define TRIG2_PIN 27
#define ECHO2_PIN 14

const int servoPin1 = 17;  // Servo pin
const int frontIR = 33;    // IR sensor pin

// PID Variables
int lastError = 0;
int lastError_maze = 0;

// Timing variables for ultrasonic sensor interrupts
volatile long echoStart1 = 0, echoEnd1 = 0;
volatile bool echoReceived1 = false;

volatile long echoStart2 = 0, echoEnd2 = 0;
volatile bool echoReceived2 = false;

volatile long echoStart1 = 0, echoEnd1 = 0;
volatile long echoStart2 = 0, echoEnd2 = 0;
volatile bool echoReceived1 = false, echoReceived2 = false;

void handleEcho1() {
  if (digitalRead(ECHO1_PIN) == HIGH) {
    echoStart1 = micros();  // Capture rising edge
  } else {
    echoEnd1 = micros();    // Capture falling edge
    echoReceived1 = true;   // Indicate that a pulse was received
  }
}

void handleEcho2() {
  if (digitalRead(ECHO2_PIN) == HIGH) {
    echoStart2 = micros();
  } else {
    echoEnd2 = micros();
    echoReceived2 = true;
  }
}

// Setup function
void setup() {
  Serial.begin(115200);

  // Initialize servo
  ESP32PWM::allocateTimer(0);
  servo1.setPeriodHertz(50);
  servo1.attach(servoPin1, 900, 2100);

  // Initialize motor pins
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(enable1Pin, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(enable2Pin, OUTPUT);

  // Configure PWM for motor control
  ledcSetup(0, 500, 8);
  ledcSetup(1, 500, 8);
  ledcAttachPin(enable1Pin, 0);
  ledcAttachPin(enable2Pin, 1);

  // Initialize ultrasonic sensor pins
  pinMode(TRIG1_PIN, OUTPUT);
  pinMode(ECHO1_PIN, INPUT);
  pinMode(TRIG2_PIN, OUTPUT);
  pinMode(ECHO2_PIN, INPUT);

  // Attach interrupts for ultrasonic sensors
  attachInterrupt(digitalPinToInterrupt(ECHO1_PIN), handleEcho1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ECHO2_PIN), handleEcho2, CHANGE);

  // Initialize QTR sensors
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){ 36, 39, 34, 35, 32 }, SensorCount);

  // Calibrate QTR sensors
  for (int i = 0; i < 100; i++) {
    if (i < 25 || i >= 75) {
      move(0, speedturn, 1);
      move(1, speedturn, 0);
    } else {
      move(0, speedturn, 0);
      move(1, speedturn, 1);
    }
    qtr.calibrate();
    delay(20);
  }
}

// Loop function
void loop() {
  long distance1 = getUltrasonicDistance(echoStart1, echoEnd1, echoReceived1);
  long distance2 = getUltrasonicDistance(echoStart2, echoEnd2, echoReceived2);

  if (distance1 > 400) {
    uint16_t position = qtr.readLineBlack(sensorValues);
    int error = position - 2000;
    int motorSpeed = Kp * error + Kd * (error - lastError);
    lastError = error;

    int rightMotorSpeed = BaseSpeed + motorSpeed;
    int leftMotorSpeed = BaseSpeed - motorSpeed;

    rightMotorSpeed = constrain(rightMotorSpeed, 0, MaxSpeed);
    leftMotorSpeed = constrain(leftMotorSpeed, 0, MaxSpeed);

    move(1, rightMotorSpeed, 1);
    move(0, leftMotorSpeed, 1);
    if (distance2 <= 100) {
      stopMotors();
      servo1.write(120);
      delay(1000);
      servo1.write(90);
      delay(1000);
    }
  } else {
    // Handle default movement (e.g., wall following)
    float error = 100 - distance1;  // Target distance from the wall
    int motorSpeed = Kp * error + Kd * (error - lastError_maze);
    lastError_maze = error;

    int rightMotorSpeed = BaseSpeed + motorSpeed;
    int leftMotorSpeed = BaseSpeed - motorSpeed;

    move(1, rightMotorSpeed, 1);
    move(0, leftMotorSpeed, 1);
  }
}

// Motor control function
void move(int motor, int speed, int direction) {
  bool inPin1 = (direction == 1);
  bool inPin2 = !inPin1;

  if (motor == 0) {  // Left motor
    ledcWrite(0, speed);
    digitalWrite(AIN1, inPin1);
    digitalWrite(AIN2, inPin2);
  } else if (motor == 1) {  // Right motor
    ledcWrite(1, speed);
    digitalWrite(BIN1, inPin1);
    digitalWrite(BIN2, inPin2);
  }
}

// Stop motors
void stopMotors() {
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
}

// Ultrasonic sensor interrupt handlers
void IRAM_ATTR handleEcho1() {
  if (digitalRead(ECHO1_PIN)) {
    echoStart1 = micros();
  } else {
    echoEnd1 = micros();
    echoReceived1 = true;
  }
}

void IRAM_ATTR handleEcho2() {
  if (digitalRead(ECHO2_PIN)) {
    echoStart2 = micros();
  } else {
    echoEnd2 = micros();
    echoReceived2 = true;
  }
}

// Calculate ultrasonic distance
long getUltrasonicDistance(long &start, long &end, volatile bool &received) {
  if (received) {
    received = false;
    return (end - start) * 0.34 / 2;  // Distance in cm
  }
  return -1;  // No valid reading
}
