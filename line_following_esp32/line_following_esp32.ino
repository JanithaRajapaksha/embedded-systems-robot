#include <QTRSensors.h>
#include <Wire.h>
#include <Adafruit_VL6180X.h>
#include <ESP32Servo.h>

QTRSensors qtr;
Servo servo1;  // create servo object to control a servo
Servo servo2;
Adafruit_VL6180X vl = Adafruit_VL6180X();

const uint8_t SensorCount = 5;
uint16_t sensorValues[SensorCount];

bool isActionActive_left = false;
bool isBoxPicked = false;
bool isBoxPlaced = false;
bool isCompletedOneLeft = false;
unsigned long actionStartTime = 0;
const unsigned long actionDurationLeftTurn = 1500;

const int Kp = 10;
const int Kd = 40;
const int MaxSpeed = 255;
const int BaseSpeed = 230;
const int speedturn = 100;

// Motor A Pins
int AIN1 = 0;
int AIN2 = 4;
int enable1Pin = 15;

// Motor B Pins
int BIN1 = 18;
int BIN2 = 5;
int enable2Pin = 19;

#define TRIG1_PIN 25
#define ECHO1_PIN 26
#define TRIG2_PIN 27
#define ECHO2_PIN 14

int servoPin1 = 17;  // GPIO pin used to connect the servo control (digital out)
int servoPin2 = 16;

// Setting PWM properties
const int freq = 500;
const int pwmChannel1 = 0;
const int pwmChannel2 = 1;
const int resolution = 8;
int dutyCycle = 200;

int frontIR = 33;

int IR = 22;
int relay = 12;

// Target distance to maintain from the left wall
const int targetDistance = 100;  // in mm

unsigned long previousMillis = 0;            // Store the last time condition was met
unsigned long turnStartMillis = 0;           // Store the start time of the turn
unsigned long straightStartMillis = 0;       // Store the start time of the straight movement
const unsigned long turnDelay = 100;         // Delay before initiating turn (ms)
const unsigned long turnHoldTime = 100;      // Hold time for the turn (ms)
const unsigned long straightHoldTime = 100;  // Hold time for straight movement (ms)
bool isTurning = false;                      // Flag to track if the robot is currently turning
bool goStraight = false;                     // Flag to track if the robot should go straight after turning
bool boxPicked = false;


int lastError = 0;
int lastError_maze = 0;

void setup() {
  Serial.begin(115200);
  //Initialize VL6180X sensor
  if (!vl.begin()) {
    while (1)
      ;
  }
  // // Allow allocation of all timers
  // ESP32PWM::allocateTimer(0);
  // ESP32PWM::allocateTimer(1);
  // ESP32PWM::allocateTimer(2);
  // ESP32PWM::allocateTimer(3);
  servo1.setPeriodHertz(50);            // Standard 50hz servo
  servo1.attach(servoPin1, 500, 2500);  // attaches the servo on pin 18 to the servo object
                                        // using SG90 servo min/max of 500us and 2400us
                                        // for MG995 large servo, use 1000us and 2000us,
                                        // which are the defaults, so this line could be
                                        // "myservo.attach(servoPin);"
  servo2.setPeriodHertz(50);            // Standard 50hz servo
  servo2.attach(servoPin2, 500, 2500);

  // Sets motor pins as outputs
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(enable1Pin, OUTPUT);

  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(enable2Pin, OUTPUT);

  // Configure PWM functionalities
  ledcSetup(pwmChannel1, freq, resolution);
  ledcSetup(pwmChannel2, freq, resolution);

  // Attach PWM channels to the GPIOs
  ledcAttachPin(enable1Pin, pwmChannel1);
  ledcAttachPin(enable2Pin, pwmChannel2);

  // Initialize ultrasonic sensor pins
  pinMode(TRIG1_PIN, OUTPUT);
  pinMode(ECHO1_PIN, INPUT);
  pinMode(TRIG2_PIN, OUTPUT);
  pinMode(ECHO2_PIN, INPUT);

  pinMode(relay, OUTPUT);

  // Initialize QTR sensor
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){ 36, 39, 34, 35, 32 }, SensorCount);

  digitalWrite(relay, HIGH);
  delay(500);
  servo1.write(75);
  servo2.write(120);
  delay(500);
  digitalWrite(relay, LOW);

  pinMode(frontIR, INPUT);
  pinMode(IR, INPUT);
  // delay(500);
  // pinMode(LED_BUILTIN, OUTPUT);
  // digitalWrite(LED_BUILTIN, HIGH);
  // delay(3000);

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
  // delay(3000);
}

void loop() {

  // long distance1 = readUltrasonicDistance(TRIG1_PIN, ECHO1_PIN);
  long distance2 = readUltrasonicDistance(TRIG2_PIN, ECHO2_PIN);
  uint8_t distance_tof = vl.readRange();

  uint16_t position = qtr.readLineBlack(sensorValues);
  uint16_t frontIRStatus = analogRead(frontIR);

  if (boxPicked && !isBoxPlaced && isCompletedOneLeft && (sensorValues[0] < 500 && sensorValues[1] > 500 && sensorValues[2] > 500 && sensorValues[3] > 500 && sensorValues[4] < 500 && frontIRStatus > 500)) {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);

    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, LOW);

    delay(1000);

    int k;
    for (int k = 0; k < 50; k++) {
      move(0, speedturn, 0);
      move(1, speedturn, 0);
      delay(20);
    }

    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);

    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, LOW);

    delay(1000);

    digitalWrite(relay, HIGH);
    delay(1000);
    servo2.write(210);
    delay(1000);
    servo1.write(75);
    delay(1000);
    servo2.write(90);
    delay(1000);

    digitalWrite(relay, LOW);

    int r;
    for (int r = 0; r < 50; r++) {
      move(0, speedturn, 0);
      move(1, speedturn, 0);
      delay(20);
    }

    isBoxPlaced = true;

    int j;
    for (int j = 0; j < 250; j++) {
      move(0, speedturn, 1);
      move(1, speedturn, 0);
      delay(20);
    }
  }

  if (distance_tof < 45 && !boxPicked) {

    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);

    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, LOW);
    Serial.println("Object detected");

    digitalWrite(relay, HIGH);
    delay(1000);
    Serial.println("Moving servo1 to 180 degrees");

    // servo1.write(180);
    // delay(1000);
    // servo1.write(90);
    // delay(1000);

    servo2.write(210);
    delay(1000);
    servo1.write(75);
    delay(1000);

    servo1.write(180);
    delay(1000);
    servo2.write(90);
    delay(1000);
    digitalWrite(relay, LOW);

    boxPicked = true;

    int j;
    for (int j = 0; j < 250; j++) {
      move(0, speedturn, 1);
      move(1, speedturn, 0);
      delay(20);
    }
  }

  if (distance2 > 300) {




  // // uint8_t distance_tof = vl.readRange();
  // uint8_t distanceIR = digitalRead(IR);
  // Serial.print("Range: ");
  // Serial.println(distanceIR);


  // if (position > 2900 && frontIRStatus < 300) {
  //   move(1, speedturn, 1);
  //   move(0, speedturn, 0);
  //   return;
  // }
  // if (position < 1100) {
  //   move(1, speedturn, 0);
  //   move(0, speedturn, 1);
  //   return;
  // }
  if (sensorValues[0] > 500 && sensorValues[1] > 500 && sensorValues[2] > 500 && sensorValues[3] > 500 && sensorValues[4] > 500) {  // Start the action
    if (boxPicked && !isBoxPlaced) {
      isCompletedOneLeft = true;
    }
    actionStartTime = millis();  // Record the start time
    isActionActive_left = true;  // Set the action as active
    return;
  }
  if (sensorValues[0] > 500 && sensorValues[1] > 500 && sensorValues[2] > 500 && sensorValues[3] < 300 && sensorValues[4] < 300) {  // Start the action
    actionStartTime = millis();                                                                                                     // Record the start time
    isActionActive_left = true;                                                                                                     // Set the action as active
    return;
  }
  if (isActionActive_left) {
    // Check if the action duration has elapsed
    if (millis() - actionStartTime >= actionDurationLeftTurn) {
      isActionActive_left = false;  // End the action after the duration
      // Optional: Add code to stop motors or reset state if necessary
      return;
    }
    // Perform the action during this time
    move(1, speedturn, 0);  // Example of an action
    move(0, speedturn, 1);
    return;
  }

  int error = position - 2000;
  int motorSpeed = Kp * error + Kd * (error - lastError);
  lastError = error;

  int rightMotorSpeed = BaseSpeed + motorSpeed;
  int leftMotorSpeed = BaseSpeed - motorSpeed;

  if (rightMotorSpeed > MaxSpeed) rightMotorSpeed = MaxSpeed;
  if (leftMotorSpeed > MaxSpeed) leftMotorSpeed = MaxSpeed;
  if (rightMotorSpeed < 0) rightMotorSpeed = 0;
  if (leftMotorSpeed < 0) leftMotorSpeed = 0;

  move(1, rightMotorSpeed, 1);
  move(0, leftMotorSpeed, 1);

  } 
  // else {
  //   // servo1.write(90);
  //   // servo2.write(120);

  //   // move(1, 100, 1);
  //   // move(0, 100, 1);
  //   // delay(1000);

  //   // Handle straight movement hold after turning
  //   if (goStraight) {
  //     if (millis() - straightStartMillis >= straightHoldTime) {
  //       goStraight = false;  // End the straight movement hold
  //     } else {
  //       move(1, BaseSpeed, 1);  // Move both motors forward
  //       move(0, BaseSpeed, 1);
  //       return;  // Continue going straight without executing PID control
  //     }
  //   }

  //   // Handle turning hold
  //   if (isTurning) {
  //     if (millis() - turnStartMillis >= turnHoldTime) {
  //       isTurning = false;               // End the turning action
  //       goStraight = true;               // Start going straight
  //       straightStartMillis = millis();  // Begin timing the straight hold
  //     }
  //     return;  // Exit to continue holding the turn
  //   }

  //   // Check if the distance conditions for initiating a turn are met
  //   if (distance1 > 300 || distance2 > 300) {
  //     unsigned long currentMillis = millis();

  //     // Check if turnDelay time has passed since last distance check
  //     if (currentMillis - previousMillis >= turnDelay) {
  //       previousMillis = currentMillis;  // Update the previousMillis time
  //       turnStartMillis = millis();      // Start timing the turn hold duration
  //       isTurning = true;                // Set turning flag to true

  //       // Execute turn based on which sensor triggered it
  //       if (distance1 > 300) {
  //         move(0, speedturn, 1);  // Turn left
  //         move(1, speedturn, 0);
  //       } else if (distance2 > 300) {
  //         move(0, speedturn, 0);  // Turn right
  //         move(1, speedturn, 1);
  //       }
  //       return;  // Exit loop to allow holding the turn
  //     }
  //   }

  //   // PID control continues as usual if no turn condition is met or turn has ended
  //   float error = targetDistance - distance1;
  //   int motorSpeed = Kp * error + Kd * (error - lastError_maze);
  //   lastError_maze = error;

  //   int rightMotorSpeed = BaseSpeed + motorSpeed;
  //   int leftMotorSpeed = BaseSpeed - motorSpeed;

  //   rightMotorSpeed = constrain(rightMotorSpeed, 0, MaxSpeed);
  //   leftMotorSpeed = constrain(leftMotorSpeed, 0, MaxSpeed);

  //   move(1, rightMotorSpeed, 1);
  //   move(0, leftMotorSpeed, 1);

  //   delay(100);  // Small delay to stabilize readings
  // }
}


void move(int motor, int speed, int direction) {
  boolean inPin1 = HIGH, inPin2 = LOW;
  if (direction == 1) {
    inPin1 = HIGH;
    inPin2 = LOW;
  } else {
    inPin1 = LOW;
    inPin2 = HIGH;
  }

  if (motor == 0) {
    ledcWrite(pwmChannel1, speed);
    digitalWrite(AIN1, inPin1);
    digitalWrite(AIN2, inPin2);

  } else if (motor == 1) {
    ledcWrite(pwmChannel2, speed);
    digitalWrite(BIN1, inPin1);
    digitalWrite(BIN2, inPin2);
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
