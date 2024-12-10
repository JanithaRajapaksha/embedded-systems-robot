#include <QTRSensors.h>
#include <Wire.h>
#include <Adafruit_VL6180X.h>
#include <Servo.h>

QTRSensors qtr;
Adafruit_VL6180X vl = Adafruit_VL6180X();

Servo servo1;  // Create servo object for servo 1
Servo servo2;  // Create servo object for servo 2

const uint8_t SensorCount = 5;
uint16_t sensorValues[SensorCount];

unsigned long actionStartTime = 0;
const unsigned long actionDuration = 2000;  // Duration for which to perform the action (in milliseconds)
bool isActionActive = false;
bool isActionActive_left = false;

bool isBoxPicked = false;

unsigned long rotationStartTime = 0;  // Stores the time when the rotation starts
const unsigned long rotationDuration = 1000;  // Rotation duration in milliseconds
bool isRotating = false;  // Tracks whether the robot is currently rotating

#define Kp 2
#define Kd 40
#define MaxSpeed 255
#define BaseSpeed 230
#define speedturn 100

//int STBY = 10;
//Motor A
int PWMB = 2;   //for speed control
int BIN1 = 24;  //Direction
int BIN2 = 22;  //Direction
//Motor B
int PWMA = 3;   //for speed control
int AIN1 = 26;  //Direction
int AIN2 = 28;  //Direction

// Ultrasonic sensor pins
#define TRIG1_PIN 6  // Left ultrasonic trigger pin
#define ECHO1_PIN 7  // Left ultrasonic echo pin
#define TRIG2_PIN 8  // Right ultrasonic trigger pin
#define ECHO2_PIN 9  // Right ultrasonic echo pin

int frontIR = A5;

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


int lastError = 0;
int lastError_maze = 0;

void setup() {

  // Initialize VL6180X sensor
  if (!vl.begin()) {
    Serial.println("Failed to find VL6180X sensor");
    while (1)
      ;
  }

  // Initialize ultrasonic sensor pins
  pinMode(TRIG1_PIN, OUTPUT);
  pinMode(ECHO1_PIN, INPUT);
  pinMode(TRIG2_PIN, OUTPUT);
  pinMode(ECHO2_PIN, INPUT);
  Serial.println("VL6180X sensor ready!");

  servo1.attach(4);   // Attach servo1 to pin 9
  servo2.attach(5);   // Attach servo2 to pin 10
  servo1.write(90);   // Set initial position for servo1
  servo2.write(180);  // Set initial position for servo2

  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){ A0, A1, A2, A3, A4 }, SensorCount);
  //qtr.setEmitterPin(2);pinMode(STBY, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(frontIR, INPUT);
  delay(500);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(3000);
  int i;
  for (int i = 0; i < 250; i++) {
    if (i < 25 || i >= 75) {
      move(0, 150, 1);
      move(1, 150, 0);
    } else {
      move(0, 150, 0);
      move(1, 150, 1);
    }
    qtr.calibrate();
    delay(20);
  }
  delay(3000);
}

void loop() {

  long distance1 = readUltrasonicDistance(TRIG1_PIN, ECHO1_PIN);
  long distance2 = readUltrasonicDistance(TRIG2_PIN, ECHO2_PIN);

  uint16_t position = qtr.readLineBlack(sensorValues);
  uint16_t frontIRStatus = analogRead(frontIR);

  uint8_t distance_tof = vl.readRange();  // Read distance from VL6180X sensor

  // Check for errors from VL6180X
  if (vl.readRangeStatus() != VL6180X_ERROR_NONE) {
    Serial.println("VL6180X range error");
  }


  // if(position<300 && distance1 > 300)
  //   {   move(1, speedturn, 0);
  //       move(0, speedturn, 1);
  //   return;
  //   }

  // if (sensorValues[0] < 300 && sensorValues[1] < 300 && sensorValues[2] > 800 && sensorValues[3] > 800 && sensorValues[4] > 800 && frontIRStatus < 300) {
  //   actionStartTime = millis();  // Record the start time
  //   isActionActive = true;       // Set the action as active
  //   return;
  // }
  // if (isActionActive) {
  //   // Check if the action duration has elapsed
  //   if (millis() - actionStartTime >= actionDuration) {
  //     isActionActive = false;  // End the action after the duration
  //     // Optional: Add code to stop motors or reset state if necessary
  //     return;
  //   }
  //   // Perform the action during this time
  //   move(1, speedturn, 1);  // Example of an action
  //   move(0, speedturn, 0);
  //   return;
  // }
  if (sensorValues[0] > 800 && sensorValues[1] > 800 && sensorValues[2] > 800 && frontIRStatus > 800) {  // Start the action
    actionStartTime = millis();                                                                          // Record the start time
    isActionActive_left = true;                                                                          // Set the action as active
    return;
  }

  if (isActionActive_left) {
    // Check if the action duration has elapsed
    if (millis() - actionStartTime >= actionDuration) {
      isActionActive_left = false;  // End the action after the duration
      // Optional: Add code to stop motors or reset state if necessary
      return;
    }
    // Perform the action during this time
    move(1, speedturn, 0);  // Example of an action
    move(0, speedturn, 1);
    return;
  }

  if (sensorValues[0] < 300 && sensorValues[1] < 300 && sensorValues[2] < 300 && sensorValues[3] < 300 && sensorValues[4] < 300 && frontIRStatus < 300) {
   actionStartTime = millis();                                                                          // Record the start time
    isActionActive_left = true;                                                                          // Set the action as active
    return;
  }

  if (sensorValues[0] > 800 && sensorValues[1] > 800 && sensorValues[2] > 800 && sensorValues[3] < 300 && sensorValues[4] < 300 && frontIRStatus > 800) {  // Start the action
    actionStartTime = millis();                                                                          // Record the start time
    isActionActive_left = true;                                                                          // Set the action as active
    return;
  }

  if (sensorValues[0] > 800 && sensorValues[1] > 800 && sensorValues[2] > 800 && sensorValues[3] < 300 && sensorValues[4] < 300 && frontIRStatus < 300) {  // Start the action
    actionStartTime = millis();                                                                          // Record the start time
    isActionActive_left = true;                                                                          // Set the action as active
    return;
  }

  if (sensorValues[0] > 800 && sensorValues[1] > 800 && sensorValues[2] > 800 && sensorValues[3] > 800 && sensorValues[4] > 800 && frontIRStatus < 400) {  // Start the action
    actionStartTime = millis();                                                                                                                            // Record the start time
    isActionActive_left = true;                                                                                                                            // Set the action as active
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

  if (distance_tof < 80) {
  servo1.write(180); 
    move(1, 0, 1);
    move(0, 0, 1);
    delay(2000);

     // Move servo1 to 90 degrees

    isBoxPicked = true;

  int j;
  for (int j = 0; j < 250; j++) {
       move(0, speedturn, 1);
      move(1, speedturn, 0);
    delay(20);
  }

  if (isBoxPicked == true && sensorValues[0] < 300 && sensorValues[1] > 800 && sensorValues[2] > 800 && sensorValues[3] > 800 && sensorValues[4] < 300 && frontIRStatus >800){}
      servo1.write(75);
      isBoxPicked == false;
  }
  

  if (sensorValues[0] < 300 && sensorValues[1] < 300 && sensorValues[2] < 300 && sensorValues[3] < 300 && sensorValues[4] < 300 && frontIRStatus < 300 && distance1 < 200) {
    
    // move(1, 0, 1);
    // move(0, 0, 1);
    // delay(2000);

    // Handle straight movement hold after turning
    if (goStraight) {
      if (millis() - straightStartMillis >= straightHoldTime) {
        goStraight = false;  // End the straight movement hold
      } else {
        move(1, BaseSpeed, 1);  // Move both motors forward
        move(0, BaseSpeed, 1);
        // return;  // Continue going straight without executing PID control
      }
    }

    // Handle turning hold
    if (isTurning) {
      if (millis() - turnStartMillis >= turnHoldTime) {
        isTurning = false;               // End the turning action
        goStraight = true;               // Start going straight
        straightStartMillis = millis();  // Begin timing the straight hold
      }
      // return;  // Exit to continue holding the turn
    }

    // Check if the distance conditions for initiating a turn are met
    if (distance1 > 300 || distance2 > 300) {
      unsigned long currentMillis = millis();

      // Check if turnDelay time has passed since last distance check
      if (currentMillis - previousMillis >= turnDelay) {
        previousMillis = currentMillis;  // Update the previousMillis time
        turnStartMillis = millis();      // Start timing the turn hold duration
        isTurning = true;                // Set turning flag to true

        // Execute turn based on which sensor triggered it
        if (distance1 > 300) {
          move(0, speedturn, 1);  // Turn left
          move(1, speedturn, 0);
        } else if (distance2 > 300) {
          move(0, speedturn, 0);  // Turn right
          move(1, speedturn, 1);
        }
        // return;  // Exit loop to allow holding the turn
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



void move(int motor, int speed, int direction) {
  boolean inPin1 = HIGH, inPin2 = LOW;
  if (direction == 1) {
    inPin1 = HIGH;
    inPin2 = LOW;
  }
  if (direction == 0) {
    inPin1 = LOW;
    inPin2 = HIGH;
  }
  if (motor == 0) {
    digitalWrite(AIN1, inPin1);
    digitalWrite(AIN2, inPin2);
    analogWrite(PWMA, speed);
  }
  if (motor == 1) {
    digitalWrite(BIN1, inPin1);
    digitalWrite(BIN2, inPin2);
    analogWrite(PWMB, speed);
  }
}