#include <ESP32Servo.h>

Servo servo1;  // create servo object to control a servo
Servo servo2;

int IR = 22;

// Motor A Pins
int AIN1 = 4;
int AIN2 = 0;
int enable1Pin = 15;

// Motor B Pins
int BIN1 = 5;
int BIN2 = 18;
int enable2Pin = 19;

const int speedturn = 100;

// Setting PWM properties
const int freq = 500;
const int pwmChannel1 = 0;
const int pwmChannel2 = 1;
const int resolution = 8;
int dutyCycle = 100;

void setup() {
  Serial.begin(115200); // Start Serial communication first

  // Allow allocation of all timers
  // ESP32PWM::allocateTimer(0);
  // ESP32PWM::allocateTimer(1);
  // ESP32PWM::allocateTimer(2);
  // ESP32PWM::allocateTimer(3);
  servo1.setPeriodHertz(50);            // Standard 50hz servo
  servo1.attach(servoPin1, 500, 2400);  // attaches the servo on pin 18 to the servo object
                                        // using SG90 servo min/max of 500us and 2400us
                                        // for MG995 large servo, use 1000us and 2000us,
                                        // which are the defaults, so this line could be
                                        // "myservo.attach(servoPin);"
  servo2.setPeriodHertz(50);            // Standard 50hz servo
  servo2.attach(servoPin2, 500, 2400);
  
  pinMode(IR, INPUT);
 
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

  Serial.println("Starting motor calibration..."); // Debug message
  delay(3000);

  for (int i = 0; i < 100; i++) {
    if (i < 25 || i >= 75) {
      move(0, speedturn, 1); // Motor A forward
      move(1, speedturn, 0); // Motor B backward
    } else {
      move(0, speedturn, 0); // Motor A backward
      move(1, speedturn, 1); // Motor B forward
    }
    delay(20);
  }

  Serial.println("Motor calibration complete."); // Debug message
}

void loop() {

  // for (int i = 0; i < 100; i++) {
  //   if (i < 25 || i >= 75) {
      // move(0, speedturn, 1); // Motor A forward
      // move(1, speedturn, 0); // Motor B backward
      // delay(2000);
    // } else {
      // move(0, speedturn, 0); // Motor A backward
      // move(1, speedturn, 1); // Motor B forward
      // delay(2000);
  //   }
  //   delay(20);
  // }

  // Move both motors forward at specified speed
  Serial.println("Moving Forward");
  move(0, dutyCycle, 1); // Motor A forward
  move(1, dutyCycle, 1); // Motor B forward

  uint8_t distanceIR = digitalRead(IR);

  if (distanceIR == 0) {

    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);

    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, LOW);
    Serial.println("Object detected");
    delay(1000);
    Serial.println("Moving servo1 to 180 degrees");
    servo1.write(180);

    delay(1000);
    // servo2.write(180);
    // delay(1000);

    delay(3000);
    servo1.write(75);
    delay(1000);

    // int j;
    // for (int j = 0; j < 250; j++) {
    //   move(0, speedturn, 1);
    //   move(1, speedturn, 0);
    //   delay(20);
    // }
  }


  // delay(2000);

  // // Stop both motors
  // Serial.println("Motors stopped");
  // move(0, 0, 1); // Motor A stop
  // move(1, 0, 1); // Motor B stop
  // delay(1000);

  // // Move both motors backward at specified speed
  // Serial.println("Moving Backwards");
  // move(0, dutyCycle, 0); // Motor A backward
  // move(1, dutyCycle, 0); // Motor B backward
  // delay(2000);

  // // Stop both motors
  // Serial.println("Motors stopped");
  // move(0, 0, 1); // Motor A stop
  // move(1, 0, 1); // Motor B stop
  // delay(1000);

  // Move both motors forward with increasing speed
  // dutyCycle = 0;
  // while (dutyCycle <= 255) {
  //   move(0, dutyCycle, 1); // Motor A forward with variable speed
  //   move(1, dutyCycle, 1); // Motor B forward with variable speed

  //   Serial.print("Forward with duty cycle: ");
  //   Serial.println(dutyCycle);

  //   dutyCycle += 5;
  //   delay(50);
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
