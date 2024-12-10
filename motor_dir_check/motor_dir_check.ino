// Motor A connections
const int enA = 3;     // PWM pin for Motor A speed control
const int in1 = 26;     // Direction control pin for Motor A
const int in2 = 28;     // Direction control pin for Motor A

// Motor B connections
const int enB = 2;    // PWM pin for Motor B speed control
const int in3 = 24;     // Direction control pin for Motor B
const int in4 = 22;     // Direction control pin for Motor B

// Speed of the motors (0 to 255)
int motorSpeed = 200;

void setup() {
  // Set all the motor control pins as outputs
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  // Start with motors off
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}

void loop() {
  // Move both motors forward
  forward();
  delay(2000);   // Run for 2 seconds

  // Stop both motors
  stopMotors();
  delay(1000);   // Wait for 1 second

  // Move both motors backward
  backward();
  delay(2000);   // Run for 2 seconds

  // Stop both motors
  stopMotors();
  delay(1000);   // Wait for 1 second
}

void forward() {
  // Set Motor A forward
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  analogWrite(enA, motorSpeed);

  // Set Motor B forward
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  analogWrite(enB, motorSpeed);
}

void backward() {
  // Set Motor A backward
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  analogWrite(enA, motorSpeed);

  // Set Motor B backward
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  analogWrite(enB, motorSpeed);
}

void stopMotors() {
  // Stop Motor A
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  analogWrite(enA, 0);

  // Stop Motor B
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  analogWrite(enB, 0);
}
