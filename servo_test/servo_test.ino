#include <Servo.h>

Servo myServo; // Create a Servo object

int pos = 0; // Variable to store the servo position

void setup() {
  myServo.attach(4); // Attach the servo on pin 9
}

void loop() {
  myServo.write(90);
  // Rotate from 0 to 180 degrees
  // for (pos = 75; pos <= 150; pos += 30) { // Increment position by 1 degree
  //   myServo.write(pos); // Move servo to the current position
  //   delay(200); // Wait 15 milliseconds for the servo to reach the position
  // }

  // // Rotate back from 180 to 0 degrees
  // for (pos = 150; pos >= 75; pos -= 30) { // Decrement position by 1 degree
  //   myServo.write(pos); // Move servo to the current position
  //   delay(200); // Wait 15 milliseconds for the servo to reach the position
  // }
  // Rotate from 0 to 180 degrees
  // for (pos = 0; pos <= 120; pos += 15) { // Increment position by 1 degree
  //   myServo.write(pos); // Move servo to the current position
  //   delay(200); // Wait 15 milliseconds for the servo to reach the position
  // }

  // // Rotate back from 180 to 0 degrees
  // for (pos = 120; pos >= 0; pos -= 15) { // Decrement position by 1 degree
  //   myServo.write(pos); // Move servo to the current position
  //   delay(200); // Wait 15 milliseconds for the servo to reach the position
  // }

}
