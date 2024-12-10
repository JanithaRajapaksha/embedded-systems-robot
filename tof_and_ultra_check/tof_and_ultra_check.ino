#include <Wire.h>
#include "Adafruit_VL6180X.h"

// Initialize the ToF sensor
Adafruit_VL6180X vl = Adafruit_VL6180X();

// Define pins for the ultrasonic sensors
#define TRIG1_PIN 25
#define ECHO1_PIN 26
#define TRIG2_PIN 27
#define ECHO2_PIN 14

void setup() {
  Serial.begin(115200);
  // while (!Serial) delay(1);

  // Initialize VL6180X sensor
  Serial.println("Initializing VL6180X ToF sensor...");
  if (!vl.begin()) {
    Serial.println("Failed to find ToF sensor");
    while (1);
  }
  Serial.println("ToF Sensor found!");

  // Setup pins for the ultrasonic sensors
  pinMode(TRIG1_PIN, OUTPUT);
  pinMode(ECHO1_PIN, INPUT);
  pinMode(TRIG2_PIN, OUTPUT);
  pinMode(ECHO2_PIN, INPUT);
}

void loop() {
  // 1. Read the ToF sensor
  float lux = vl.readLux(VL6180X_ALS_GAIN_5);
  // Serial.print("Lux: "); Serial.println(lux);

  uint8_t range = vl.readRange();
  uint8_t status = vl.readRangeStatus();

  if (status == VL6180X_ERROR_NONE) {
    Serial.print("ToF Range: "); Serial.print(range); Serial.println(" mm");
  } else {
    Serial.println("Error reading ToF sensor");
  }

  // 2. Read the first ultrasonic sensor
  long distance1 = readUltrasonicDistance(TRIG1_PIN, ECHO1_PIN);
  Serial.print("Ultrasonic 1 Distance: "); Serial.print(distance1); Serial.println(" mm");

  // 3. Read the second ultrasonic sensor
  long distance2 = readUltrasonicDistance(TRIG2_PIN, ECHO2_PIN);
  Serial.print("Ultrasonic 2 Distance: "); Serial.print(distance2); Serial.println(" mm");

  Serial.println("--------------------------");
  delay(100);
}

// Function to read the distance from an HC-SR04 ultrasonic sensor
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
