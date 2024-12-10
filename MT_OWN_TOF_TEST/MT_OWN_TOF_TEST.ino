#include <Wire.h>
#include "Adafruit_VL6180X.h"

Adafruit_VL6180X vl = Adafruit_VL6180X();

#define sht1 6
#define sht2 5


void setup() {
  Serial.begin(115200);

  // wait for serial port to open on native usb devices
  while (!Serial) {
    delay(1);
  }

  Serial.println("Adafruit VL6180x test!");
  if (!vl.begin()) {
    Serial.println("Failed to find sensor");
    while (1)
      ;
  }
  Serial.println("Sensor found!");
}

void loop() {
  for (int i = 1; i <= 2; i++) {
    if (i == 1) {

      digitalWrite(sht1, HIGH);

      float lux1 = vl.readLux(VL6180X_ALS_GAIN_5);

      // Serial.print("Lux1: ");
      // Serial.println(lux1);

      uint8_t range1 = vl.readRange();
      uint8_t status1 = vl.readRangeStatus();

      if (status1 == VL6180X_ERROR_NONE) {
        Serial.print("Range1: ");
        Serial.print(range1);
      }

      // Some error occurred, print it out!

      if ((status1 >= VL6180X_ERROR_SYSERR_1) && (status1 <= VL6180X_ERROR_SYSERR_5)) {
        Serial.println("System error");
      } else if (status1 == VL6180X_ERROR_ECEFAIL) {
        Serial.println("ECE failure");
      } else if (status1 == VL6180X_ERROR_NOCONVERGE) {
        Serial.println("No convergence");
      } else if (status1 == VL6180X_ERROR_RANGEIGNORE) {
        Serial.println("Ignoring range");
      } else if (status1 == VL6180X_ERROR_SNR) {
        Serial.println("Signal/Noise error");
      } else if (status1 == VL6180X_ERROR_RAWUFLOW) {
        Serial.println("Raw reading underflow");
      } else if (status1 == VL6180X_ERROR_RAWOFLOW) {
        Serial.println("Raw reading overflow");
      } else if (status1 == VL6180X_ERROR_RANGEUFLOW) {
        Serial.println("Range reading underflow");
      } else if (status1 == VL6180X_ERROR_RANGEOFLOW) {
        Serial.println("Range reading overflow");
      }
      digitalWrite(sht1, LOW);
      delay(100);
    } else {

      digitalWrite(sht2, HIGH);

      float lux2 = vl.readLux(VL6180X_ALS_GAIN_5);

      // Serial.print("Lux2: ");
      // Serial.println(lux2);

      uint8_t range2 = vl.readRange();
      uint8_t status2 = vl.readRangeStatus();

      if (status2 == VL6180X_ERROR_NONE) {
        Serial.print("            Range2: ");
        Serial.println(range2);
      }

      // Some error occurred, print it out!

      if ((status2 >= VL6180X_ERROR_SYSERR_1) && (status2 <= VL6180X_ERROR_SYSERR_5)) {
        Serial.println("System error");
      } else if (status2 == VL6180X_ERROR_ECEFAIL) {
        Serial.println("ECE failure");
      } else if (status2 == VL6180X_ERROR_NOCONVERGE) {
        Serial.println("No convergence");
      } else if (status2 == VL6180X_ERROR_RANGEIGNORE) {
        Serial.println("Ignoring range");
      } else if (status2 == VL6180X_ERROR_SNR) {
        Serial.println("Signal/Noise error");
      } else if (status2 == VL6180X_ERROR_RAWUFLOW) {
        Serial.println("Raw reading underflow");
      } else if (status2 == VL6180X_ERROR_RAWOFLOW) {
        Serial.println("Raw reading overflow");
      } else if (status2 == VL6180X_ERROR_RANGEUFLOW) {
        Serial.println("Range reading underflow");
      } else if (status2 == VL6180X_ERROR_RANGEOFLOW) {
        Serial.println("Range reading overflow");
      }
      digitalWrite(sht2, LOW);
      delay(1000);
    }
  }
}