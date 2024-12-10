/*
* Lesson 52: Coontrol a DC motor with Relay using Arduino | Arduino Step By Step Course
Basic code
 * Arduino code change the direction of rotation 
 * of a DC motor with 2 relays.
 
 * 
 * This is basic code. I have advanced code which can be used in both 
 * for Low-level trigger and High-lever trigger relay with clean code
Please watch video explaining this code : https://youtu.be/2n0vUa0cZOI

 * 
 * Written by Ahmad Shamshiri for Robojax.com on 
 * Sunday August 18, 2019 
 * at 20:22 in Ajax, Ontario, Canada
 * 

This code is available at http://robojax.com/course1/?vid=lecture52
 
with over 100 lectures Free On  YouTube Watch it here http://robojax.com/L/?id=338
Get the code for the course: http://robojax.com/L/?id=339  


or make donation using PayPal http://robojax.com/L/?id=64
* 
 * This code is "AS IS" without warranty or liability. Free to be used as long as you keep this note intact.* 
 * This code has been download from Robojax.com
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

int IR = 22;
int relay1 = 12;


void setup() {
  Serial.begin(115200);

    pinMode(relay1, OUTPUT);// set pin as output for relay 1
pinMode(IR, INPUT);


    // keep the motor off by keeping both HIGH
    digitalWrite(relay1, LOW); 


 
  
}



void loop() {

 // Rotate in CCW direction
 if (digitalRead(IR) == 0){
  Serial.println("ON");
  digitalWrite(relay1, HIGH);// turn relay 1 ON

  // delay(1000);// wait for 3 seconds
 } else {
 // stop the motor
  digitalWrite(relay1, LOW);// turn relay 1 OFF
  Serial.println("OFF");


  // delay(1000);// stop for 2 seconds
 }
// digitalWrite(relay1, HIGH);// turn relay 1 ON

//   delay(1000);// wait for 3 seconds

//  // stop the motor
//   digitalWrite(relay1, LOW);// turn relay 1 OFF



//   delay(1000);// stop for 2 seconds
          
}// loop end



 