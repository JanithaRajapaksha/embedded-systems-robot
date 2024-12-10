#include <QTRSensors.h>
QTRSensors qtr;

const uint8_t SensorCount = 6;
uint16_t sensorValues[SensorCount];

#define Kp 2 
#define Kd 40
#define MaxSpeed 255
#define BaseSpeed 230
#define speedturn 100

//int STBY = 10; 
//Motor A
int PWMA = 11; //for speed control
int AIN1 = 9; //Direction
int AIN2 = 10; //Direction
//Motor B
int PWMB = 6; //for speed control
int BIN1 = 8; //Direction
int BIN2 = 7; //Direction

int lastError=0;

void setup()
{   
    qtr.setTypeAnalog();
    qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5}, SensorCount);
    //qtr.setEmitterPin(2);pinMode(STBY, OUTPUT);
    pinMode(PWMA, OUTPUT);pinMode(AIN1, OUTPUT);pinMode(AIN2, OUTPUT);
    pinMode(PWMB, OUTPUT);pinMode(BIN1, OUTPUT);pinMode(BIN2, OUTPUT);
    delay(500);pinMode(LED_BUILTIN, OUTPUT);digitalWrite(LED_BUILTIN, HIGH);
    delay(3000);
    int i;
    for (int i = 0; i < 100; i++)
    {   
        if ( i  < 25 || i >= 75 ) 
        {   move(0,speedturn, 1);
            move(1,speedturn, 0);
        }
        else
        {   move(0,speedturn, 0);
            move(1,speedturn, 1);
        }
        qtr.calibrate();delay(20);
    }
    delay(3000); 
}  

void loop() {
    // Read sensor values and get the line position
    uint16_t position = qtr.readLineBlack(sensorValues);

    // Check if all leftmost sensors (A0, A1, A2) are detecting black (above a threshold)
    if (sensorValues[0] > 800 && sensorValues[1] > 800 && sensorValues[2] > 800) {
        // Sharp right turn
        move(1, speedturn, 1); // Move right motor forward
        move(0, speedturn, 0); // Move left motor backward
        return;
    }
    
    // Check if all rightmost sensors (A3, A4, A5) are detecting black (above a threshold)
    if (sensorValues[3] > 800 && sensorValues[4] > 800 && sensorValues[5] > 800) {
        // Sharp left turn
        move(1, speedturn, 0); // Move right motor backward
        move(0, speedturn, 1); // Move left motor forward

        return;
    }

    // If neither sharp turn condition is met, continue with normal PID control
    if (position > 6700) {
        move(1, speedturn, 1); // Right motor forward
        move(0, speedturn, 0); // Left motor backward
        return;
    }

    if (position < 300) {
        move(1, speedturn, 0); // Right motor backward
        move(0, speedturn, 1); // Left motor forward
        return;
    }

    // Compute the error and PID control
    int error = position - 3500;
    int motorSpeed = Kp * error + Kd * (error - lastError);
    lastError = error;

    // Calculate motor speeds
    int rightMotorSpeed = BaseSpeed + motorSpeed;
    int leftMotorSpeed = BaseSpeed - motorSpeed;

    // Ensure motor speeds are within allowed limits
    if (rightMotorSpeed > MaxSpeed) rightMotorSpeed = MaxSpeed; 
    if (leftMotorSpeed > MaxSpeed) leftMotorSpeed = MaxSpeed;
    if (rightMotorSpeed < 0) rightMotorSpeed = 0;    
    if (leftMotorSpeed < 0) leftMotorSpeed = 0;

    // Move motors based on calculated speeds
    move(1, rightMotorSpeed, 1);
    move(0, leftMotorSpeed, 1);
}


void move(int motor, int speed, int direction)
{   
    boolean inPin1=HIGH,inPin2=LOW;
    if(direction == 1){inPin1 = HIGH;inPin2 = LOW;}  
    if(direction == 0){inPin1 = LOW; inPin2 = HIGH;}
    if(motor == 0)
    {   digitalWrite(AIN1, inPin1);digitalWrite(AIN2, inPin2);analogWrite(PWMA, speed);
    }
    if(motor == 1)
    {   digitalWrite(BIN1, inPin1);digitalWrite(BIN2, inPin2);analogWrite(PWMB, speed);
    }  
}