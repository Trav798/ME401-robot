#include "servo_motors.h"

Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;

void setupServos(void)
{   
    ESP32PWM::allocateTimer(3);
    servo1.setPeriodHertz(50);    // standard 50 hz servo
    servo2.setPeriodHertz(50);    // standard 50 hz servo
    servo3.setPeriodHertz(50);    // standard 50 hz servo
    servo4.setPeriodHertz(50);    // standard 50 hz servo
    
    // TODO: Should we permanently attach these, or let students do that later
    // servo1.attach(SERVO1_PIN, 500, 2400); // attaches the servo on pin 18 to the servo object
    // servo2.attach(SERVO2_PIN, 500, 2400); // attaches the servo on pin 18 to the servo object
    // servo3.attach(SERVO3_PIN, 500, 2400); // attaches the servo on pin 18 to the servo object
    // servo4.attach(SERVO4_PIN, 500, 2400); // attaches the servo on pin 18 to the servo object
}