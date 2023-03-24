#pragma once

#include "common.h"
#include <ESP32Servo.h>

const char SERVO1_PIN = 26;
const char SERVO2_PIN = 25;
const char SERVO3_PIN = 33;
const char SERVO4_PIN = 17;

extern Servo servo1;
extern Servo servo2;
extern Servo servo3;
extern Servo servo4;

void setupServos(void);