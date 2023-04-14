#ifndef HELPER_FUNCTIONS_H_
#define HELPER_FUNCTIONS_H_

#include <Arduino.h>
#undef B1 // arduino.h's definition of this macro conflicts with Eigen
#include "Eigen.h"
// #include "communications.h"
#include <robot_state.h>
#include <ESP32Servo.h>

#define MIN_MOTOR_SPEED 20

using namespace Eigen;

float angleBetween(Vector2f vec1, Vector2f vec2);
float distanceBetween(Vector2f origin, Vector2f point);
float rad2deg(float rad);
int minSpeed(int input, int min);
float twoPointSlope(Vector2f point1, Vector2f point2);
float convertVoltage2Distance(int input);


#endif