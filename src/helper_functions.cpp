#include <helper_functions.h>


float distanceBetween(Vector2f origin, Vector2f point) {
  Vector2f difference = point - origin;
  return difference.norm();
}

float angleBetween(Vector2f vec1, Vector2f vec2) {
  // Serial.printf("angleBetween x: %f, y: %f\n", vec1(0), vec1(1));
  // Make clockwise positive  
  float result = (vec1(0)*vec2(1)) - (vec1(1)*vec2(0));
  int dir = 1;
  if ( result > 0) {
    dir = -1;
  }

  return dir * acos(vec1.dot(vec2) / (vec1.norm()*vec2.norm()));
}

float rad2deg(float rad) {
  return (360*rad) / (2*M_PI);
}

int minSpeed(int input, int min) {
  int output = input;
  if (abs(input) < min) {
    output = (input/abs(input))*min;
  }

  return output;
}

float twoPointSlope(Vector2f point1, Vector2f point2) {
    return (point2(0) - point1(0)) / (point2(1) - point1(1));
}



// Vector2f selectTarget(RobotState robotState, BallPosition ballArray[NUM_BALLS]) {
  
// }