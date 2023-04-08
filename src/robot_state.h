#ifndef ROBOT_STATE_H_
#define ROBOT_STATE_H_

#include <Arduino.h>
#undef B1 // arduino.h's definition of this macro conflicts with Eigen

#include <Eigen.h>


#define UP_ROT_EPSILON .001

using namespace Eigen;

enum CurrentState {
    STANDBY = 0,
    BALL = 1,
    BACKUP = 2,
    APPROACH = 3,
    HOME = 4,
};

class RobotState {
private:
    Rotation2Df rotation;
    Vector2f frontVector;
    Vector2f position;
    int16_t lastx = 0;
    int16_t lasty = 0;
public:
    RobotState();
    bool updateRobotPosition(int16_t robotx, int16_t roboty);
    bool updateRobotRotation(float theta);
    Rotation2Df getRotation();
    Vector2f getFrontVector();
    Vector2f getPosition();
};


#endif