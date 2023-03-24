#include <robot_state.h>

RobotState::RobotState() : rotation(0), frontVector(1,0), position(0,0) {
    Serial.println("Robot state class has been initialized");
}

/**
 * @brief Update the position data for the robot
 * 
 * @param robotx int, robot x position
 * @param roboty int, robot y position
 * @return true: the position has changed since last updated
 * @return false: the position hasn't changed since last updated
 */
bool RobotState::updateRobotPosition(int16_t robotx, int16_t roboty) {
    bool hasChanged = true;
    if (robotx == this->lastx && roboty == this->lasty) {
        hasChanged = false;
    }
    this->position = Eigen::Vector2f((float)robotx, (float)roboty);
    return hasChanged;
}

bool RobotState::updateRobotRotation(float theta) {
    bool hasChanged = true;
    if (abs(theta - this->rotation.angle()) < UP_ROT_EPSILON) {
        hasChanged = false;
    }
    this->rotation = Eigen::Rotation2Df(theta); // counterclockwise is positive for eigen
    this->frontVector = this->rotation * Eigen::Vector2f(1,0);
    return hasChanged;
}


Rotation2Df RobotState::getRotation() {
    return this->rotation;
}

Vector2f RobotState::getFrontVector() {
    return this->frontVector;
}

Vector2f RobotState::getPosition() {
    return this->position;
}

