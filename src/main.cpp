#include <Arduino.h>
#undef B1 // arduino.h's definition of this macro conflicts with Eigen


#include "debug.h"
#include "common.h"
#include "dc_motors.h"
#include "servo_motors.h"
#include "communications.h"
#include "stdint.h"
#include <MedianFilterLib.h>
#include "helper_functions.h"
#include <Eigen.h>
#include "robot_state.h"

#define MY_ROBOT_ID 7
#define IR_PIN_IN 34
#define INDICATOR_PIN 20 // TODO select a pin
#define MIN_MOTOR_SPEED 20

#define MOVEMENT_TIMEOUT 500
#define ROTATION_EPSILON .2
#define DRIVE_EPSILON 20

void rotateState(unsigned long lastRotUpdate);
void driveState(unsigned long lastPosUpdate);
void standbyState();

// MedianFilter<float> medianFilter(10);



// PID setup
double kp = 300.0; double ki=7700.0; double kd=8.125; 
double setpoint = 0;
int pos = 0;



void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  setupDCMotors();
  setupServos();
  setupCommunications();
  pinMode(INDICATOR_PIN, OUTPUT);

  // setPIDgains1(kp,ki,kd);

  // servo1.attach(SERVO1_PIN, 1300, 1700);
  // servo2.attach(SERVO2_PIN, 1300, 1700);
  servo3.attach(SERVO3_PIN, 1300, 1700); // right
  servo4.attach(SERVO4_PIN, 1300, 1700); // left

  // servo3.writeMicroseconds(1500);
  // servo4.writeMicroseconds(1500);
}


// extern double output1;// don't need i think?

CurrentState state = ROTATE;
RobotState robotState;
BallPosition ballArray[NUM_BALLS];

Vector2f targetPos(0,0);

const float kpDrive = .2;
const float kpRot = 15;

unsigned long lastPosUpdate = 0;
unsigned long lastRotUpdate = 0;




void loop() {
  Serial.println("------------------------------");
  // Update robot info
  RobotPose poseData =  getRobotPose(MY_ROBOT_ID);
  lastPosUpdate = robotState.updateRobotPosition(poseData.x, poseData.y) ? millis() : lastPosUpdate;
  lastRotUpdate = robotState.updateRobotRotation(((float)poseData.theta)/1000) ? millis() : lastRotUpdate; 
  Serial.printf("raw theta: %d\n", poseData.theta);

  // Update ball info
  int numBalls = getBallPositions(ballArray);
  if (numBalls > 0) { // TODO handle multiple balls
    targetPos(0) = (float)ballArray[0].x;
    targetPos(1) = (float)ballArray[0].y;
  }
  Serial.printf("targetPos: x: %f, y: %f\n", targetPos(0), targetPos(1));

  // Handle different states
  // TODO update state diagram to include nested states
  if (state == ROTATE) {
    Serial.println("ROTATE state");
    rotateState(lastRotUpdate);
  } else if (state == DRIVE) {
    Serial.println("DRIVE state");
    driveState(lastPosUpdate);
  } else if (state == STANDBY) {
    standbyState();
    Serial.println("STANDBY state");
  }  else {
    Serial.println("Unknown state");
  }

  // // IR SENSOR STUFF
  // // Scan 180 degrees in front of robot with ir sensor and
  // // store results in the array irReadings
  // float irReadings[15];
  // int index = 0;
  // setSetpoint1(setpoint);
  // for (int i = -700; i <= 700; i += 100) {
  //   irReadings[index] = convertVoltage2Distance((float)analogRead(IR_PIN_IN));
  //   index++;
  //   setpoint = i;
  //   setSetpoint1(setpoint);
  //   delay(200);
  // }
  // irReadings[index] = convertVoltage2Distance(analogRead(IR_PIN_IN));
  delay(100);
}



////////////////////////// STATES /////////////////////
void rotateState(unsigned long lastRotUpdate) {
  bool rotationComplete = false;
  // check if the robots position is being updated
  if (millis() - lastRotUpdate < MOVEMENT_TIMEOUT) {
    // calculates the current pointing error
    float thetaError = angleBetween(robotState.getFrontVector(), targetPos - robotState.getPosition());

    int servo3Speed = (int)(kpRot*thetaError);
    int servo4Speed = (int)(-1*kpRot*thetaError);
    Serial.printf("servo3Speed: %d\n", setServo3Speed(servo3, servo3Speed)); // should work, maybe not
    Serial.printf("servo4Speed: %d\n", setServo4Speed(servo4, servo4Speed));
    Serial.printf("theta error: %f\n", thetaError);

    // if the pointing error is low enough, start driving towards the ball
    rotationComplete = abs(thetaError) < ROTATION_EPSILON;
  } else {
    // stop the robot if the camera has lost contact
    servo3.writeMicroseconds(1500);
    servo4.writeMicroseconds(1500);
    Serial.println("Rotation not updating, stopping the robot");
  }

  // change the state if the rotation is done
  if (rotationComplete) {
    state = DRIVE;
  }
}

void driveState(unsigned long lastPosUpdate) {
  bool driveComplete = false;
  // calculate the pointing error to decide if we are too far off track and need to rotate again
  float thetaError = angleBetween(robotState.getFrontVector(), targetPos - robotState.getPosition());
  Serial.printf("thetaError: %f\n", thetaError);
  if (abs(thetaError) > 2*ROTATION_EPSILON) {
    // change state to ROTATE if we are too far off track
    Serial.println("Off course, rotating");
    state = ROTATE;
    return;
  } else if (millis() - lastPosUpdate < MOVEMENT_TIMEOUT) {
    // calculate distance from ball
    float distanceError = distanceBetween(robotState.getPosition(), targetPos);

    // do control loop, set robot linear speed
    int servoSpeed = (int)(kpDrive*distanceError);
    Serial.printf("servoSpeed: %d\n", servoSpeed);
    setServo3Speed(servo3, servoSpeed);
    setServo4Speed(servo4, servoSpeed);

    // decide if we have reached the ball
    driveComplete = distanceError < DRIVE_EPSILON;
  } else {
    // if the camera has lost contact, stop the robot
    servo3.writeMicroseconds(1500);
    servo4.writeMicroseconds(1500);
    Serial.println("Position not updating");
  }

  // if we have reached the ball, stop
  if (driveComplete) {
    state = STANDBY;
  }
}

void standbyState() {
  digitalWrite(INDICATOR_PIN, HIGH);
}
