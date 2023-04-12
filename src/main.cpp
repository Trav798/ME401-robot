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
#include <ESP32Servo.h>
#include <iostream>

using namespace std;

#define MY_ROBOT_ID 7
#define IR_PIN_IN 35
// #define IR_SERVO_PIN 999
#define INDICATOR_PIN 5
#define MIN_MOTOR_SPEED 20
#define LEFT_SWITCH 18
#define RIGHT_SWITCH 19
#define GATE_PIN 22 
#define HOME_POS 1870,130
#define OBSTACLE_EPSILON 300


#define MOVEMENT_TIMEOUT 500
#define ROTATION_EPSILON .1
#define DRIVE_EPSILON 250
#define MAX_BALLS 4

void ballState();
void backupState();
void standbyState(int numBalls);
void approachState();
void homeState();
int setServo3Speed(int speed);
int setServo4Speed(int speed);
void leftLimitCallback();
void rightLimitCallback();
bool chooseBallTarget(BallPosition balls[NUM_BALLS], int numBalls);
float moveToTarget();
void stopRobot();
void openGate();
void closeGate();
void setDC(double rot);



// MedianFilter<float> medianFilter(10);



// PID setup
// double kp = 100.0; double ki=7700.0; double kd=8.125; // was 100, 7700, 8.125
// double kp = 100.0; double ki=0.0; double kd=0.0; 
double kp = 6000; double ki=79470; double kd=1130.25; 
double setpoint = 0;
int pos = 0;
Servo gateMotor;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  setupDCMotors();
  setupServos();
  setupCommunications();

  pinMode(INDICATOR_PIN, OUTPUT);
  pinMode(LEFT_SWITCH, INPUT_PULLDOWN);
  pinMode(RIGHT_SWITCH, INPUT_PULLDOWN);
  pinMode(IR_PIN_IN, INPUT);
  attachInterrupt(LEFT_SWITCH, leftLimitCallback, RISING);
  attachInterrupt(RIGHT_SWITCH, rightLimitCallback, RISING);

  gateMotor.attach(GATE_PIN);
  delay(100);
  closeGate();
  delay(500);
  setPIDgains1(kp,ki,kd);


  // servo1.attach(SERVO1_PIN, 1300, 1700);
  // servo2.attach(SERVO2_PIN, 1300, 1700);
  servo3.attach(SERVO3_PIN, 1300, 1700); // right
  servo4.attach(SERVO4_PIN, 1300, 1700); // left

  // servo3.writeMicroseconds(1500);
  // servo4.writeMicroseconds(1500);
}








//////////////////////////////// GLOBAL VARIABLES ///////////////////////////////////
CurrentState state = BALL;
RobotState robotState;
BallPosition ballArray[NUM_BALLS];

Vector2f targetPos(HOME_POS);

const float kpDrive = .2;
const float kpRot = 6;
bool rotating = true;
bool firstBackup = true;

unsigned long updateTarget = millis();
bool freezeTarget = false;
int ballCount = 0;
unsigned long lastPosUpdate = 0;










/////////////////////////////// MAIN BODY ///////////////////////////////////////
void loop() {
  Serial.println("------------------------------");
  // Update robot info
  RobotPose poseData =  getRobotPose(MY_ROBOT_ID);
  lastPosUpdate = robotState.updateRobotPosition(poseData.x, poseData.y) ? millis() : lastPosUpdate;
  robotState.updateRobotRotation(((float)poseData.theta)/1000); 
  // Serial.printf("raw theta: %d\n", poseData.theta);


  // Update ball info only if all requirements are met
  int numBalls = getBallPositions(ballArray);
  printBallPositions(numBalls,ballArray);
  if (millis() > updateTarget && !freezeTarget) { 
    if(numBalls > 0 && chooseBallTarget(ballArray, numBalls)) {
      updateTarget = millis() + 2500;
    } else {
      freezeTarget = true;
      rotating = true;
      targetPos = Eigen::Vector2f(HOME_POS);
      state = HOME;
    }
  }
  Serial.printf("targetPos: x: %f, y: %f\n", targetPos(0), targetPos(1));


  // Handle different states
  if (state == BALL) {
    Serial.println("BALL state");
    ballState();
  } else if (state == STANDBY) {
    Serial.println("STANDBY state");
    standbyState(numBalls);
  } else if (state == BACKUP) {
    Serial.println("BACKUP state");
    backupState();
  } else if (state == APPROACH) {
    Serial.println("APPROACH state");
    approachState();
  } else if (state == HOME) {
    Serial.println("HOME state");
    homeState();
  } else {
    Serial.println("Unknown state");
  }

  delay(50);
}








////////////////////////// STATES /////////////////////
void ballState() {
  float distanceError = 2*DRIVE_EPSILON;
  if (millis() - lastPosUpdate < MOVEMENT_TIMEOUT) {
    distanceError = moveToTarget();
  } else {
    stopRobot();
  }

  if (distanceError < DRIVE_EPSILON) {
    rotating = true;
    state = APPROACH;
  }
}

void backupState() {
  if (firstBackup) {
    //backup robot before scanning
    setServo3Speed(-100);
    setServo4Speed(-100);
    delay(1000);
    stopRobot();

    float distances[19]; // Array to store distances
    setDC(0);
    delay(500);
    // Scan 18 times by changing the dc motor position by 10 degrees each time and then taking a scan TODO: change motor from Servo to DC
    for (int irDCPos = 0; irDCPos <= 18; irDCPos++) { //TODO, THE INITIAL AND FINAL POSITION WILL BE DETERMIONED UPON TESTING (MOST LIKELY NOT JUST 0 AND 180 BUT 45 AND 225)
        setDC((double)irDCPos*10);
        delay(250);
        distances[irDCPos] = convertVoltage2Distance(analogRead(IR_PIN_IN));
        delay(50);
    }
    setDC(0);
    

    // this for loop should populate obstacleEdges with the positions the robot needs to go to to pass all obstacle edges
    Vector2f avoidVectors[4] = {Eigen::Vector2f(0,0), Eigen::Vector2f(0,0), Eigen::Vector2f(0,0), Eigen::Vector2f(0,0)};
    int len = 0;
    // 85 75 65 55 45 35 25 15 5 -5 -15 -25 -35 -45 -55 -65 -75 -85
    for(int i = 0; i < 18; i++) {
      float distanceDiff = abs(distances[i] - distances[i+1]);
      // the scan starts all the way to the left and moves clockwise. If avoidDir = 1, avoid to the left. If it is negative, avoid to the right
      float avoidDir = (distances[i] - distances[i+1]) / distanceDiff;
      float currentAngle = 85 - (10*i);
      if (distanceDiff > OBSTACLE_EPSILON) {
        avoidVectors[i] = robotState.getPosition() + (Eigen::Rotation2Df(currentAngle + (avoidDir*15)) * robotState.getFrontVector() * 500); // TODO: 500 is a magic number rn
        len++;
      }
      if(len > 4) { // so we don't go beyond the bounds of the array
        break;
      }
    }

    // if not way around the obstacle is detected
    if (len == 0) {
      setServo3Speed(-50);
      setServo4Speed(50);
      delay(1000);
      setServo3Speed(100);
      setServo4Speed(100);
      delay(3000);
      return;
    } 

    int smallestIndex = 0;
    float smallest = distanceBetween(avoidVectors[0], robotState.getPosition());
    for (int i = 1; i < len - 1; i++) {
      float tempDist = distanceBetween(avoidVectors[i], robotState.getPosition());
      if (tempDist < smallest) {
        smallestIndex = i;
        smallest = tempDist;
      }
    }

    targetPos = avoidVectors[smallestIndex];

    freezeTarget = true;
    firstBackup = false;


    

    // //take array and find biggest difference as well as the smallest distance used for vector calculation, also finds angle from getfrontvector
    // float biggestDifference = 0.0;
    // float smallestDistance = 0.0;
    // int anglePos = 0;
    // for (int i = 1; i < 17; i++) {
    //   int difference = abs(distances[i] - distances[i-1]);
    //   if (difference > biggestDifference) {
    //     if (distances[i] > distances[i-1]){
    //       biggestDifference = difference;
    //       smallestDistance = distances[i-1];
    //       if (i < 9) {
    //         anglePos = 90 - (10 * (i-1)+15) * M_PI/180;
    //       } else if ( i > 9) {
    //         anglePos = 90 -(10 * (i-1)-15) * M_PI/180;
    //       } else {
    //         anglePos = -15 * M_PI/180;
    //       }
    //     }
    //     else {
    //       biggestDifference = difference;
    //       smallestDistance = distances[i];
    //       if (i < 9) {
    //         anglePos = 90 - (10 * (i-1)+15) * M_PI/180;
    //       } else if ( i > 9) {
    //         anglePos = 90 -(10 * (i-1)-15) * M_PI/180;
    //       } else {
    //         anglePos = -15 * M_PI/180;
    //       }
    //     } 
    //   }
      // Vector2f avoidVector =  Eigen::Rotation2Df(anglePos) * robotState.getFrontVector();
      // avoidVector = avoidVector * (1.5 * smallestDistance);
      // targetPos = robotState.getPosition() + avoidVector;
      // freezeTarget = true;
      // firstBackup = false;
    // }
  } else {

    float distanceError = 2*DRIVE_EPSILON;
    if (millis() - lastPosUpdate < MOVEMENT_TIMEOUT) {
      distanceError = moveToTarget();
    } else {
      stopRobot();
    }


    if (distanceError < DRIVE_EPSILON / 2) {
      freezeTarget = false;
      firstBackup = true;
      state = BALL;
    }
  }
  

  


}

void standbyState(int numBalls) {
  stopRobot();
  digitalWrite(INDICATOR_PIN, HIGH);
  BallPosition temp[NUM_BALLS];
  if (chooseBallTarget(temp, NUM_BALLS) > 0) {
    state = BALL;
  }
}


void approachState() {
  //  Vector2f currentPos = robotState.getPosition();
  // openGate();
  setServo3Speed(100);
  setServo4Speed(100);
  delay(2000);
  // closeGate();
  stopRobot();

  ballCount++;
  if (ballCount > MAX_BALLS) {
    freezeTarget = true;
    targetPos = Eigen::Vector2f(HOME_POS);
    state = HOME;
  } else {
    updateTarget = millis()-10;
    state = BALL;
  }
 }

 void homeState() {
  float distanceError = 2*DRIVE_EPSILON;
  if (millis() - lastPosUpdate < MOVEMENT_TIMEOUT) {
    distanceError = moveToTarget();
  } else {
    stopRobot();
  }

  // release the balls
  if (distanceError < DRIVE_EPSILON/2) {
    Serial.println("Releasing balls");
    stopRobot();
    openGate();
    delay(1000);
    setServo3Speed(-100);
    setServo4Speed(-100);
    delay(2500);
    closeGate();

    freezeTarget = false;
    rotating = true;
    targetPos = Eigen::Vector2f(1000,1000); // so it doesn't immediately try to approach
    ballCount = 0;
    state = BALL;
  }
 }











////////////////////// ADDITIONAL HELPER FUNCTIONS ////////////////////////////////
int setServo3Speed(int speed) {
  int servo3Speed = minSpeed(speed, MIN_MOTOR_SPEED);
  servo3.writeMicroseconds(1500 + servo3Speed);
  return servo3Speed;
}

int setServo4Speed(int speed) {
  int servo4Speed = minSpeed(speed, MIN_MOTOR_SPEED);
  servo4.writeMicroseconds(1500 - servo4Speed);
  return servo4Speed;
}

void IRAM_ATTR leftLimitCallback() {
  state = BACKUP;
}

void IRAM_ATTR rightLimitCallback() {
  state = BACKUP;
}

bool chooseBallTarget(BallPosition balls[NUM_BALLS], int numBalls) {
  int smallest = -1;
  float lowestDistance = 99999;
  Serial.printf("numBalls: %d\n", numBalls);
  for (int i = 0; i < numBalls; i++) {
    Vector2f ballPos((float)balls[i].x, (float)balls[i].y);
    float tempDist = distanceBetween(ballPos, robotState.getPosition());
    float homeDist = distanceBetween(ballPos, Eigen::Vector2f(HOME_POS));
    if (tempDist < lowestDistance && homeDist > DRIVE_EPSILON) {
      smallest = i;
      lowestDistance = tempDist;
    }
  }

  if (smallest < 0) {
    Serial.println("No valid ball targets");
    return false;
  } else {
    targetPos = Eigen::Vector2f((float)balls[smallest].x, (float)balls[smallest].y);
    return true;
  }
}

float moveToTarget() {
  // calculate the pointing error to decide if we are too far off track and need to rotate again
  float thetaError = angleBetween(robotState.getFrontVector(), targetPos - robotState.getPosition());
  // calculate distance from ball
  float distanceError = distanceBetween(robotState.getPosition(), targetPos);
  Serial.printf("thetaError: %f\n", thetaError);
  if (rotating) {
    // calculate servo speed
    int servoSpeed = (int)(kpRot*thetaError);
    Serial.printf("servo3Speed(right): %d\n", setServo3Speed(servoSpeed));
    Serial.printf("servo4Speed(left): %d\n", setServo4Speed(-servoSpeed));
    // Serial.printf("theta error: %f\n", thetaError);

    // if the pointing error is low enough, start driving towards the ball
    if (abs(thetaError) < ROTATION_EPSILON) {
      rotating = false;
      stopRobot();
    }
  } else {

    // do control loop, set robot linear speed
    int servoSpeed = (int)(kpDrive*distanceError);
    Serial.printf("servoSpeed: %d\n", servoSpeed);
    setServo3Speed(servoSpeed);
    setServo4Speed(servoSpeed);

    // start rotating again if too far off track
    if (abs(thetaError) > 2*ROTATION_EPSILON) {
      Serial.println("Off course, rotating");
      rotating = true;
      return distanceError;
    }
  }
  return distanceError;
}

void stopRobot() {
  servo3.writeMicroseconds(1500);
  servo4.writeMicroseconds(1500);
}

void openGate() {
  gateMotor.write(0);
}

void closeGate() {
  gateMotor.write(90);
}

void setDC(double rot) {
  Serial.printf("dcPosition: %f\n", getPosition1());
  setSetpoint1(((-rot)/180)*1400);
}

