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
#define IR_PIN_IN 21
// #define IR_SERVO_PIN 999
#define INDICATOR_PIN 5
#define MIN_MOTOR_SPEED 20
#define LEFT_SWITCH 18
#define RIGHT_SWITCH 19
#define GATE_PIN 22 
#define HOME_POS 1800,200


#define MOVEMENT_TIMEOUT 500
#define ROTATION_EPSILON .2
#define DRIVE_EPSILON 250

void ballState(unsigned long lastRotUpdate);
void backupState();
void standbyState(int numBalls);
void approachState();
void homeState();
int setServo3Speed(int speed);
int setServo4Speed(int speed);
void leftLimitCallback();
void rightLimitCallback();
Vector2f chooseBallTarget(BallPosition balls[NUM_BALLS], int numBalls);
float moveToTarget();
void stopRobot();
void openGate();
void closeGate();
void setDC(int rot);



// MedianFilter<float> medianFilter(10);



// PID setup
double kp = 300.0; double ki=7700.0; double kd=8.125; 
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
const float kpRot = 12;
bool rotating = true;

unsigned long updateTarget = millis();
bool freezeTarget = false;
int ballCount = 0;










/////////////////////////////// MAIN BODY ///////////////////////////////////////
void loop() {
  Serial.println("------------------------------");
  // Update robot info
  RobotPose poseData =  getRobotPose(MY_ROBOT_ID);
  unsigned long lastPosUpdate = robotState.updateRobotPosition(poseData.x, poseData.y) ? millis() : lastPosUpdate;
  robotState.updateRobotRotation(((float)poseData.theta)/1000); 
  // Serial.printf("raw theta: %d\n", poseData.theta);


  // Update ball info only if all requirements are met
  int numBalls = getBallPositions(ballArray);
  if (millis() > updateTarget && !freezeTarget && numBalls > 0) { 
    targetPos = chooseBallTarget(ballArray, numBalls);
    // targetPos(0) = (float)ballArray[0].x;
    // targetPos(1) = (float)ballArray[0].y;
    updateTarget = millis() + 2500;
  } else if (numBalls == 0) {
    state = STANDBY;
  }
  Serial.printf("targetPos: x: %f, y: %f\n", targetPos(0), targetPos(1));


  // Handle different states
  if (state == BALL) {
    Serial.println("BALL state");
    ballState(lastPosUpdate);
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

  delay(100);
}








////////////////////////// STATES /////////////////////
void ballState(unsigned long lastPosUpdate) {
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

  // if (digitalRead(RIGHT_SWITCH) == LOW){
  //   servo3.writeMicroseconds(-100);
  //   servo4.writeMicroseconds(-75);
  // } else if (digitalRead(LEFT_SWITCH) == LOW) {
  //   servo3.writeMicroseconds(-75);
  //   servo4.writeMicroseconds(-100);
  // } else {
  //   servo3.writeMicroseconds(-100);
  //   servo4.writeMicroseconds(-100);
  // }

  //backup robot before scanning
  setServo3Speed(-100);
  setServo4Speed(-100);
  delay(1000);
  stopRobot();

  int distances[18]; // Array to store distances
  int irDCPos = 0; // Current servo position //TODO: Find the right value for this position(should be far left position)
  setDC(irDCPos);

  //MAYBE A DELAY HERE SO THE SERVO CAN REACH HOME BEFORE ANOTHER SCAN

  // Scan 18 times by changing the dc motor position by 10 degrees each time and then taking a scan TODO: change motor from Servo to DC
  for (irDCPos = 0; irDCPos <= 180; irDCPos += 10) { //TODO, THE INITIAL AND FINAL POSITION WILL BE DETERMIONED UPON TESTING (MOST LIKELY NOT JUST 0 AND 180 BUT 45 AND 225)
      setDC(irDCPos);
      delay(1000);
      distances[irDCPos] = analogRead(IR_PIN_IN);
  }

  //take array and find biggest difference as well as the smallest distance used for vector calculation, also finds angle from getfrontvector
  float biggestDifference = 0.0;
  float smallestDistance = 0.0;
  int anglePos = 0;
  for (int i = 1; i < 17; i++) {
    int difference = abs(distances[i] - distances[i-1]);
    if (difference > biggestDifference) {
      if (distances[i] > distances[i-1]){
        biggestDifference = difference;
        smallestDistance = distances[i-1];
        if (i < 9) {
          anglePos = 90 - (10 * (i-1)+15) * M_PI/180;
        } else if ( i > 9) {
          anglePos = 90 -(10 * (i-1)-15) * M_PI/180;
        } else {
          anglePos = -15 * M_PI/180;
        }
      }
      else {
        biggestDifference = difference;
        smallestDistance = distances[i];
        if (i < 9) {
          anglePos = 90 - (10 * (i-1)+15) * M_PI/180;
        } else if ( i > 9) {
          anglePos = 90 -(10 * (i-1)-15) * M_PI/180;
        } else {
          anglePos = -15 * M_PI/180;
        }
      } 
    }
    Vector2f avoidVector =  Eigen::Rotation2Df(anglePos) * robotState.getFrontVector();
    avoidVector = avoidVector * (1.5 * smallestDistance);

    Serial.println("avoidVector");
    cout << avoidVector << endl;



  }
  

  


}

void standbyState(int numBalls) {
  stopRobot();
  digitalWrite(INDICATOR_PIN, HIGH);
  if (numBalls > 0) {
    state = BALL;
  }
}


void approachState() {
//  Vector2f currentPos = robotState.getPosition();
  openGate();
  setServo3Speed(100);
  setServo4Speed(100);
  delay(1000);
  closeGate();
  delay(500);
  stopRobot();

  ballCount++;
  if (ballCount > 3) {
    freezeTarget = true;
    targetPos = Eigen::Vector2f(HOME_POS);
    state = HOME;
  } else {
    state = BALL;
  }
 }

 void homeState() {
  float distanceError = moveToTarget();

  // release the balls
  if (distanceError < 1.5*DRIVE_EPSILON) {
    openGate();
    setServo3Speed(-100);
    setServo4Speed(-100);
    delay(1500);
    closeGate();

    freezeTarget = false;
    rotating = true;
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

Vector2f chooseBallTarget(BallPosition balls[NUM_BALLS], int numBalls) {
  int smallest = 0;
  float lowestDistance = distanceBetween(Eigen::Vector2f((float)balls[0].x, (float)balls[0].y), robotState.getPosition());
  for (int i = 1; i < numBalls; i++) {
    float tempDist = distanceBetween(Eigen::Vector2f((float)balls[i].x, (float)balls[i].y), robotState.getPosition());
    if (tempDist < lowestDistance) {
      smallest = i;
      lowestDistance = tempDist;
    }
  }

  std::cout << Eigen::Vector2f((float)balls[smallest].x, (float)balls[smallest].y) << std::endl;
  return Eigen::Vector2f((float)balls[smallest].x, (float)balls[smallest].y);
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
  gateMotor.write(15);
}

void closeGate() {
  gateMotor.write(90);
}

void setDC(int rot) {
  setSetpoint1(map(0, 180, 0, 1400,rot));
}