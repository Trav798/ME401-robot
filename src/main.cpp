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
#include "Servo.h"

#define MY_ROBOT_ID 7
#define IR_PIN_IN 34
#define INDICATOR_PIN 5
#define MIN_MOTOR_SPEED 20
#define LEFT_SWITCH 18
#define GATE_PIN 999 //TODO change to a real pin number

#define MOVEMENT_TIMEOUT 500
#define ROTATION_EPSILON .2
#define DRIVE_EPSILON 50

void rotateState(unsigned long lastRotUpdate);
void driveState(unsigned long lastPosUpdate);
void backupState();
void standbyState();
int setServo3Speed(int speed);
int setServo4Speed(int speed);
void leftLimitCallback();

Servo gateMotor;


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
  pinMode(LEFT_SWITCH, INPUT_PULLDOWN);
  attachInterrupt(LEFT_SWITCH, leftLimitCallback, RISING);
  gateMotor.attatch(GATE_PIN);
  gateMotor.write(90); //TODO: FIGURE OUT THE STARTING VALUE FOR THE SERVO MOTOR


  // setPIDgains1(kp,ki,kd);

  // servo1.attach(SERVO1_PIN, 1300, 1700);
  // servo2.attach(SERVO2_PIN, 1300, 1700);
  servo3.attach(SERVO3_PIN, 1300, 1700); // right
  servo4.attach(SERVO4_PIN, 1300, 1700); // left

  // servo3.writeMicroseconds(1500);
  // servo4.writeMicroseconds(1500);
}


//////////////////////////////// GLOBAL VARIABLES ///////////////////////////////////

CurrentState state = ROTATE;
RobotState robotState;
BallPosition ballArray[NUM_BALLS];

Vector2f targetPos(0,0);

const float kpDrive = .2;
const float kpRot = 18;

unsigned long lastPosUpdate = 0;
unsigned long lastRotUpdate = 0;



/////////////////////////////// MAIN BODY ///////////////////////////////////////
void loop() {
  Serial.println("------------------------------");
  // Update robot info
  RobotPose poseData =  getRobotPose(MY_ROBOT_ID);
  lastPosUpdate = robotState.updateRobotPosition(poseData.x, poseData.y) ? millis() : lastPosUpdate;
  lastRotUpdate = robotState.updateRobotRotation(((float)poseData.theta)/1000) ? millis() : lastRotUpdate; 
  // Serial.printf("raw theta: %d\n", poseData.theta);


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
    Serial.println("STANDBY state");
    standbyState();
  } else if (state == BACKUP) {
    Serial.println("BACKUP state");
    backupState();
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

    int servoSpeed = (int)(kpRot*thetaError);
    // int servo4Speed = (int)(-1*kpRot*thetaError);
    Serial.printf("servo3Speed(right): %d\n", setServo3Speed(-servoSpeed)); // should work, maybe not
    Serial.printf("servo4Speed(left): %d\n", setServo4Speed(servoSpeed));
    // Serial.printf("theta error: %f\n", thetaError);

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
    setServo3Speed(servoSpeed);
    setServo4Speed(servoSpeed);

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
    Vector2f vector1 =  targetPos - robotState.getPosition();
    vector1 *= 1.5;
    Vector2f targetPos = robotState.getPosition() + vector1;
    gateMotor.write(90);//TODO: THIS IS SUPPOSED TO OPEN THE GATE
    state = APPROACH;
  }
}

// TODO make this better and not take 5 seconds
void backupState() {
  setServo3Speed(-100);
  setServo4Speed(-100);
  delay(2000);
  setServo3Speed(50);
  setServo4Speed(-50);
  delay(2000);
  setServo3Speed(-100);
  setServo4Speed(-100);
  delay(2000);
  state = ROTATE;

}

void standbyState() {
  servo3.writeMicroseconds(1500);
  servo4.writeMicroseconds(1500);
  digitalWrite(INDICATOR_PIN, HIGH);
}


void approachState() {
 Vector2f currentPos = robotState.getPosition();
 if (distanceBetween(currentPos, targetPos) < 20) {
    gateMotor.write(0); // TODO: FIGURE OUT IOF THIS IS THE RIGHT POSITION
    delay(1000)
    servo3.writeMicroseconds(1500);
    servo4.writeMicroseconds(1500);
    state = STANDBY
  } else {
    setServo3Speed(100);
    setServo4Speed(100);
  }
 }





////////////////////// ADDITIONAL HELPER FUNCTIONS ////////////////////////////////
int setServo3Speed(int speed) {
  int servo3Speed = minSpeed(speed, MIN_MOTOR_SPEED);
  servo3.writeMicroseconds(1500 - servo3Speed);
  return servo3Speed;
}

int setServo4Speed(int speed) {
  int servo4Speed = minSpeed(speed, MIN_MOTOR_SPEED);
  servo4.writeMicroseconds(1500 + servo4Speed);
  return servo4Speed;
}

void IRAM_ATTR leftLimitCallback() {
  state = BACKUP;
}
