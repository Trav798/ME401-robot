#pragma once

#include "common.h"

void setupDCMotors();

#ifdef DCMOTOR1
void setSetpoint1(double setpoint);
void setPIDgains1(double kp, double ki, double kd);
double getSetpoint1(void);
double getPosition1(void);
double getError1(void);
double getOutput1(void);
#endif

#ifdef DCMOTOR2
void setSetpoint2(double setpoint);
void setPIDgains2(double kp, double ki, double kd);
double getSetpoint2(void);
double getPosition2(void);
double getError2(void);
double getOutput2(void);
#endif