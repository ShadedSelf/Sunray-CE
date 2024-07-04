// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)


#ifndef STATE_ESTIMATOR_H
#define STATE_ESTIMATOR_H

/*struct StateData
{
  vec3_t forward;
  vec3_t right;
  vec3_t up;

  vec3_t position;
  float heading;

  float stateGroundSpeed; // m/s
  float lateralError; // lateral error
};*/



#include <Arduino.h>

#include <quaternion_type.h>
#include <vector_type.h>

extern vec3_t forward;
extern vec3_t right;
extern vec3_t up;

extern vec3_t position;
extern float heading;  // direction (rad)

extern float stateRoll;
extern float statePitch;
extern float stateDeltaIMU;
extern float stateGroundSpeed; // m/s
extern float lateralError; // lateral error

extern double headingOffset;

/*extern float stateDeltaLast;
extern float stateDeltaSpeed;
extern float stateDeltaSpeedLP;
extern float stateDeltaSpeedIMU;
extern float stateDeltaSpeedWheels;
extern float diffIMUWheelYawSpeed;
extern float diffIMUWheelYawSpeedLP;*/

extern bool gpsJump;
extern unsigned long lastInvalidTime;
extern unsigned long lastFixJumpTime;

extern bool imuIsCalibrating;
extern unsigned long imuDataTimeout;
extern float lastIMUYaw; 


bool startIMU(bool forceIMU);
void readIMU();
void computeRobotState();
void resetImuTimeout();

#endif

