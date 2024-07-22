// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)


#ifndef STATE_ESTIMATOR_H
#define STATE_ESTIMATOR_H

#include <Arduino.h>

#include "src/math/quaternion_type.h"
#include "src/math/vector_type.h"

extern vec3_t forward;
extern vec3_t right;
extern vec3_t up;

extern vec3_t position;
extern float heading;  // direction (rad)

//extern float stateRoll;
//extern float statePitch;
extern float stateGroundSpeed; // m/s
extern float lateralError; // lateral error

extern double headingOffset;

extern unsigned long lastInvalidTime;
extern unsigned long lastFixJumpTime;

extern unsigned long imuDataTimeout;

bool startIMU(bool forceIMU);
void readIMU();
void computeRobotState();
void resetImuTimeout();

#endif

