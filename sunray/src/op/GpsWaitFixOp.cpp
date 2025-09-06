// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)

#include "op.h"
#include <Arduino.h>
#include "../../robot.h"
#include "../../map.h"

String GpsWaitFixOp::name(){
    return "GpsWaitFix";
}

void GpsWaitFixOp::begin()
{
    CONSOLE.println("WARN: no gps solution!");
    stateSensor = SENS_GPS_INVALID;
    
    motor.setLinearAngularSpeed(0, 0, LINEAR_ACCELERATION, ANGULAR_ACCELERATION);
    motor.setMowState(false);     
}


void GpsWaitFixOp::end(){
}

void GpsWaitFixOp::run()
{
    motor.setLinearAngularSpeed(0, 0, LINEAR_ACCELERATION, ANGULAR_ACCELERATION);
      
    battery.resetIdle();
    if (gps.solution == SOL_FIXED)
        changeOp(*nextOp);
}


