// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)

#include "op.h"
#include <Arduino.h>
#include "../../robot.h"
#include "../../StateEstimator.h"
#include "../../map.h"



String EscapeReverseOp::name(){
    return "EscapeReverse";
}

void EscapeReverseOp::begin()
{
    // obstacle avoidance
    driveReverseStopTime = millis() + 3000;

    vec3_t pos = position - forward * 0.2 * (maps.trackReverse ? -1.0 : 1.0);
    drive = maps.isInsidePerimeter(pos.x, pos.y);

    if (DISABLE_MOW_MOTOR_AT_OBSTACLE) 
        motor.setMowState(false);                         
}

void EscapeReverseOp::run()
{
    battery.resetIdle();

    if (drive)
        motor.setLinearAngularSpeed(-0.1 * (maps.trackReverse ? -1.0 : 1.0), 0, LINEAR_ACCELERATION);                                   

    if (millis() > driveReverseStopTime)
    {
        motor.stopImmediately(false); 
        driveReverseStopTime = 0;

        if (!maps.isDocking())
            maps.addObstacle(position.x, position.y);              

        changeOp(*nextOp, false);    // continue current operation
    }
}

void EscapeReverseOp::end(){
}



void EscapeReverseOp::onImuTilt(){
    stateSensor = SENS_IMU_TILT;
    changeOp(errorOp);
}

void EscapeReverseOp::onImuError(){
    stateSensor = SENS_IMU_TIMEOUT;
    changeOp(errorOp);
}


