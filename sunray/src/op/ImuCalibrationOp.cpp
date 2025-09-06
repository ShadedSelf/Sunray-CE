// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)

#include "op.h"
#include <Arduino.h>
#include "../../robot.h"
#include "../../map.h"
#include "../../StateEstimator.h"


String ImuCalibrationOp::name(){
    return "ImuCalibration";
}

void ImuCalibrationOp::begin()
{
    motor.setLinearAngularSpeed(0, 0, LINEAR_ACCELERATION, ANGULAR_ACCELERATION);

    nextImuCalibrationSecond = 0;
    imuCalibrationSeconds = 0;
}


void ImuCalibrationOp::end(){

}

void ImuCalibrationOp::run()
{
    battery.resetIdle();

    motor.setLinearAngularSpeed(0, 0, LINEAR_ACCELERATION, ANGULAR_ACCELERATION);
    //motor.stopImmediately(true);   

    if (millis() > nextImuCalibrationSecond)
    {
        nextImuCalibrationSecond = millis() + 1000;  
        imuCalibrationSeconds++;

        CONSOLE.print("IMU gyro calibration (robot must be static)... ");        
        CONSOLE.println(imuCalibrationSeconds);        
        buzzer.sound(SND_PROGRESS, true);
               
        if (imuCalibrationSeconds >= 5)
        {
            // use magnometer heading if regular heading too far at startup
            //if (USE_MAGNOMETER && fabs(distancePI(heading, imuDriver.magYaw)) > radians(30.0))
              //  headingOffset = distancePI(imuDriver.yaw, imuDriver.magYaw);

            imuDriver.resetData();
            resetImuTimeout();
            Op::changeOp(*nextOp);
        }
    }           
}


