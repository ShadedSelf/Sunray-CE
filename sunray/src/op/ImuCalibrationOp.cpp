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

void ImuCalibrationOp::begin(){
    nextImuCalibrationSecond = 0;
    imuCalibrationSeconds = 0;
}


void ImuCalibrationOp::end(){

}

void ImuCalibrationOp::run()
{
    battery.resetIdle();
    motor.stopImmediately(true);   

    if (millis() > nextImuCalibrationSecond)
    {
        nextImuCalibrationSecond = millis() + 1000;  
        imuCalibrationSeconds++;

        CONSOLE.print("IMU gyro calibration (robot must be static)... ");        
        CONSOLE.println(imuCalibrationSeconds);        
        buzzer.sound(SND_PROGRESS, true);
               
        if (imuCalibrationSeconds >= 5)
        {                 
            imuDriver.resetData();
            resetImuTimeout();
            Op::changeOp(*nextOp);
        }
    }           
}


