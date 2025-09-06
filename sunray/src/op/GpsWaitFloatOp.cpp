// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)

#include "op.h"
#include <Arduino.h>
#include "../../robot.h"
#include "../../map.h"

String GpsWaitFloatOp::name(){
  return "GpsWaitFloat";
}

void GpsWaitFloatOp::begin()
{
    CONSOLE.println("WARN: no gps solution!");
    stateSensor = SENS_GPS_INVALID;

    motor.setLinearAngularSpeed(0, 0, LINEAR_ACCELERATION, ANGULAR_ACCELERATION);
    motor.setMowState(false);     
}


void GpsWaitFloatOp::end(){
}

void GpsWaitFloatOp::run()
{
    motor.setLinearAngularSpeed(0, 0, LINEAR_ACCELERATION, ANGULAR_ACCELERATION);
    
      // reboot gps every 5 min
    if (millis() - startTime > 5 * 60 * 1000)
    {
        startTime = millis();
        gps.reboot();
        // try hard reset if more time passes??
        /*
        the CFG-RST command enables to issue different types of resets:

        COLD_START_SOFT_RESET: (0xFF, 0xFF, 0x01, 0x00)

        HOT_START_SOFT_RESET:  (0x00, 0x00, 0x01, 0x00)

        COLD_START_CONTROLLED_HARD_RESET: (0xFF, 0xFF, 0x04, 0x00)                
        
        HOT_START_CONTROLLED_HARD_RESET:(0x00, 0x00, 0x04, 0x00)

        STOP_GNSS: (0x00, 0x00, 0x08, 0x00)

        START_GNSS: (0x00, 0x00, 0x09, 0x00)*/
    }
    
    battery.resetIdle();
    if (gps.solution == SOL_FIXED || gps.solution == SOL_FLOAT)    
        changeOp(*nextOp);  
}


