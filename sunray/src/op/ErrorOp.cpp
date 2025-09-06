// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)

#include "op.h"
#include <Arduino.h>
#include "../../robot.h"
#include "../../map.h"
#include "../../comm.h"


String ErrorOp::name(){
    return "Error";
}

void ErrorOp::begin()
{
    CONSOLE.println("OP_ERROR"); 
     
    //motor.stopImmediately(true);
    motor.setLinearAngularSpeed(0, 0, LINEAR_ACCELERATION, ANGULAR_ACCELERATION);
    motor.setMowState(false);      
  
    buzzer.sound(SND_ERROR, true);
}

void ErrorOp::end(){
}

void ErrorOp::run()
{
    motor.setLinearAngularSpeed(0, 0, LINEAR_ACCELERATION, ANGULAR_ACCELERATION);

    if (battery.chargerConnected())  
        changeOp(chargeOp);

    // Switch off if too much time in error OP
    if (SWITH_OFF_AT_ERROR && millis() - startTime > SWITH_OFF_AT_ERROR_TIME * 60 * 1000)
        cmdSwitchOffRobot();
}


