// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)

#include "op.h"
#include <Arduino.h>
#include "../../robot.h"
#include "../../StateEstimator.h"
#include "../../map.h"
#include "../../helper.h"


String ChargeOp::name(){
  return "Charge";
}


void ChargeOp::begin(){
  nextConsoleDetailsTime = 0;
  retryTouchDock = false;
  betterTouchDock = false;

  CONSOLE.print("OP_CHARGE");
  CONSOLE.print(" dockOp.initiatedByOperator=");
  CONSOLE.print(dockOp.initiatedByOperator);
  CONSOLE.print(" dockReasonRainTriggered=");
  DEBUGLN(dockOp.dockReasonRainTriggered);

  //motor.stopImmediately(true); // do not use PID to get to stop 
  motor.setLinearAngularSpeed(0, 0, false); 
  motor.setMowState(false);     
  //motor.enableTractionMotors(false); // keep traction motors off (motor drivers tend to generate some incorrect encoder values when stopped while not turning)
  
  buzzer.sound(SND_PLUG, true);
}


void ChargeOp::end(){

}

void ChargeOp::run()
{
  if (retryTouchDock || betterTouchDock)
  {
    if (millis() > retryTouchDockSpeedTime){                            
      retryTouchDockSpeedTime = millis() + 1000;
      motor.enableTractionMotors(true); // allow traction motors to operate                               
      motor.setLinearAngularSpeed(0.1, 0);
    }

    if (retryTouchDock){
      if (millis() > retryTouchDockStopTime) {
        motor.setLinearAngularSpeed(0, 0);
        retryTouchDock = false;
        DEBUGLN("ChargeOp: retryTouchDock failed");
        motor.enableTractionMotors(true); // allow traction motors to operate                               
        maps.setIsDocked(false);
        changeOp(idleOp);    
      }
    } 
    else if (betterTouchDock)
    {
      if (millis() > betterTouchDockStopTime) {
        DEBUGLN("ChargeOp: betterTouchDock completed");
        motor.setLinearAngularSpeed(0, 0);            
        betterTouchDock = false;
      }        
    }
  }
  
  battery.resetIdle();
  
  if (battery.chargerConnected())
  {        
    maps.setIsDocked(true);    

    // get robot position and yaw from docking pos
    // sensing charging contacts means we are in docking station - we use docking point coordinates to get rid of false fix positions in
    // docking station
    float dockHeading;
    maps.getDockingPos(position.x, position.y, dockHeading);
    headingOffset = distancePI(imuDriver.yaw, dockHeading);
                       
    if (battery.chargingHasCompleted())
    {
      if (millis() > nextConsoleDetailsTime)
      {
        nextConsoleDetailsTime = millis() + 30000;
        CONSOLE.print("ChargeOp: charging completed (DOCKING_STATION=");
        CONSOLE.print(DOCKING_STATION);
        CONSOLE.print(", battery.isDocked=");
        CONSOLE.print(battery.isDocked());
        CONSOLE.print(", dockOp.initiatedByOperator=");
        CONSOLE.print(dockOp.initiatedByOperator);        
        CONSOLE.print(", maps.mowPointsIdx=");
        CONSOLE.print(maps.mowPointsIdx);
        CONSOLE.print(", DOCK_AUTO_START=");
        CONSOLE.print(DOCK_AUTO_START);
        CONSOLE.print(", dockOp.dockReasonRainTriggered=");
        CONSOLE.print(dockOp.dockReasonRainTriggered);
        CONSOLE.print(", dockOp.dockReasonRainAutoStartTime(min remain)=");
        CONSOLE.print( ((int)(dockOp.dockReasonRainAutoStartTime - millis())) / 60000 );                                
        CONSOLE.print(", timetable.mowingAllowed=");                
        CONSOLE.print(timetable.mowingAllowed());
        CONSOLE.print(", finishAndRestart=");                
        CONSOLE.print(finishAndRestart);                
        DEBUGLN(")");
      }
    }
  }

  if (timetable.shouldAutostartNow())
    onTimetableStartMowing();

  // reset whole sketch after X days
  if (millis() > 5 * 24 * 60 * 60 * 1000)
    while (true) { }
}

void ChargeOp::onTimetableStopMowing(){        
}

void ChargeOp::onTimetableStartMowing()
{
  DEBUGLN("TIMETABLE START");

  changeOp(mowOp); // continue mowing           
}

void ChargeOp::onChargerDisconnected()
{
  if (DOCKING_STATION && DOCK_RETRY_TOUCH)
  {    
    DEBUGLN("ChargeOp::onChargerDisconnected - retryTouchDock");

    retryTouchDock = true;
    retryTouchDockStopTime = millis() + 5000;
    retryTouchDockSpeedTime = millis();
  } 
  else
  {
    motor.enableTractionMotors(true); // allow traction motors to operate                               
    maps.setIsDocked(false);
    changeOp(idleOp);    
  }
}

void ChargeOp::onBadChargingContactDetected()
{
  if (DOCKING_STATION && DOCK_RETRY_TOUCH)
  {    
    DEBUGLN("ChargeOp::onBadChargingContactDetected - betterTouchDock");

    betterTouchDock = true;
    betterTouchDockStopTime = millis() + 5000;
    retryTouchDockSpeedTime = millis();
  } 
}

void ChargeOp::onChargerConnected(){
  if (retryTouchDock){
    DEBUGLN("ChargeOp: retryTouchDock succeeded");        
    motor.setLinearAngularSpeed(0, 0);
    retryTouchDock = false;
  }
}

void ChargeOp::onBatteryUndervoltage(){    
  stateSensor = SENS_BAT_UNDERVOLTAGE;
}

void ChargeOp::onRainTriggered(){
  if (!DOCKING_STATION)
    return;

  // ensure rain sensor is dry for RAIN_DOCK_TIME hours
  dockOp.dockReasonRainAutoStartTime = millis() + 60000 * 60 * RAIN_DOCK_TIME;
}
