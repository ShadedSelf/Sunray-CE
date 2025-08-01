// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)

#include "op.h"
#include <Arduino.h>
#include "../../robot.h"
#include "../../StateEstimator.h"
#include "../../LineTracker.h"
#include "../../Stats.h"
#include "../../map.h"


MowOp::MowOp(){
    lastMapRoutingFailed = false;
    mapRoutingFailedCounter = 0;
}

String MowOp::name(){
    return "Mow";
}

void MowOp::begin()
{
    DEBUGLN("OP_MOW");   

    bool error = false;
    bool routingFailed = false;      
   
    motor.enableTractionMotors(true); // allow traction motors to operate         
    motor.setLinearAngularSpeed(0,0);      
    if ((previousOp != &escapeReverseOp && previousOp != &escapeForwardOp) || DISABLE_MOW_MOTOR_AT_OBSTACLE)
        motor.setMowState(false);              
    battery.setIsDocked(false);                             

    // plan route to next target point 

    dockOp.dockReasonRainTriggered = false;    

    if ((initiatedByOperator && previousOp == &idleOp) || lastMapRoutingFailed)
        maps.clearObstacles();

    if (maps.startMowing(position.x, position.y)){
        if (maps.nextPoint(true, position.x, position.y)) {
            lastFixTime = millis();                
            maps.setLastTargetPoint(position.x, position.y);        
            motor.setMowState(true);            
        } else {
            error = true;
            DEBUGLN("error: no waypoints!");              
        }
    } else error = true;

    if (error){
        stateSensor = SENS_MAP_NO_ROUTE;
        //op = OP_ERROR;
        routingFailed = true;
        motor.setMowState(false);
    }

    if (routingFailed){
        lastMapRoutingFailed = true; 
        mapRoutingFailedCounter++;    
        if (mapRoutingFailedCounter > 60){
            DEBUGLN("error: too many map routing errors!");
            stateSensor = SENS_MAP_NO_ROUTE;
            changeOp(errorOp);      
        } else {    
        changeOp(gpsRebootRecoveryOp, true);
        }
    } else {
        lastMapRoutingFailed = false;
        mapRoutingFailedCounter = 0;
    }
}


void MowOp::end(){
}

void MowOp::run()
{
    if (!detectObstacle())
        detectObstacleRotation();                              
     
    // line tracking
    trackLine(true); 
    detectSensorMalfunction();    
    battery.resetIdle();
    
    if (timetable.shouldAutostopNow())
        onTimetableStopMowing();
}

void MowOp::onTempOutOfRangeTriggered(){
    if (!DOCKING_STATION)
        return;

    DEBUGLN("TEMP OUT-OF-RANGE TRIGGERED");

    stateSensor = SENS_TEMP_OUT_OF_RANGE;
    dockOp.dockReasonRainTriggered = true;
    dockOp.dockReasonRainAutoStartTime = millis() + 60000 * 60 * TEMP_DOCK_TIME; // try again after one hour      
    dockOp.setInitiatedByOperator(false);
    changeOp(dockOp);
}

void MowOp::onBatteryLowShouldDock(){    
    DEBUGLN("BATTERY LOW TRIGGERED - DOCKING");

    dockOp.setInitiatedByOperator(false);
    changeOp(dockOp);
}

void MowOp::onTimetableStopMowing()
{
    DEBUGLN("TIMETABLE STOP");

    if (DOCKING_STATION)
    {
        dockOp.setInitiatedByOperator(false);
        changeOp(dockOp);
    }
    else
        changeOp(idleOp);   
}

void MowOp::onTimetableStartMowing(){        
}

void MowOp::onObstacle(){
    DEBUGLN("triggerObstacle");      

    statMowObstacles++;      

    if (maps.isDocking() && maps.retryDocking(position.x, position.y)) {
        changeOp(escapeReverseOp, true);                      
        return;
    }

    if (OBSTACLE_AVOIDANCE && maps.wayMode != WAY_DOCK)
        changeOp(escapeReverseOp, true);
    else {     
        DEBUGLN("error: obstacle!");         

        stateSensor = SENS_OBSTACLE;
        changeOp(errorOp);                
    }
}
    
void MowOp::onObstacleRotation(){
    DEBUGLN("triggerObstacleRotation");    

    statMowObstacles++;   

    if (OBSTACLE_AVOIDANCE && maps.wayMode != WAY_DOCK){    
        if (FREEWHEEL_IS_AT_BACKSIDE)  
            changeOp(escapeForwardOp, true);      
        else
            changeOp(escapeReverseOp, true);
    }
    else { 
        DEBUGLN("error: obstacle!"); 
                   
        stateSensor = SENS_OBSTACLE;
        changeOp(errorOp);
    }
}


void MowOp::onOdometryError(){
    if (!ENABLE_ODOMETRY_ERROR_DETECTION)
        return;

    DEBUGLN("error: odometry error!");
    stateSensor = SENS_ODOMETRY_ERROR;
    changeOp(errorOp);
}
    
void MowOp::onMotorOverload(){
    if (!ENABLE_OVERLOAD_DETECTION)
        return;
  
    if (motor.motorOverloadDuration > 20000){
        DEBUGLN("error: motor overload!"); 
        stateSensor = SENS_OVERLOAD;
        changeOp(errorOp);
    }
}
    
void MowOp::onMotorError(){
    if (!ENABLE_FAULT_OBSTACLE_AVOIDANCE)
    {
        DEBUGLN("no obstacle avoidance activated on motor errors, giving up");

        stateSensor = SENS_MOTOR_ERROR;
        changeOp(errorOp);        
        return;
    }


    if (motor.motorError){
        // this is the molehole situation: motor error will permanently trigger on molehole => we try obstacle avoidance (molehole avoidance strategy)
        motor.motorError = false; // reset motor error flag
        motorErrorCounter++;

        DEBUG("MowOp::onMotorError motorErrorCounter=");       
        DEBUGLN(motorErrorCounter);

        if (maps.wayMode != WAY_DOCK && motorErrorCounter < 5){
            changeOp(escapeReverseOp, true);     // trigger obstacle avoidance 
            return;
        }

        // obstacle avoidance failed with too many motor errors (it was probably not a molehole situation)
        DEBUGLN("error: motor error - giving up!");
        motorErrorCounter = 0;
        stateSensor = SENS_MOTOR_ERROR;
        changeOp(errorOp);
        return;      
    }
}

void MowOp::onTargetReached(){
    if (maps.wayMode == WAY_MOW){    
        maps.clearObstacles(); // clear obstacles if target reached
        motorErrorCounter = 0; // reset motor error counter if target reached
        stateSensor = SENS_NONE; // clear last triggered sensor
    }
}


void MowOp::onGpsFixTimeout(){
    // no gps solution
    if (REQUIRE_VALID_GPS){
#ifdef UNDOCK_IGNORE_GPS_DISTANCE
        if (!maps.isUndocking() || maps.getDockDistance() > UNDOCK_IGNORE_GPS_DISTANCE){
#else
        if (!maps.isUndocking()){
#endif
            stateSensor = SENS_GPS_FIX_TIMEOUT;
            changeOp(gpsWaitFixOp, true);
        }
    }
}

void MowOp::onGpsNoSignal(){
    if (REQUIRE_VALID_GPS){
#ifdef UNDOCK_IGNORE_GPS_DISTANCE
        if (!maps.isUndocking() || maps.getDockDistance() > UNDOCK_IGNORE_GPS_DISTANCE){
#else
        if (!maps.isUndocking()){
#endif
            stateSensor = SENS_GPS_INVALID;
            changeOp(gpsWaitFloatOp, true);
        }
    }
}

void MowOp::onKidnapped(bool state){
    if (state){
        stateSensor = SENS_KIDNAPPED;      
        motor.setLinearAngularSpeed(0,0, false); 
        motor.setMowState(false);    
        changeOp(kidnapWaitOp, true); 
    }
}

void MowOp::onNoFurtherWaypoints(){
    DEBUGLN("mowing finished!");

    if (!finishAndRestart)
    {             
        if (DOCKING_STATION){
            dockOp.setInitiatedByOperator(false);
            changeOp(dockOp);               
        } else {
            idleOp.setInitiatedByOperator(false);
            changeOp(idleOp); 
        }
    }
}

void MowOp::onImuTilt(){
    stateSensor = SENS_IMU_TILT;
    changeOp(errorOp);
}

void MowOp::onImuError(){
    stateSensor = SENS_IMU_TIMEOUT;
    changeOp(errorOp);
}

