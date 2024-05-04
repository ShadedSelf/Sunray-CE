// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)


#include "LineTracker.h"
#include "robot.h"
#include "StateEstimator.h"
#include "helper.h"
#include "pid.h"
#include "src/op/op.h"

float stanleyTrackingNormalK = STANLEY_CONTROL_K_NORMAL;
float stanleyTrackingNormalP = STANLEY_CONTROL_P_NORMAL;    
float stanleyTrackingSlowK = STANLEY_CONTROL_K_SLOW;
float stanleyTrackingSlowP = STANLEY_CONTROL_P_SLOW;    

float setSpeed = 0.1; // linear speed (m/s)
bool stateKidnapped = false;
bool printmotoroverload = false;


// control robot velocity (linear,angular) to track line to next waypoint (target)
// uses a stanley controller for line tracking
// https://medium.com/@dingyan7361/three-methods-of-vehicle-lateral-control-pure-pursuit-stanley-and-mpc-db8cc1d32081
void trackLine(bool runControl){  

  bool targetReached =
    maps.distanceToTargetPoint(stateX, stateY) < (SMOOTH_CURVES ? 0.2 : TARGET_REACHED_TOLERANCE);

  if (targetReached){
    activeOp->onTargetReached();
    if (!maps.nextPoint(false,stateX,stateY))   
      activeOp->onNoFurtherWaypoints(); // finish    
  }  
  
  Point target = maps.targetPoint;
  Point lastTarget = maps.lastTargetPoint;

  //float targetDelta = pointsAngle(lastTarget.x(), lastTarget.y(), target.x(), target.y());   
  float targetDelta = pointsAngle(stateX, stateY, target.x(), target.y());    
  if (maps.trackReverse) targetDelta = scalePI(targetDelta + PI);
  targetDelta = scalePIangles(targetDelta, stateDelta);
  
  float trackerDiffDelta = distancePI(stateDelta, targetDelta);                         
  lateralError = distanceLineInfinite(stateX, stateY, lastTarget.x(), lastTarget.y(), target.x(), target.y());        
  
  float distToPath = distanceLine(stateX, stateY, lastTarget.x(), lastTarget.y(), target.x(), target.y());        
  float targetDist = maps.distanceToTargetPoint(stateX, stateY);
  float lastTargetDist = maps.distanceToLastTargetPoint(stateX, stateY);  

  // allow rotations only near last or next waypoint or if too far away from path
  bool angleToTargetFits = true;
  if ( (targetDist < 0.25) || (lastTargetDist < 0.25) || (fabs(distToPath) > 0.25) ) {
    if (SMOOTH_CURVES) angleToTargetFits = (fabs(trackerDiffDelta)/PI*180.0 < 120);
    else               angleToTargetFits = (fabs(trackerDiffDelta)/PI*180.0 < 20);
  }
  else
    angleToTargetFits = true;


  float linear = 0.0;  
  float angular = 0.0; 
  //linear *= (PI - fabs(trackerDiffDelta)) / PI;
  //linear *= max(cos(trackerDiffDelta), 0.0);
  //linear *= cos(trackerDiffDelta * 0.5);

  // rover angle too far away from target angle, rotate rover
  if (!angleToTargetFits){
    linear = 0;
    angular = 29.0 / 180.0 * PI * 1.25; //  29 degree/s (0.5 rad/s);                    
    if (trackerDiffDelta < 0) angular *= -1.0;
  } 
  else // line control (stanley)    
  {
    bool straight = maps.nextPointIsStraight();
    bool trackslow_allowed = true;

    // in case of docking or undocking - check if trackslow is allowed
    if ( maps.isUndocking() || maps.isDocking() ) {
        float dockX = 0;
        float dockY = 0;
        float dockDelta = 0;
        maps.getDockingPos(dockX, dockY, dockDelta);
        float dist_dock = distance(dockX, dockY, stateX, stateY);
        // only allow trackslow if we are near dock (below DOCK_UNDOCK_TRACKSLOW_DISTANCE)
        if (dist_dock > DOCK_UNDOCK_TRACKSLOW_DISTANCE)
            trackslow_allowed = false;
    }

    // planner forces slow tracking (e.g. docking etc)
    if (maps.trackSlow && trackslow_allowed)
      linear = 0.1;   
    // reduce speed when approaching/leaving waypoints         
    else if (setSpeed > 0.2 
      && (targetDist < 0.25 && (!straight)) //approaching
      || (lastTargetDist < 0.25))           //leaving  
        linear = 0.15;         
    else
    {
      if (gps.solution == SOL_FLOAT) linear = min(setSpeed, 0.1); // reduce speed for float solution
      else if (sonar.nearObstacle()) linear = 0.1; // slow down near obstacles
      else                           linear = setSpeed;         // desired speed
    }     

    // slow down speed in case of overload and overwrite all prior speed 
    if (motor.motorLeftOverload || motor.motorRightOverload || motor.motorMowOverload)
    {
      if (!printmotoroverload)
          CONSOLE.println("motor overload detected: reduce linear speed to 0.1");
      printmotoroverload = true;
      linear = 0.1;  
    } 
    else
      printmotoroverload = false; 
          
    // correct for path errors 
    float k = stanleyTrackingNormalK; // STANLEY_CONTROL_K_NORMAL;
    float p = stanleyTrackingNormalP; // STANLEY_CONTROL_P_NORMAL;        
    angular = p * trackerDiffDelta + atan(k * lateralError); // correct for path errors       
    angular = max(-PI/4, min(PI/4, angular));
    
    if (maps.trackReverse) linear *= -1;   // reverse line tracking needs negative speed 
    //linear *= pow( max(cos(trackerDiffDelta), 0.0), 5.0);
  }
  
  // check some pre-conditions that can make linear+angular speed zero
  if (fixTimeout != 0)
    if (millis() > lastFixTime + fixTimeout * 1000.0)
      activeOp->onGpsFixTimeout();            

  if ((gps.solution == SOL_FIXED) || (gps.solution == SOL_FLOAT))       
    if (abs(linear) > 0.06)
      if ((millis() > linearMotionStartTime + 5000) && (stateGroundSpeed < 0.03))
        // if in linear motion and not enough ground speed => obstacle
        if (GPS_SPEED_DETECTION) {         
          CONSOLE.println("gps no speed => obstacle!");
          triggerObstacle();
          return; }
  else { // no gps solution
    if (REQUIRE_VALID_GPS && millis() > lastFixTime + INVALID_GPS_TIMEOUT * 1000.0 && !maps.isDocking()){
      CONSOLE.println("WARN: no gps solution!");
      activeOp->onGpsNoSignal();
    }
  }

  // gps-jump/false fix check
  if (KIDNAP_DETECT){
    float allowedPathTolerance = KIDNAP_DETECT_ALLOWED_PATH_TOLERANCE;     
    if ( maps.isUndocking() || maps.isDocking() ) {
        float dockX = 0;
        float dockY = 0;
        float dockDelta = 0;
        maps.getDockingPos(dockX, dockY, dockDelta);
        float dist = distance(dockX, dockY, stateX, stateY);
        // check if current distance to docking station is below
        if (dist < KIDNAP_DETECT_DISTANCE_DOCK_UNDOCK)
            allowedPathTolerance = KIDNAP_DETECT_ALLOWED_PATH_TOLERANCE_DOCK_UNDOCK;
    }
    if (fabs(distToPath) > allowedPathTolerance){ // actually, this should not happen (except on false GPS fixes or robot being kidnapped...)
      if (!stateKidnapped){
        stateKidnapped = true;
        activeOp->onKidnapped(stateKidnapped);
      }            
    }
    else {
      if (stateKidnapped) {
        stateKidnapped = false;
        activeOp->onKidnapped(stateKidnapped);        
      }
    }
  }

  bool mow = true;
  if (stateOp == OP_DOCK || detectLift())
    mow = false;

  // wait until mowing motor is running
  if (mow && millis() < motor.motorMowSpinUpTime + 3000) { 
    if (!buzzer.isPlaying())
      buzzer.sound(SND_WARNING, true);
    linear = 0;
    angular = 0;     
  }

  if (runControl){
    motor.setLinearAngularSpeed(linear, angular);      
    motor.setMowState(mow);    
  }
}


