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
Point last_rotation_target;
bool rotateLeft = false;
bool rotateRight = false;
bool stateKidnapped = false;
bool printmotoroverload = false;

int get_turn_direction_preference() {
  Point target = maps.targetPoint;
  float targetDelta = pointsAngle(stateX, stateY, target.x(), target.y());
  float r = (MOWER_SIZE / 100);
  float cur_angle = stateDelta;

  if (FREEWHEEL_IS_AT_BACKSIDE) {
	  cur_angle = scalePI(stateDelta + PI);
	  targetDelta = scalePI(targetDelta + PI);
  }

  int right = 0;
  int left = 0;
  int samples = 16;
  for(int i = 1; i < samples; ++i)
  {
    float a = 2.0 * PI / (float)samples * (float)i;
    float px = stateX + cos(a) * r;
    float py = stateY + sin(a) * r;
    float angle = pointsAngle(stateX, stateY, px, py);

    if (!maps.isInsidePerimeter(px, py)) {
      // skip points in front of us
      if (fabs(angle-cur_angle) < 0.05)
        continue;

      if (cur_angle < targetDelta) {
        if (angle >= cur_angle && angle <= targetDelta)
          left++;
        else
          right++;
      } else {
        if (angle <= cur_angle && angle >= targetDelta)
          right++;
        else
          left++;
      }
    
      /*float a = scalePI(cur_angle + 2.0 * (PI / (float)samples * (float)i));
      float px = stateX + cos(a) * r;
      float py = stateY + sin(a) * r;
      //float angle = pointsAngle(stateX, stateY, px, py);

      if (a > targetDelta && maps.checkpoint(px, py))
        right++;
      if (a <= targetDelta && maps.checkpoint(px, py))
        left++;*/
    }
  }

  if (right < left)
    return 1;
  if (left < right)
    return -1;

  return 0;
}

// control robot velocity (linear,angular) to track line to next waypoint (target)
// uses a stanley controller for line tracking
// https://medium.com/@dingyan7361/three-methods-of-vehicle-lateral-control-pure-pursuit-stanley-and-mpc-db8cc1d32081
void trackLine(bool runControl){  

  bool targetReached =
    maps.distanceToTargetPoint(stateX, stateY) < (SMOOTH_CURVES ? 0.2 : TARGET_REACHED_TOLERANCE);

  if (targetReached){
    rotateLeft = false;
    rotateRight = false;
    activeOp->onTargetReached();
    if (!maps.nextPoint(false,stateX,stateY)){
      // finish        
      activeOp->onNoFurtherWaypoints();      
    }
  }  
  
  Point target = maps.targetPoint;
  Point lastTarget = maps.lastTargetPoint;

  float linear = 0.0;  
  float angular = 0.0; 
  bool mow = true;
  if (stateOp == OP_DOCK) mow = false;

  float targetDelta = pointsAngle(stateX, stateY, target.x(), target.y());      
  if (maps.trackReverse) targetDelta = scalePI(targetDelta + PI);
  targetDelta = scalePIangles(targetDelta, stateDelta);
  float trackerDiffDelta = distancePI(stateDelta, targetDelta);                         
  lateralError = distanceLineInfinite(stateX, stateY, lastTarget.x(), lastTarget.y(), target.x(), target.y());        
  
  float distToPath = distanceLine(stateX, stateY, lastTarget.x(), lastTarget.y(), target.x(), target.y());        
  float targetDist = maps.distanceToTargetPoint(stateX, stateY);
  float lastTargetDist = maps.distanceToLastTargetPoint(stateX, stateY);  

  if ( (last_rotation_target.x() != target.x() || last_rotation_target.y() != target.y()) &&
        (rotateLeft || rotateRight ) ) {
    // CONSOLE.println("reset left / right rot (target point changed)");
    rotateLeft = false;
    rotateRight = false;
  }

  // allow rotations only near last or next waypoint or if too far away from path
  // it might race between rotating mower and targetDist check below
  // if we race we still have rotateLeft or rotateRight true
  bool angleToTargetFits = true;
  if ( (targetDist < 0.25) || (lastTargetDist < 0.25) || (fabs(distToPath) > 0.25) ||
       rotateLeft || rotateRight ) {
    if (SMOOTH_CURVES)
      angleToTargetFits = (fabs(trackerDiffDelta)/PI*180.0 < 120);
    else     
      angleToTargetFits = (fabs(trackerDiffDelta)/PI*180.0 < 20);
  } else {
    // while tracking the mowing line do allow rotations if angle to target increases (e.g. due to gps jumps)
    angleToTargetFits = (fabs(trackerDiffDelta)/PI*180.0 < 45);  
  }

  // rover angle too far away from target angle, rotate rover
  if (!angleToTargetFits){
    // angular control (if angle to far away, rotate to next waypoint)
    linear = 0;
    angular = 29.0 / 180.0 * PI * 1.25; //  29 degree/s (0.5 rad/s);               
    if ((!rotateLeft) && (!rotateRight)){ // decide for one rotation direction (and keep it)
      int r = 0;
      // no idea but don't work in reverse mode...
      if (!maps.trackReverse)
        r = get_turn_direction_preference();
      // store last_rotation_target point
      last_rotation_target.setXY(target.x(), target.y());
      
      if      (r == 1) rotateRight = true;
      else if (r == -1) rotateLeft = true;
      else if (trackerDiffDelta < 0)
        rotateRight = true;
      else
        rotateLeft = true;
    }        
    if (rotateRight) angular *= -1.0;
  } 
  else {
    // line control (stanley)    
    bool straight = maps.nextPointIsStraight();
    bool trackslow_allowed = true;

    rotateLeft = false;
    rotateRight = false;

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
    else if (
      (setSpeed > 0.2) 
      && (targetDist < 0.25 && (!straight)) //approaching
      || (lastTargetDist < 0.25))           //leaving  
        linear = 0.15;         
    else {
      if (gps.solution == SOL_FLOAT)        
        linear = min(setSpeed, 0.1); // reduce speed for float solution
      else
        linear = setSpeed;         // desired speed
      if (sonar.nearObstacle())
        linear = 0.1; // slow down near obstacles
    }     

    // slow down speed in case of overload and overwrite all prior speed 
    if ( (motor.motorLeftOverload) || (motor.motorRightOverload) || (motor.motorMowOverload) ){
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
    /*if (maps.trackSlow && trackslow_allowed) {
      k = stanleyTrackingSlowK; //STANLEY_CONTROL_K_SLOW;   
      p = stanleyTrackingSlowP; //STANLEY_CONTROL_P_SLOW;          
    }*/         
    //angular = p * trackerDiffDelta + atan(k * lateralError); // correct for path errors
    angular = p * trackerDiffDelta + atan(k * lateralError * fabs(motor.linearSpeedSet)); // correct for path errors       
    angular = max(-PI/4, min(PI/4, angular));
    if (maps.trackReverse) linear *= -1;   // reverse line tracking needs negative speed 
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

  // in any case, turn off mower motor if lifted 
  // also, if lifted, do not turn on mowing motor so that the robot will drive and can do obstacle avoidance 
  if (detectLift()) mow = false;
   
  if (mow) { 
    if (millis() < motor.motorMowSpinUpTime + 3000){
       // wait until mowing motor is running
      if (!buzzer.isPlaying()) buzzer.sound(SND_WARNING, true);
      linear = 0;
      angular = 0;     
    }
  }

  if (runControl){
    motor.setLinearAngularSpeed(linear, angular);      
    motor.setMowState(mow);    
  }
}


