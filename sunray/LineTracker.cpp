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
#include "Stats.h"

float stanleyTrackingNormalK = STANLEY_CONTROL_K_NORMAL;
float stanleyTrackingNormalP = STANLEY_CONTROL_P_NORMAL;    
float stanleyTrackingSlowK = 0;
float stanleyTrackingSlowP = 0;    

float setSpeed = 0.1; // linear speed (m/s)
Timer stanleyTimer = Timer(MICROS_TIME);

bool isNearPerimeter()
{
  bool isInside = true;

  vec3_t rr = position + right * 0.1;
  vec3_t ll = position - right * 0.1;

  isInside = isInside && maps.isInsidePerimeter(rr.x, rr.y);
  isInside = isInside && maps.isInsidePerimeter(ll.x, ll.y);

  return !isInside && maps.wayMode != WAY_DOCK;   
}

bool isCloseToDock(float dist = 0)
{
  float dockX = 0;
  float dockY = 0;
  float dockDelta = 0;
  maps.getDockingPos(dockX, dockY, dockDelta);
  float dist_dock = distance(dockX, dockY, position.x, position.y);

  // default value
  if (dist < 0.001)
    dist = DOCK_UNDOCK_TRACKSLOW_DISTANCE;

  return dist_dock < dist;// && (maps.isUndocking() || maps.isDocking());
}

bool shouldUseRearTraction(vec3_t target, vec3_t targetDir)
{
  if (maps.isDocking())         // forward traction if docking
    return false;
  else if (maps.isUndocking())  // rear traction if undocking
    return true;
  else
  {
    // exceptions
    if (distance(target, vec3_t(-1.86, -2.01)) < 0.1)
      return true;

    // perimeter
    vec3_t td  = targetDir.norm();
    vec3_t tds = vec3_t(-td.y, td.x);

    vec3_t rf = target + td  * 0.1;
    vec3_t lb = target - td  * 0.1;
    vec3_t rr = target + tds * 0.1;
    vec3_t ll = target - tds * 0.1;

    bool isInside = true;
    isInside = isInside && maps.isInsidePerimeter(rr.x, rr.y);
    isInside = isInside && maps.isInsidePerimeter(ll.x, ll.y);
    isInside = isInside && maps.isInsidePerimeter(rf.x, rf.y);
    isInside = isInside && maps.isInsidePerimeter(lb.x, lb.y);

    // rear traction if not driving into perimeter
    return isInside;
  }
}

vec3_t lastStuckPos = {0};
float lastStuckRot = 0.0;
unsigned long lastStuckTime = 0;
unsigned long stuckCounter = 0;
bool isStuck()
{
  unsigned long now = millis();

  // wait for motor to spin up before checking for obstacles
  if (millis() - motor.motorMowSpinUpTime < 2000)
  {
    lastStuckTime = now;
    return false;
  }

  // check every 3 seconds for lack of movement
  if (now - lastStuckTime > 3000)
  {
    float dist = (position-lastStuckPos).mag();
    float rot = fabs(distancePI(imuDriver.yaw, lastStuckRot));

    // no linear or angular movement
    if (gps.solution == SOL_FIXED
    && dist < 0.05
    && rot < radians(10.0))
    {
      stuckCounter++;
      
      statMowGPSNoSpeedCounter++;
      triggerObstacle();
    }
    else
      stuckCounter = 0;

    lastStuckPos = position;
    lastStuckRot = imuDriver.yaw;
    lastStuckTime = now;
  }

  // if no movement for 1 minute return true
  if (stuckCounter >= 20)
  {
    stuckCounter = 0;
    return true;
  }
  return false;
}

bool shouldRotateInPlace(float trackerDiffDelta)
{
    return fabs(trackerDiffDelta) > radians(10.0); // too much angular error
}

// control robot velocity (linear,angular) to track line to next waypoint (target)
// uses a stanley controller for line tracking
// https://medium.com/@dingyan7361/three-methods-of-vehicle-lateral-control-pure-pursuit-stanley-and-mpc-db8cc1d32081
float lastTargetHeading = 0.0;
bool stillRotation      = false;
bool decelerateLinear   = false;
bool decelerateAngular  = false;
void trackLine(bool runControl)
{
  vec3_t target     = vec3_t(maps.targetPoint.x(),     maps.targetPoint.y()    );
  vec3_t lastTarget = vec3_t(maps.lastTargetPoint.x(), maps.lastTargetPoint.y());

  vec3_t targetDir = target - lastTarget;
  vec3_t currDir   = target - position;
  float dirDot     = dot(currDir, targetDir);
  
  // rear traction
  maps.trackReverse = shouldUseRearTraction(target, targetDir);
  
  // -- Calculate variables --
  float distToPath     = distanceLine(position.x, position.y, lastTarget.x, lastTarget.y, target.x, target.y);        
  float targetDist     = distance(position, target);
  float lastTargetDist = distance(position, lastTarget);
  
  float targetDelta = angleInterpolation(
    pointsAngle(lastTarget.x, lastTarget.y, target.x, target.y) * sign(dirDot),
    pointsAngle(position.x,    position.y,  target.x, target.y),
    constrain(targetDist - 0.1, 0.0, 1.0)
  );
  if (maps.trackReverse) targetDelta = scalePI(targetDelta + PI);
  targetDelta = scalePIangles(targetDelta, heading);
  
  float trackerDiffDelta = distancePI(heading, targetDelta);

  lateralError = distanceLineInfinite(position.x, position.y, lastTarget.x, lastTarget.y, target.x, target.y);        

  // not rotating and deviated from target heading
  if (!stillRotation && fabs(trackerDiffDelta) > radians(20.0))
  {
    stillRotation = true;
    lastTargetHeading = heading;
  }


  float linear = 0.0;  
  float angular = 0.0;

  // is still rotation roquired
  // otherwise line control (stanley)
  if (stillRotation)
  {
    angular = 0.9 * sign(trackerDiffDelta);

    // angular deceleration
    float stoppingDistance = 0.5 * powf(motor.angularSpeedSet, 2) / ANGULAR_ACCELERATION;

    if (!decelerateAngular && fabs(trackerDiffDelta) <= stoppingDistance)
      decelerateAngular = true;
      
    if (decelerateAngular)
      angular = 0.0;

    // stop
    if (fabs(motor.angularSpeedSet) < 0.0001)
    {
      stillRotation     = false;
      decelerateAngular = false;
    }
  } 
  else
  {
     // desired speed
    linear = setSpeed;

    // linear speed modifiers
    if (motor.motorLeftOverload || motor.motorRightOverload || motor.motorMowOverload)
                                                  linear = min(0.1, linear);
    if (maps.trackSlow && isCloseToDock())        linear = min(0.1, linear);  // planner forces slow tracking (e.g. docking etc)                        
    if (gps.solution == SOL_FLOAT)                linear = min(0.1, linear);  // slown down for float solution
    if (sonar.nearObstacle())                     linear = min(0.1, linear);  // slow down near obstacles
    if (maps.isDocking())                         linear = min(0.15, linear);
    if (SET_PERIMETER_SPEED && isNearPerimeter()) linear = min(PERIMETER_SPEED, linear); // set speed near perimeter

    if (ADAPTIVE_SPEED && !maps.trackReverse && maps.wayMode != WAY_DOCK)
    {
      float minCurrent = ADAPTIVES_SPEED_MINCURRENT;
      float maxCurrent = ADAPTIVES_SPEED_MAXCURRENT;
      float diffCurrent = maxCurrent - minCurrent;

      float minSpeed = 0.1;

      float t = (motor.motorMowSenseFLP - minCurrent) / diffCurrent;
      float aSpeed = lerp(setSpeed, minSpeed, constrain(t, 0.0, 1.0));
      linear = min(aSpeed, linear);
    }


    // correct for path errors 
    float k = stanleyTrackingNormalK;
    float p = stanleyTrackingNormalP;        
    angular = p * trackerDiffDelta + atan(k * lateralError);  // correct for path errors   
    angular = constrain(angular, -PI/4.0, PI/4.0);            // constrain rotation
    angular *= linear / setSpeed;                             // make angular proportional to linear
    
    linear *= exp(-trackerDiffDelta * trackerDiffDelta * HEADING_ERROR_SPEED_FACTOR); // slow down on heading error
    if (maps.trackReverse)
      linear *= -1.0;   // reverse line tracking needs negative speed
    else if (!maps.isDocking())
      linear *= 1.0 - constrain(-imuDriver.pitch / (PI*0.5) * 4.0 - 0.1, 0.0, 1.0); // slow down on uphill
    
    // linear deceleration
    float stoppingDistance = 0.5 * powf(motor.linearSpeedSet, 2) / LINEAR_ACCELERATION;
    
    if (!decelerateLinear && targetDist <= max(stoppingDistance, 0.01))
      decelerateLinear = true;
      
    if (decelerateLinear)
      linear = 0.0;
  }
  
  // stop OP if last fix is greater than timeout
  if (fixTimeout != 0 && millis() > lastFixTime + fixTimeout * 1000.0 && !isCloseToDock(3.0))
      activeOp->onGpsFixTimeout();
   
  // check if mower is stuck somewhere and stop current OP
  if (isStuck())
  {
    stateSensor = SENS_LIFT;
    activeOp->changeOp(errorOp);
  }

  // no gps solution
  if (gps.solution == SOL_INVALID && REQUIRE_VALID_GPS && millis() > lastFixTime + INVALID_GPS_TIMEOUT * 1000.0
  && !maps.isDocking() && !maps.isUndocking()) // use isCloseToDock instead? 
  {
    CONSOLE.println("WARN: no gps solution!");
    activeOp->onGpsNoSignal();
  }

  // kidnap detect
  if (KIDNAP_DETECT && fabs(distToPath) > KIDNAP_DETECT_ALLOWED_PATH_TOLERANCE && !maps.isInsidePerimeter(position.x, position.y) && !maps.isUndocking())
    activeOp->onKidnapped();


  // -- set speeds --

  bool mow = true;
  if (stateOp == OP_DOCK || detectLift())
    mow = false;

  // wait until mowing motor is running
  if (mow && millis() - motor.motorMowSpinUpTime < 2000)
  { 
    if (!buzzer.isPlaying())
      buzzer.sound(SND_WARNING, true);

    linear = 0;
    angular = 0;     
  }

  if (runControl){
    motor.setLinearAngularSpeed(linear, angular, LINEAR_ACCELERATION, ANGULAR_ACCELERATION);      
    motor.setMowState(mow);    
  }

  // target reached
  if ((decelerateLinear && fabs(motor.linearSpeedSet) < 0.0001) || dirDot <= 0.0)
  {
    activeOp->onTargetReached();

    if (!maps.nextPoint(false, position.x, position.y))   
      activeOp->onNoFurtherWaypoints(); // finish
    else
      stillRotation = shouldRotateInPlace(trackerDiffDelta);  // rotation
     
    lastTargetHeading = heading;
    decelerateLinear = false;
  }
}


