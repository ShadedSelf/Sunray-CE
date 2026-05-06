// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)

#include <Arduino.h>
#include "StateEstimator.h"
#include "src/op/op.h"

#include "config.h"
#include "robot.h"
#include "Stats.h"
#include "helper.h"
#include "i2c.h"
#include "gps.h"

#include "src/math/quaternion_type.h"
#include "src/math/vector_type.h"


vec3_t forward = {1, 0, 0};
vec3_t right   = {0,-1, 0};
vec3_t up      = {0, 0, 1};

vec3_t position = {0};
double heading = 0;  // direction (rad)
double headingOffset = 0.0;

vec3_t lastPosition = {0};
unsigned long stateLeftTicks = 0;
unsigned long stateRightTicks = 0;

// last values used for gps heading
vec3_t lastHeadingPos = {0};
float lastYaw = 0.0;

// previous gps position
vec3_t lastGpsPos = {0};
float lastGpsVarN;
float lastGpsVarE;
float lastGpsVarEN;

unsigned long lastInvalidTime = 0;
unsigned long lastFixJumpTime = 0;

float lateralError = 0; // lateral error

quat_t euler_to_quaternion(vec3_t rpy)
{
    float roll = rpy.x;
    float pitch = rpy.y;
    float yaw = rpy.z;
    float qx = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - cos(roll/2) * sin(pitch/2) * sin(yaw/2);
    float qy = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + sin(roll/2) * cos(pitch/2) * sin(yaw/2);
    float qz = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - sin(roll/2) * sin(pitch/2) * cos(yaw/2);
    float qw = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + sin(roll/2) * sin(pitch/2) * sin(yaw/2);
    return quat_t(qw, qx, qy, qz);
}

vec3_t quaternion_to_euler(quat_t q)
{
    float w = q.w;
    float x = q.v.x;
    float y = q.v.y;
    float z = q.v.z;
    float t0 = 2.0 * (w * x + y * z);
    float t1 = 1.0 - 2.0 * (x * x + y * y);
    float roll = atan2(t0, t1);
    float t2 = +2.0 * (w * y - z * x);
    t2 = (t2 > 1.0) ? 1.0 : t2;
    t2 = (t2 < -1.0) ? -1.0 : t2;
    float pitch = asin(t2);
    float t3 = 2.0 * (w * z + x * y);
    float t4 = 1.0 - 2.0 * (y * y + z * z);
    float yaw = atan2(t3, t4);
    return vec3_t(roll, pitch, yaw);
}

bool shouldUseGps()
{
  // should we use GPS if rover is close to dock station?
  // avoids fix jumps
  bool dockGPS = true;
  #ifdef DOCK_IGNORE_GPS_DISTANCE
    if (maps.isDocking() && maps.getDockDistance() < DOCK_IGNORE_GPS_DISTANCE)
      dockGPS = false;
  #endif

  // use gps only on fix/float
  // ignore if signal jumped recently or coming from invalid
  bool solution = (
    (gps.solution == SOL_FIXED || (gps.solution == SOL_FLOAT && maps.wayMode != WAY_DOCK)) 
    && millis() - lastInvalidTime > IGNORE_GPS_AFTER_INVALID * 1000.0
    && millis() - lastFixJumpTime > IGNORE_GPS_AFTER_JUMP * 1000.0
  );

  return dockGPS && solution;
}

// https://learn.sparkfun.com/tutorials/9dof-razor-imu-m0-hookup-guide#using-the-mpu-9250-dmp-arduino-library
// start IMU sensor and calibrate
bool startIMU(bool forceIMU)
{    
  DEBUGLN("Detecting IMU");

  // detect IMU
  int counter = 0;  
  while (true)
  {
    imuDriver.detect();
    if (imuDriver.imuFound)
      break;

    watchdogReset();
    I2Creset();  
    Wire.begin();    
    #ifdef I2C_SPEED
      Wire.setClock(I2C_SPEED);   
    #endif
    
    counter++;
    if (counter > 5)
    {    
      // no I2C recovery possible - this should not happen (I2C module error)
      CONSOLE.println("ERROR IMU not found");
      activeOp->onImuError();            
      return false;
    }       
  }

  watchdogReset();

  // initialize IMU
  counter = 0;  
  while (true)
  {
    if (imuDriver.begin())
      break;

    CONSOLE.print("Unable to communicate with IMU.");
    CONSOLE.println("Check connections, and try again.");

    watchdogReset();
    delay(100);   

    counter++;
    if (counter > 5)
    {
      activeOp->onImuError();        
      return false;
    }  
  }              

  // calibrate IMU
  activeOp->changeOp(imuCalibrationOp, true);

  return true;
}

// read IMU sensor (and restart if required)
// I2C recovery: It can be minutes or hours, then there's an I2C error (probably due an spike on the 
// SCL/SDA lines) and the I2C bus on the pcb1.3 (and the arduino library) hangs and communication is delayed. 
// We check if the communication is significantly (10ms instead of 1ms) delayed, if so we restart the I2C 
// bus (by clocking out any garbage on the I2C bus) and then restarting the IMU module.
// https://learn.sparkfun.com/tutorials/9dof-razor-imu-m0-hookup-guide/using-the-mpu-9250-dmp-arduino-library
void readIMU(){
  if (!imuDriver.imuFound) return;

  // Check for new data in the FIFO  
  // check time for I2C access : if too long, there's an I2C issue and we need to restart I2C bus...
  unsigned long startTime = millis();
  bool avail = imuDriver.isDataAvail();
  unsigned long duration = millis() - startTime;    

  if (avail)
    resetImuTimeout(); // reset IMU data timeout, if IMU data available

  if (duration > 30 || millis() > imuDataTimeout)
  {
    if (millis() > imuDataTimeout){
      CONSOLE.print("ERROR IMU data timeout: ");
      CONSOLE.print(millis()-imuDataTimeout);
      CONSOLE.println(" (check RTC battery if problem persists)");  
    } else {
      CONSOLE.print("ERROR IMU timeout: ");
      CONSOLE.print(duration);     
      CONSOLE.println(" (check RTC battery if problem persists)");          
    }
    stateSensor = SENS_IMU_TIMEOUT;
    motor.stopImmediately(true);    

    // restart I2C bus
    statImuRecoveries++;  
    Wire.begin(); 
    startIMU(true);

    // if require IMU for mowing
    //{
      //activeOp->changeOp(waitImuOP)
    //}

    return;
  } 
  
  if (avail && ENABLE_TILT_DETECTION)
    if (degrees(fabs(scalePI(imuDriver.roll))  > 100.0)
    ||  degrees(fabs(scalePI(imuDriver.pitch)) > 100.0))
      activeOp->onImuTilt();
}

void resetImuTimeout(){
  imuDataTimeout = millis() + 500;  
}


// Wheel diamter estimator

vec3_t lastGpsPosIntegral = vec3_t(0, 0, 0);
unsigned long totalTicks = TICKS_PER_REVOLUTION_L * 60000;
double totalDistance = (TICKS_PER_REVOLUTION_L * 60000 * 0.5 * PI) / (1000.0 * TICKS_PER_REVOLUTION_L) * WHEEL_DIAMETER;
int tempTicks = 0;

void estimateWheelDiameter(long leftDelta, long rightDelta, vec3_t gpsPos)
{
  vec3_t target     = vec3_t(maps.targetPoint.x(),     maps.targetPoint.y()    );
  vec3_t lastTarget = vec3_t(maps.lastTargetPoint.x(), maps.lastTargetPoint.y());
       
  float targetDist     = distance(position, target);
  float lastTargetDist = distance(position, lastTarget);

  if (gps.solution == SOL_FIXED
  && millis() - lastFixTime < 500
  && shouldUseGps()
  && fabs(motor.linearSpeedSet) > setSpeed * 0.9
  && targetDist > 1.0
  && lastTargetDist > 1.0)
  {
    tempTicks += abs(leftDelta + rightDelta);
    if (gps.solutionAvail)
    {
      totalDistance += dot(gpsPos - lastGpsPosIntegral, forward) * sign(motor.linearSpeedSet);
      totalTicks += tempTicks;

      tempTicks = 0;
    }

    if (totalTicks > TICKS_PER_REVOLUTION_L * 60000)
    {
      totalDistance *= 0.9;
      totalTicks *= 0.9;
    }

    motor.wheelDiameter = (1000.0 * totalDistance * TICKS_PER_REVOLUTION_L) / (totalTicks * 0.5 * PI);
  }
  else
    tempTicks = 0;

  if (gps.solutionAvail)
    lastGpsPosIntegral = gpsPos;
}


// Wheel base estimator

float lastHeading = 0.0;
unsigned long totalAngularTicks = 0;
double totalAngularDistance = 0.0;
double wheelBase = 40.0;
int tempAngularTicks = 0;

void estimateWheelBase(long leftDelta, long rightDelta)
{
  if (fabs(motor.angularSpeedSet) > 0.3
  && fabs(motor.linearSpeedSet) < 0.001
  && fabs(imuDriver.yawSpeed) > 0.1)
  {
    tempAngularTicks += leftDelta - rightDelta;
    if (imuDriver.newData)
    {
      totalAngularDistance += fabs(distancePI(imuDriver.yaw, lastHeading));
      totalAngularTicks += abs(tempAngularTicks);

      tempAngularTicks = 0;
    }

    if (totalAngularTicks > ULONG_MAX * 0.9)
    {
      totalAngularDistance *= 0.75;
      totalAngularTicks *= 0.75;
    }

    if (totalAngularDistance > TAU * 10.0)
      motor.wheelBaseCm = (PI * motor.wheelDiameter * (totalAngularTicks / TICKS_PER_REVOLUTION_L)) / totalAngularDistance * 0.1;
  }
  else
    tempAngularTicks = 0;

  if (imuDriver.newData)
    lastHeading = imuDriver.yaw;
}



double uncertainty = 9999.0;

double gpsVariance = 0.2/10000.0;
double odometryVariance = 8.2 / 10000.0;
double innovationVariance = 0.0 / 10000.0;

vec3_t odometryEst = vec3_t(0);
void estimateOdometryError(vec3_t gpsPos)
{
  double inn = (gpsPos.xy() - position.xy()).mag()
             * (gpsPos.xy() - position.xy()).mag();

  double var = (gpsPos.xy() - odometryEst.xy()).mag()
             * (gpsPos.xy() - odometryEst.xy()).mag();

              
  double distance = (gpsPos.xy() - lastGpsPos.xy()).mag();
  
  double a = 0.0001;

  /*double gVar = inn - uncertainty;
  gVar = max(gVar, 0.1/10000.0);
  gVar = gps.variance;*/
  gpsVariance = gpsVariance * (1.0 - 0.1) + gps.variance * 0.1; 
    
  if (fabs(motor.linearSpeedSet) > 0.15 && distance > 0.01)
  {
    double oVar = (var - gpsVariance * 0.5) / distance;
    oVar = max(oVar, 0.0);
    odometryVariance = odometryVariance * (1.0 - a) + oVar * a;
  }
  
  double iVar = inn - gps.variance;
  iVar = max(iVar, 0.0);
  innovationVariance = innovationVariance * (1.0 - (a*100.0)) + iVar * (a*100.0);
}


double odoMovement = 0;

double lastHeadingOffset = -10.0;
double yawTime = 0.0;
double yawDrift = 0.0;
double yawDriftOffset = 0.0;

double yawUncertainty = 0.1;
double yawVariance = 0.0;
double yawDT = 0.0;
void estimateYawError(double nHeading, double nHeadingVar)
{
  vec3_t target     = vec3_t(maps.targetPoint.x(),     maps.targetPoint.y()    );
  vec3_t lastTarget = vec3_t(maps.lastTargetPoint.x(), maps.lastTargetPoint.y());
  vec3_t direction  = (target - lastTarget).norm();
       
  float targetDist     = distance(position, target);
  float lastTargetDist = distance(position, lastTarget);

  // Path constrain
  if ((stateOp == OP_MOW || stateOp == OP_DOCK)
  //&& (maps.wayMode == WAY_MOW || maps.wayMode == WAY_DOCK)
  && gps.solution == SOL_FIXED
  && direction.mag() > 0.1
  && fabs(motor.angularSpeedSet) < 0.1
  && fabs(motor.linearSpeedSet) > 0.1
  && lastTargetDist > 0.15
  && targetDist > 0.15
  && fabs(lateralError) < 0.02)
  {
    double pHeading = atan2(direction.y, direction.x);
    if (motor.linearSpeedSet < 0.0)
      pHeading = scalePId(pHeading + PI);
      
    double headingDiff = scalePId(pHeading - imuDriver.yaw);
    headingOffset = angleInterpolation(headingOffset, headingDiff, 0.01);

    yawUncertainty = (1.0 - 0.01) * yawUncertainty;
  }

  // Drift
  if (lastHeadingOffset < -5.0)
    lastHeadingOffset = headingOffset;

  if (yawTime > 60.0 && stateOp == OP_MOW)
  {
    //yawDrift += (lastHeadingOffset - headingOffset) / yawTime * 0.1;
    yawDrift = (lastHeadingOffset - headingOffset) / yawTime;
    
    yawTime = 0.0;
    //DEBUG(yawDrift*60.0*10.0);
    //DEBUG(" ");
    //DEBUGLN((lastHeadingOffset - headingOffset)*10.0);
    lastHeadingOffset = headingOffset;
  }

  if (fabs(yawDrift) * 60.0 > 0.1  && (stateOp == OP_MOW || stateOp == OP_DOCK))
  {
    lastHeadingOffset = headingOffset;
    yawTime = 0.0;
    yawDrift = 0.0;
    activeOp->changeOp(imuCalibrationOp, true);
  }

  // GPS Fusion
  if (sqrt(nHeadingVar) > 0.2 || odoMovement < 0.04 || targetDist < 0.15 || lastTargetDist  < 0.15)
    return;

  double K = yawUncertainty / (yawUncertainty + nHeadingVar);
  yawUncertainty = (1.0 - K) * yawUncertainty;

  K = max(K, 0.01);

  /*double yawDiff = scalePId(nHeading - heading);
  double yawVar = max(sq(yawDiff) - nHeadingVar, 0.0) / sq(yawDT);
  float a = 0.001;
  yawVariance = yawVariance * (1.0 - a) + yawVar * a;
  yawDrift = yawDrift * (1.0 - a*0.01) + yawDiff / yawDT * a*0.01;
  DEBUG(sqrt(yawVariance));
  DEBUG(" ");
  DEBUGLN(yawDrift * 60.0);*/

  double headingDiff = scalePId(nHeading - imuDriver.yaw);
  headingOffset = angleInterpolation(headingOffset, headingDiff, K);

  //yawBias = scalePId(nHeaiding - heading) / dt * 0.0001

  odoMovement = 0;
  yawDT = 0.0;
}

// compute robot state (x,y,delta)
// uses complementary filter ( https://gunjanpatel.wordpress.com/2016/07/07/complementary-filter-design/ )
// to fusion GPS heading (long-term) and IMU heading (short-term)
// with IMU: heading (stateDelta) is computed by gyro (stateDeltaIMU)
// without IMU: heading (stateDelta) is computed by odometry (deltaOdometry)

Timer odometryTimer(MICROS_TIME);
float magOffset = 0.0;
bool ignoringGps = false;

#define ODOMETRY_RPM_TO_STATE true
void computeRobotState()
{
  odometryTimer.update();

  // Ticks
  long leftDelta  = motor.motorLeftTicks  - stateLeftTicks;
  long rightDelta = motor.motorRightTicks - stateRightTicks; 
  
  stateLeftTicks  = motor.motorLeftTicks;
  stateRightTicks = motor.motorRightTicks; 

  // Odometry linear and angular change
#if ODOMETRY_RPM_TO_STATE
  float speedLeft  = motor.motorLeftRpmCurr  * PI * (motor.wheelDiameter / 1000.0) / 60.0;
  float speedRight = motor.motorRightRpmCurr * PI * (motor.wheelDiameter / 1000.0) / 60.0;

  float distLeft  = speedLeft  * odometryTimer.deltaTimeSeconds(); // meters
  float distRight = speedRight * odometryTimer.deltaTimeSeconds(); // meters

  float accelerationLeft  = motor.motorLeftRpmAcc  * PI * (motor.wheelDiameter / 1000.0) / 60.0;
  float accelerationRight = motor.motorRightRpmAcc * PI * (motor.wheelDiameter / 1000.0) / 60.0;
  
#else
  float distLeft  = (float)leftDelta  / (TICKS_PER_REVOLUTION_L / (motor.wheelDiameter / 10.0) / PI) / 100.0;
  float distRight = (float)rightDelta / (TICKS_PER_REVOLUTION_R / (motor.wheelDiameter / 10.0) / PI) / 100.0;

  speedLeft  = distLeft  / odometryTimer.deltaTimeSeconds();
  speedRight = distRight / odometryTimer.deltaTimeSeconds();
#endif

  float speedOdometry = (speedLeft + speedRight) / 2.0;
  float distOdometry  =  (distLeft + distRight) / 2.0;
  float angularOdometry = -(distLeft - distRight) / (motor.wheelBaseCm / 100.0);

  float linearAcc = (accelerationLeft + accelerationRight) / 2.0;

  //if (imuDriver.accX - linearAcc) triggerObstacle()

  // orientation and heading
  if (/*still*/ stateOp != OP_MOW && stateOp != OP_DOCK)
  {
    yawDrift = 0.0;
    yawTime = 0.0;
    lastHeadingOffset = headingOffset;
  }
  //else
    //yawDriftOffset = scalePId(yawDriftOffset + yawDrift * odometryTimer.deltaTimeSeconds());
  
  vec3_t rpy;
  if (imuDriver.imuFound) // IMU available
  {
    bool magZone = !shouldUseGps();// || maps.distanceToLastTargetPoint(position.x, position.y) > 4.0 || gps.solution != SOL_FIXED;
    if (!magZone)
      magOffset = distancePI(imuDriver.magYaw, heading);

    if (imuDriver.newData)
      heading = (USE_MAGNOMETER && maps.isDocking() && magZone && !battery.isDocked())
        ? scalePI(imuDriver.magYaw + magOffset)
        : scalePI(imuDriver.yaw    + headingOffset /*+ yawDriftOffset*/);
    else
      heading = scalePI(heading + angularOdometry * 0.5 + imuDriver.yawSpeed * 0.5 * odometryTimer.deltaTimeSeconds()); 

    rpy = vec3_t(imuDriver.roll, imuDriver.pitch, heading);
  }
  else // odometry
  {
    heading = scalePI(heading + angularOdometry);  
    rpy = vec3_t(0.0, 0.0, heading);
  } 
  
  // corrected orientation quaternion and vectors
  quat_t x; x.setRotation({1,0,0}, rpy.x, false);  
  quat_t y; y.setRotation({0,1,0}, rpy.y, false);  
  quat_t z; z.setRotation({0,0,1}, rpy.z, false);
  quat_t rot = (z*y*x).norm();

  forward = rot.rotate({1, 0, 0}, GLOBAL_FRAME).norm(); 
  right   = rot.rotate({0,-1, 0}, GLOBAL_FRAME).norm();
  up      = rot.rotate({0, 0, 1}, GLOBAL_FRAME).norm();
  
  // gps position
  vec3_t gpsPos;
  if (absolutePosSource)
  {
    double e, n;
    linearRelativeLL(absolutePosSourceLat, absolutePosSourceLon, gps.lat, gps.lon, n, e);
    gpsPos = vec3_t(e, n, gps.height);
  }  
  else
    gpsPos = vec3_t(gps.relPosE + RELPOS_OFFSET_E, gps.relPosN + RELPOS_OFFSET_N, 0);

  // detect fix jumps before heading fusion
  if (gps.solutionAvail
  && gps.solution == SOL_FIXED
  && (gpsPos.xy() - lastGpsPos.xy()/*position*/).mag() > 0.15
  && millis() - lastFixJumpTime > IGNORE_GPS_AFTER_JUMP * 1000.0
  && !ignoringGps)
  {
    ignoringGps = true;
    statGPSJumps++;
    lastFixJumpTime = millis();
  }
  // store last position if ignoring gps
  if (ignoringGps)
  {
    lastHeadingPos = gpsPos;
    lastYaw = imuDriver.yaw;
  }
  if (ignoringGps &&  millis() - lastFixJumpTime > IGNORE_GPS_AFTER_JUMP * 1000.0)
    ignoringGps = false;


  // estimate wheel diameter and wheel base
  estimateWheelDiameter(leftDelta, rightDelta, gpsPos);
  estimateWheelBase(leftDelta, rightDelta);

  // Uncertainties
  {
    odoMovement += fabs(distOdometry);
    yawTime += odometryTimer.deltaTimeSeconds();
    yawDT += odometryTimer.deltaTimeSeconds();

    //yawUncertainty += sq(odometryTimer.deltaTimeSeconds()) * sq(yawDrift);
    yawUncertainty += odometryTimer.deltaTimeSeconds() * sq(yawDrift);
  }
  {
    position    += forward * distOdometry;
    odometryEst += forward * distOdometry;

    uncertainty += odometryVariance * fabs(distOdometry);
  }

  // gps heading and position
  if (gps.solutionAvail && shouldUseGps())
  {
    gps.heading = scalePI(-gps.heading + PI * 0.5);
    if (motor.linearSpeedSet < 0.0)
      gps.heading = scalePI(gps.heading + PI);

    vec3_t deltaGps = gpsPos - lastGpsPos;
    double nHeading = atan2(deltaGps.y, deltaGps.x);
    if (motor.linearSpeedSet < 0.0)
      nHeading = scalePId(nHeading + PI);

    double var_dx = lastGpsVarE + gps.varianceE;
    double var_dy = lastGpsVarN + gps.varianceN;
    double cov_dxdy = lastGpsVarEN + gps.varianceEN;

    double r2 = deltaGps.x*deltaGps.x + deltaGps.y*deltaGps.y;

    double nVar =
      ( deltaGps.y*deltaGps.y * var_dx
      + deltaGps.x*deltaGps.x * var_dy
      - 2.0 * deltaGps.x * deltaGps.y * cov_dxdy)
      / (r2 * r2);
    if (isnan(nHeading)) nHeading = 0.0;
    if (isnan(nVar)) nVar = 99999999.9;

    double vHeading = atan2(gps.velocityY, gps.velocityX);
    if (motor.linearSpeedSet < 0.0)
      vHeading = scalePId(vHeading + PI);
    double vVar = (sq(gps.velocityY)*gps.varianceVE + sq(gps.velocityX)*gps.varianceVN - 2.0*gps.velocityX*gps.velocityY*gps.varianceVEN)
                / sq(sq(gps.velocityX) + sq(gps.velocityY));
    if (isnan(vHeading)) vHeading = 0.0;
    if (isnan(vVar)) vVar = 99999999.9;

    /*DEBUG(gps.heading);
    DEBUG(" ");
    DEBUG(gps.headingAcc);
    DEBUG(" ");
    DEBUG(nHeading);
    DEBUG(" ");
    DEBUG(sqrt(nVar));
    DEBUG(" ");
    DEBUG(vHeading);
    DEBUG(" ");
    DEBUGLN(sqrt(vVar));*/


    // -- Heading --
    //estimateYawError(gps.heading, gps.headingAcc*gps.headingAcc);
    estimateYawError(nHeading, nVar);

    // -- Position --
    estimateOdometryError(gpsPos);

    // gps antenna offset
    vec3_t measuredPos = gpsPos;
    if (GPS_POSITION_OFFSET_ENABLED && imuDriver.imuFound)
    {
      vec3_t gpsOffset = 
        forward * (GPS_POSITION_OFFSET_FORWARD / 100.0)
        + right * (GPS_POSITION_OFFSET_RIGHT / 100.0)
        + up    * (GPS_POSITION_OFFSET_UP  / 100.0);

      measuredPos += gpsOffset;
    }
    
    if (gpsVariance > 1.0/10000.0)
      uncertainty += innovationVariance * 0.1;
    
    double K = uncertainty / (uncertainty + gpsVariance);
    position += K * (measuredPos.xy() - position);
    uncertainty *= (1.0 - K);
    
    if (gps.solution == SOL_FIXED)
      lastFixTime = millis();
  }

  if (gps.solutionAvail)
  {
    lastGpsPos = gpsPos;
    lastGpsVarE = gps.varianceE;
    lastGpsVarN = gps.varianceN;
    lastGpsVarEN = gps.varianceEN;

    odometryEst = gpsPos;
  }

  if (isnan(position.x))
  {
    position = vec3_t(0);
    headingOffset = 0.0;
  }

  // remove height component
  position = position.xy();

  // set last invalid time
  if (gps.solution == SOL_INVALID)
    lastInvalidTime = millis();
 
 
  if (stateOp == OP_MOW)
    statMowDistanceTraveled += (position-lastPosition).mag();
  lastPosition = position;

  gps.solutionAvail = false;
}

