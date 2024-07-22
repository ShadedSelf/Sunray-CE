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

#include "src/math/quaternion_type.h"
#include "src/math/vector_type.h"


vec3_t forward = {1,0,0};
vec3_t right = {0,-1,0};
vec3_t up = {0,0,1};

vec3_t position = {0,0,0};
float heading = 0;  // direction (rad)
double headingOffset = 0.0;

//float stateRoll = 0;
//float statePitch = 0;
float stateGroundSpeed = 0; // m/s

unsigned long stateLeftTicks = 0;
unsigned long stateRightTicks = 0;

vec3_t lastGpsPos = {0,0,0};
float lastHeading = 0;

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
    vec3_t dockPos = {0,0,0};
    float dockDelta = 0;
    maps.getDockingPos(dockPos.x, dockPos.y, dockDelta);

    float dockDist = (dockPos - position).mag();
    if (maps.isDocking() && dockDist < DOCK_IGNORE_GPS_DISTANCE)
      dockGPS = false;
  #endif

  // use gps only on fix/float
  // ignore if signal jumped recently or coming from invalid
  bool solution = ((gps.solution == SOL_FIXED || gps.solution == SOL_FLOAT) 
    && millis() - lastInvalidTime > IGNORE_GPS_AFTER_INVALID * 1000.0
    && millis() - lastFixJumpTime > IGNORE_GPS_AFTER_JUMP * 1000.0);

  return dockGPS && solution;
}

// https://learn.sparkfun.com/tutorials/9dof-razor-imu-m0-hookup-guide#using-the-mpu-9250-dmp-arduino-library
// start IMU sensor and calibrate
bool startIMU(bool forceIMU){    
  // detect IMU
  int counter = 0;  
  while (forceIMU || counter < 1)
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
    if (counter > 5){    
      // no I2C recovery possible - this should not happen (I2C module error)
      CONSOLE.println("ERROR IMU not found");
      activeOp->onImuError();            
      return false;
    }       
  }  

  // initialize IMU
  counter = 0;  
  while (true)
  {    
    if (imuDriver.begin())
      break;

    CONSOLE.print("Unable to communicate with IMU.");
    CONSOLE.print("Check connections, and try again.");
    CONSOLE.println();

    watchdogReset();
    delay(500);   

    counter++;
    if (counter > 5){
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
    I2Creset();  
    Wire.begin(); 
    startIMU(true);

    return;
  } 
  
  if (avail && ENABLE_TILT_DETECTION)
    if (fabs(scalePI(imuDriver.roll) > 60.0/180.0*PI)
    ||  fabs(scalePI(imuDriver.pitch) > 100.0/180.0*PI))
      activeOp->onImuTilt();
}

void resetImuTimeout(){
  imuDataTimeout = millis() + 500;  
}

// compute robot state (x,y,delta)
// uses complementary filter ( https://gunjanpatel.wordpress.com/2016/07/07/complementary-filter-design/ )
// to fusion GPS heading (long-term) and IMU heading (short-term)
// with IMU: heading (stateDelta) is computed by gyro (stateDeltaIMU)
// without IMU: heading (stateDelta) is computed by odometry (deltaOdometry)
void computeRobotState()
{  
  long leftDelta = motor.motorLeftTicks-stateLeftTicks;
  long rightDelta = motor.motorRightTicks-stateRightTicks;  
  stateLeftTicks = motor.motorLeftTicks;
  stateRightTicks = motor.motorRightTicks;    
    
  float distLeft = (float)leftDelta / motor.ticksPerCm;
  float distRight = (float)rightDelta / motor.ticksPerCm;  
  //float distLeft = motor.motorLeftRpmCurr * PI * ((float)motor.wheelDiameter / 10.0) / 60.0 * 0.02;
  //float distRight =  motor.motorRightRpmCurr * PI * ((float)motor.wheelDiameter / 10.0) / 60.0 * 0.02; 
  float distOdometry = (distLeft + distRight) / 2.0;
  float deltaOdometry = -(distLeft - distRight) / motor.wheelBaseCm;  

  // orientation and heading
  vec3_t rpy;
  if (imuDriver.imuFound) // IMU available
  {
    heading = scalePI(imuDriver.yaw + headingOffset);
    rpy = vec3_t(imuDriver.roll, imuDriver.pitch, heading);
  }
  else // odometry
  {
    heading = scalePI(heading + deltaOdometry);  
    rpy = vec3_t(0.0, 0.0, heading);
  }
  
  // corrected orientation quaternion and vectors
  quat_t x; x.setRotation({1,0,0}, rpy.x, false);  
  quat_t y; y.setRotation({0,1,0}, rpy.y, false);  
  quat_t z; z.setRotation({0,0,1}, rpy.z, false);
  quat_t rot = (z*y*x).norm();

  forward = rot.rotate({1,0,0}, GLOBAL_FRAME).norm(); 
  right = rot.rotate({0,-1,0}, GLOBAL_FRAME).norm();
  up = rot.rotate({0,0,1}, GLOBAL_FRAME).norm();
  
  // gps position
  vec3_t gpsPos = {0,0,0};
  if (absolutePosSource)
    relativeLL(absolutePosSourceLat, absolutePosSourceLon, gps.lat, gps.lon, gpsPos.y, gpsPos.x);    
  else
    gpsPos = vec3_t(gps.relPosE, gps.relPosN, 0) - vec3_t(3075.239, -5381.777, 0);  

  /*CONSOLE.print(gps.relPosE - gpsPos.x, 8);
  CONSOLE.print(" ");
  CONSOLE.println(gps.relPosN - gpsPos.y, 8);*/

  // gps antenna offset
  if (GPS_POSITION_OFFSET_ENABLED && imuDriver.imuFound)
  {
    vec3_t gpsOffset = forward * (GPS_POSITION_OFFSET_FORWARD / 100.0)
                      + right * (GPS_POSITION_OFFSET_RIGHT / 100.0)
                      + up * (GPS_POSITION_OFFSET_UP / 100.0);

    gpsPos += gpsOffset ^ vec3_t(1,1,0);
  }

  // detect fix jumps before heading fusion
  {
    /*if (gps.solutionAvail && gps.solution == SOL_FIXED
    && distGPS > 0.25
    && millis() > lastFixJumpTime + IGNORE_GPS_AFTER_JUMP * 1000.0)
      lastFixJumpTime = millis();

    // store last position if ignoring gps fusion
    if (gps.solutionAvail && distGPS > 0.1 && millis() <= lastInvalidTime + IGNORE_GPS_AFTER_INVALID * 1000.0)
    {
      lastGpsPos = gpsPos;
      lastHeading = heading;
    }*/
  }

  // gps heading and position
  if (gps.solutionAvail && shouldUseGps())
  {
    gps.solutionAvail = false;
    stateGroundSpeed = 0.9 * stateGroundSpeed + 0.1 * abs(gps.groundSpeed); 

    float distGPS = (gpsPos-lastGpsPos).mag();   
    
    if (fabs(motor.linearSpeedSet) < 0.01)    // reset previous position and heading
    {
      lastGpsPos = gpsPos;
      lastHeading = heading;
    }
    else if (distGPS > 0.2 && gps.solution == SOL_FIXED)    // gps-imu heading fusion
    {                 
      if (fabs(distancePI(heading, lastHeading)) / PI * 180.0 < 10) // make sure robot is not turning  
      {
        float headingGPS = atan2(gpsPos.y-lastGpsPos.y, gpsPos.x-lastGpsPos.x);
        if (motor.linearSpeedSet < 0)
          headingGPS = scalePI(headingGPS + PI); // consider if driving reverse

        if (imuDriver.imuFound) // imu
        {
          float headingDiff = distancePI(imuDriver.yaw, headingGPS);
          headingOffset = angleInterpolation(headingOffset, headingDiff, 1.0 - GPS_IMU_FUSION);

          if (fabs(distancePI(heading, headingGPS) / PI * 180) > 45) // IMU-based heading too far away => use GPS heading
            headingOffset = headingDiff;
        }
        else // odometry
          heading = angleInterpolation(headingGPS, heading, 0.9);
      }

      lastGpsPos = gpsPos;
      lastHeading = heading;
    } 

    // set last fix time
    if (gps.solution == SOL_FIXED)
      lastFixTime = millis();
    
    // update state for fix and float
    if (gps.solution == SOL_FIXED) // fix
      position = gpsPos;
    else if (gps.solution == SOL_FLOAT) // allows planner to use float solution?
      position = (position + forward * (distOdometry/100.0)) * IMU_FLOAT_FUSION
                + gpsPos * (1.0 - IMU_FLOAT_FUSION);
  }
  else // no GPS data available, use odometry
    position += forward * (distOdometry/100.0);

  // set last invalid time
  if (gps.solutionAvail && gps.solution == SOL_INVALID)
    lastInvalidTime = millis();
 
 
  if (stateOp == OP_MOW) statMowDistanceTraveled += distOdometry/100.0;
   
  /*if (imuDriver.imuFound)
    stateDeltaSpeedIMU = 0.99 * stateDeltaSpeedIMU + 0.01 * stateDeltaIMU / 0.02; // IMU yaw rotation speed (20ms timestep)

  stateDeltaSpeedWheels = 0.99 * stateDeltaSpeedWheels + 0.01 * deltaOdometry / 0.02; // wheels yaw rotation speed (20ms timestep) 
  stateDeltaIMU = 0;

  // compute yaw rotation speed (delta speed)
  stateDeltaSpeed = (stateDelta - stateDeltaLast) / 0.02;  // 20ms timestep
  stateDeltaSpeedLP = stateDeltaSpeedLP * 0.95 + fabs(stateDeltaSpeed) * 0.05;     
  stateDeltaLast = stateDelta;

  if (imuDriver.imuFound) {
    diffIMUWheelYawSpeed = stateDeltaSpeedIMU - stateDeltaSpeedWheels;
    diffIMUWheelYawSpeedLP = diffIMUWheelYawSpeedLP * 0.95 + fabs(diffIMUWheelYawSpeed) * 0.05;  
  }*/
}

