// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)

#include "motor.h"
#include "config.h"
#include "helper.h"
#include "robot.h"
#include "Arduino.h"


void Motor::begin() {
  pwmMax = MAX_GEAR_PWM;
  pwmMaxMow = MAX_MOW_PWM;
  
  ticksPerRevolution = TICKS_PER_REVOLUTION;
	wheelBaseCm = WHEEL_BASE_CM;    // wheel-to-wheel distance (cm) 36
  wheelDiameter = WHEEL_DIAMETER; // wheel diameter (mm)
  ticksPerCm = ((float)ticksPerRevolution) / (((float)wheelDiameter) / 10.0) / PI;    // computes encoder ticks per cm (do not change)  

  // Motor PIDs -------
  motorLeftPID.Kp = motorRightPID.Kp = MOTOR_PID_KP;  // 2.0;  
  motorLeftPID.Ki = motorRightPID.Ki = MOTOR_PID_KI;  // 0.03; 
  motorLeftPID.Kd = motorRightPID.Kd = MOTOR_PID_KD;  // 0.03;
  
  motorLeftPID.TaMax = motorRightPID.TaMax = 0.1;
  
  motorLeftPID.y_min = motorRightPID.y_min = -pwmMax;
  motorLeftPID.y_max = motorRightPID.y_max = pwmMax;
  motorLeftPID.max_output = motorRightPID.max_output = pwmMax;

  motorLeftPID.reset(); 
  motorRightPID.reset();		 

#if MOW_RPM_CONTROL
  motorMowPID.Kp = 0.0005;
  motorMowPID.Ki = 0.0;
  motorMowPID.Kd = 0.0;
  
  motorMowPID.TaMax = 0.1;
  
  motorMowPID.y_min = -10;
  motorMowPID.y_max = 10;
  motorMowPID.max_output = 10;

  motorMowPID.reset(); 
#endif
  //------------------

  #ifdef MOTOR_DRIVER_BRUSHLESS
    motorLeftSwapDir = true;
  #else
    motorLeftSwapDir = false;  
  #endif
  motorRightSwapDir = false;
  
  // apply optional custom motor direction swapping 
  #ifdef MOTOR_LEFT_SWAP_DIRECTION
    motorLeftSwapDir = !motorLeftSwapDir;
  #endif
  #ifdef MOTOR_RIGHT_SWAP_DIRECTION
    motorRightSwapDir = !motorRightSwapDir;
  #endif

  motorError = false;
  recoverMotorFault = false;
  recoverMotorFaultCounter = 0;
  nextRecoverMotorFaultTime = 0;
  enableMowMotor = ENABLE_MOW_MOTOR; //Default: true
  tractionMotorsEnabled = true;
  
  motorLeftOverload = false;
  motorRightOverload = false;
  motorMowOverload = false; 
  
  odometryError = false;  
  
  motorLeftSense = 0;
  motorRightSense = 0;
  motorMowSense = 0;  
  motorLeftSenseLP = 0;
  motorRightSenseLP = 0;
  motorMowSenseLP = 0;  
  motorMowSenseFLP = 0;
  motorsSenseLP = 0;

  linearSpeedSet = 0;
  angularSpeedSet = 0;
  motorLeftRpmSet = 0;
  motorRightRpmSet = 0;
  motorMowPWMSet = 0;
  motorMowForwardSet = true;
  toggleMowDir = MOW_TOGGLE_DIR;

  lastControlTime = 0;
  //nextSenseTime = 0;
  motorLeftTicks =0;  
  motorRightTicks =0;
  motorMowTicks = 0;
  motorLeftTicksZero=0;
  motorRightTicksZero=0;
  motorLeftPWMCurr =0;    
  motorRightPWMCurr=0; 
  motorMowPWMCurr = 0;
  /*motorLeftPWMCurrLP = 0;
  motorRightPWMCurrLP=0;   
  motorMowPWMCurrLP = 0;*/
  
  motorLeftRpmCurr=0;
  motorRightRpmCurr=0;
  motorMowRpmCurr=0;
  motorMowRpmCurrLP = 0;
  
  setLinearAngularSpeedTimeoutActive = false;  
  setLinearAngularSpeedTimeout = 0;
  motorMowSpinUpTime = 0;

  motorRecoveryState = false;
}

void Motor::setMowMaxPwm( int val ){
  pwmMaxMow = val;
}

void Motor::speedPWM( int pwmLeft, int pwmRight, int pwmMow )
{
  //Correct Motor Direction
  if (motorLeftSwapDir) pwmLeft *= -1;
  if (motorRightSwapDir) pwmRight *= -1;

  // ensure pwm is lower than Max
  pwmLeft = min(pwmMax, max(-pwmMax, pwmLeft));
  pwmRight = min(pwmMax, max(-pwmMax, pwmRight));  
  pwmMow = min(pwmMaxMow, max(-pwmMaxMow, pwmMow)); 
  
  motorDriver.setMotorPwm(pwmLeft, pwmRight, pwmMow);
}

// linear: m/s
// angular: rad/s
// -------unicycle model equations----------
//      L: wheel-to-wheel distance
//     VR: right speed (m/s)
//     VL: left speed  (m/s)
//  omega: rotation speed (rad/s)
//      V     = (VR + VL) / 2       =>  VR = V + omega * L/2
//      omega = (VR - VL) / L       =>  VL = V - omega * L/2
void Motor::setLinearAngularSpeed(float linear, float angular, bool useLinearRamp){
  setLinearAngularSpeedTimeout = millis() + 1000;
  setLinearAngularSpeedTimeoutActive = true;

  // linear
  if (USE_LINEAR_SPEED_RAMP && useLinearRamp)
    linearSpeedSet = 0.95 * linearSpeedSet + 0.05 * linear;
  else
    linearSpeedSet = linear;

  // angular
  angularSpeedSet = angular;   
   
   
  float rspeed = linearSpeedSet + angularSpeedSet * (wheelBaseCm /100.0 / 2.0);          
  float lspeed = linearSpeedSet - angularSpeedSet * (wheelBaseCm /100.0 / 2.0);   

  // RPM = V / (2*PI*r) * 60
  motorRightRpmSet =  rspeed / (PI*(((float)wheelDiameter)/1000.0)) * 60.0;   
  motorLeftRpmSet = lspeed / (PI*(((float)wheelDiameter)/1000.0)) * 60.0;   
}


void Motor::enableTractionMotors(bool enable){
  if (enable == tractionMotorsEnabled) return;
  if (enable)
    CONSOLE.println("traction motors enabled");
  else 
    CONSOLE.println("traction motors disabled");
  tractionMotorsEnabled = enable;
}


void Motor::setMowState(bool switchOn){
  if (enableMowMotor && switchOn)
  {
    if (fabs(motorMowPWMSet) > 0)
      return; // mowing motor already switch ON
   
    bool rnd = true;
    /*if (RANDOM_MOW_MOTOR_DIRECTION)
    {
      randomSeed(gps.iTOW);
      rnd = random(0, 2) == 1;
    }*/

    motorMowSpinUpTime = millis();
    if (toggleMowDir && rnd){
      // toggle mowing motor direction each mow motor start
      motorMowForwardSet = !motorMowForwardSet;
      motorMowPWMSet = (motorMowForwardSet) ? pwmMaxMow : -pwmMaxMow; 
    } 
    else      
      motorMowPWMSet = pwmMaxMow;  
  }
  else
  {
    motorMowPWMSet = 0;  
    motorMowPWMCurr = 0;
  }
}


void Motor::stopImmediately(bool includeMowerMotor){
  linearSpeedSet = 0;
  angularSpeedSet = 0;
  motorRightRpmSet = 0;
  motorLeftRpmSet = 0;      
  motorLeftPWMCurr = 0;
  motorRightPWMCurr = 0;  
  if (includeMowerMotor) {
    motorMowPWMSet = 0;
    //motorMowPWMCurr = 0;    
  }
  speedPWM(motorLeftPWMCurr, motorRightPWMCurr, motorMowPWMCurr);
  // reset PID
  motorLeftPID.reset();
  motorRightPID.reset();
#if MOW_RPM_CONTROL
  motorMowPID.reset(); 
#endif
  // reset unread encoder ticks
  int ticksLeft, ticksRight, ticksMow;
  motorDriver.getMotorEncoderTicks(ticksLeft, ticksRight, ticksMow);        
}

unsigned long prevTimeLeft, prevTimeRight;
bool shouldUpdateLeft, shouldUpdateRight;
void Motor::run()
{
  // timeframes
  unsigned long currTime = micros();
  unsigned long controlDt = currTime - lastControlTime;
  if (controlDt >= 50000) // 50 ms
  {
    float deltaControlTimeSec =  (double)controlDt / 1000.0 / 1000.0;
    lastControlTime = currTime;
    
    // angular timeout
    if (setLinearAngularSpeedTimeoutActive && millis() > setLinearAngularSpeedTimeout){
      setLinearAngularSpeedTimeoutActive = false;
      motorLeftRpmSet = 0;
      motorRightRpmSet = 0;
    }
      
    sense();
    checkOverload();

    // if motor driver indicates a fault signal, try a recovery   
    // if motor driver uses too much current, try a recovery     
    // if there is some error (odometry, too low current, rpm fault), try a recovery 
    if (!recoverMotorFault 
    && (checkFault()
    || checkCurrentTooHighError()
    || checkMowRpmFault()
    || checkOdometryError()
    || checkCurrentTooLowError()))
    {
      stopImmediately(true);
      recoverMotorFault = true;
      nextRecoverMotorFaultTime = millis() + 1000;                  
      motorRecoveryState = true;
    } 

    // try to recover from a motor driver fault signal by resetting the motor driver fault
    // if it fails, indicate a motor error to the robot control (so it can try an obstacle avoidance)  
    if (nextRecoverMotorFaultTime != 0){
      if (millis() > nextRecoverMotorFaultTime){
        if (recoverMotorFault){
          nextRecoverMotorFaultTime = millis() + 10000;
          recoverMotorFaultCounter++;                                               
          CONSOLE.print("motor fault recover counter ");
          CONSOLE.println(recoverMotorFaultCounter);
          motorDriver.resetMotorFaults();
          recoverMotorFault = false;  
          if (recoverMotorFaultCounter >= 10){ // too many successive motor faults
            //stopImmediately();
            CONSOLE.println("ERROR: motor recovery failed");
            recoverMotorFaultCounter = 0;
            motorError = true;
          }
        } else {
          CONSOLE.println("resetting recoverMotorFaultCounter");
          recoverMotorFaultCounter = 0;
          nextRecoverMotorFaultTime = 0;
          motorRecoveryState = false;
        }        
      }
    }

  }
  

  // -- Calculate motor RPM --
  
  int ticksLeft, ticksRight, ticksMow;
  motorDriver.getMotorEncoderTicks(ticksLeft, ticksRight, ticksMow);

  // does pid need to update
  shouldUpdateLeft  = ticksLeft  != 0 || (controlDt >= 50000);
  shouldUpdateRight = ticksRight != 0 || (controlDt >= 50000); 
  
  if (motorLeftPWMCurr < 0) ticksLeft *= -1;
  if (motorRightPWMCurr < 0) ticksRight *= -1;
  if (motorMowPWMCurr < 0) ticksMow *= -1;
  motorLeftTicks += ticksLeft;
  motorRightTicks += ticksRight;
  motorMowTicks += ticksMow;

#if RESPONSIVE_RPM
  unsigned long timeLeft, timeRight, timeMow;
  motorDriver.getMotorTickTime(timeLeft, timeRight, timeMow);

  // should PID controller run
  shouldUpdateLeft = timeLeft != prevTimeLeft || shouldUpdateLeft;
  shouldUpdateRight = timeRight != prevTimeRight || shouldUpdateRight;
  prevTimeLeft = timeLeft;
  prevTimeRight = timeRight;

  // calculat speed via tick time
  motorLeftRpmCurr =  (60000000.0 / (timeLeft  * ticksPerRevolution)) * (float)(fabs(motorLeftPWMCurr) > 0.5);
  motorRightRpmCurr = (60000000.0 / (timeRight * ticksPerRevolution)) * (float)(fabs(motorRightPWMCurr) > 0.5);
  motorMowRpmCurr =   (60000000.0 / (timeMow   * 6))                  * (float)(fabs(motorMowPWMCurr) > 0.5);

  if (motorLeftPWMCurr < 0.0) motorLeftRpmCurr *= -1.0;
  if (motorRightPWMCurr < 0.0) motorRightRpmCurr *= -1.0;
  if (motorMowPWMCurr < 0.0) motorMowRpmCurr *= -1.0;
# else
  // calculate speed via tick count
  motorLeftRpmCurr = 60.0 * ( ((float)ticksLeft) / ((float)ticksPerRevolution) ) / deltaControlTimeSec;
  motorRightRpmCurr = 60.0 * ( ((float)ticksRight) / ((float)ticksPerRevolution) ) / deltaControlTimeSec;
  motorMowRpmCurr = 60.0 * ( ((float)ticksMow) / ((float)6.0) ) / deltaControlTimeSec; // assuming 6 ticks per revolution
#endif

  // set rpm to 0 if we dont get ticks in 10 cycles
  if (ticksLeft == 0) motorLeftTicksZero++;
  if (ticksLeft != 0) motorLeftTicksZero = 0;
  if (motorLeftTicksZero > 10) motorLeftRpmCurr = 0;

  if (ticksRight == 0) motorRightTicksZero++;
  if (ticksRight != 0) motorRightTicksZero = 0;
  if (motorRightTicksZero > 10) motorRightRpmCurr = 0;

  // speed controller
  control(shouldUpdateLeft, shouldUpdateRight, true);
}  

// measure motor currents
void Motor::sense()
{
  motorDriver.getMotorCurrent(motorLeftSense, motorRightSense, motorMowSense);

  float lp = 0.995;
  motorRightSenseLP = lp * motorRightSenseLP + (1.0-lp) * motorRightSense;
  motorLeftSenseLP = lp * motorLeftSenseLP + (1.0-lp) * motorLeftSense;
  motorMowSenseLP = lp * motorMowSenseLP + (1.0-lp) * motorMowSense; 
  motorsSenseLP = motorRightSenseLP + motorLeftSenseLP + motorMowSenseLP;

  float flp = 0.8;
  motorMowSenseFLP = flp * motorMowSenseFLP + (1.0-flp) * motorMowSense; 
  
  /*motorRightPWMCurrLP = lp * motorRightPWMCurrLP + (1.0-lp) * motorRightPWMCurr;
  motorLeftPWMCurrLP = lp * motorLeftPWMCurrLP + (1.0-lp) * motorLeftPWMCurr;
  motorMowPWMCurrLP = lp * motorMowPWMCurrLP + (1.0-lp) * motorMowPWMCurr; */
}

//unsigned long leftTimeUpdate;
void Motor::control(bool updateLeft, bool updateRight, bool updateMow){
  //########################  Calculate PWM for left driving motor ############################
  if (updateLeft)
  {
    motorLeftPID.Kp = 0.9 * (3.0/50.0);
    motorLeftPID.Kp = 0.1;

    motorLeftPID.x = motorLeftRpmCurr;
    motorLeftPID.w = motorLeftRpmSet;
    motorLeftPID.compute();
    motorLeftPWMCurr += motorLeftPID.y;
    motorLeftPWMCurr = constrain(motorLeftPWMCurr, -pwmMax, pwmMax);
  }

  //########################  Calculate PWM for right driving motor ############################
  if (updateRight)
  {
    motorRightPID.Kp = 0.9 * (3.0/50.0);
    motorRightPID.Kp = 0.1;

    motorRightPID.x = motorRightRpmCurr;
    motorRightPID.w = motorRightRpmSet;
    motorRightPID.compute();
    motorRightPWMCurr += motorRightPID.y;
    motorRightPWMCurr = constrain(motorRightPWMCurr, -pwmMax, pwmMax);
  }

  //########################  Calculate PWM for mowing motor ############################
  if (updateMow)
  {  
  #if MOW_RPM_CONTROL
    motorMowPID.x = motorMowRpmCurr;
    motorMowPID.w = MOW_RPM * sign(motorMowPWMSet) * (float)(fabs(motorMowPWMSet) > 0.5);
    motorMowPID.compute();
    motorMowPWMCurr += motorMowPID.y;
    motorMowPWMCurr = constrain(motorMowPWMCurr, -pwmMaxMow, pwmMaxMow);
  #else
    motorMowPWMCurr = 0.85 * motorMowPWMCurr + 0.15 * motorMowPWMSet;
    //motorMowPWMCurr = 0.97 * motorMowPWMCurr + 0.03 * motorMowPWMSet;
    //motorMowPWMCurr = 0.98 * motorMowPWMCurr + 0.02 * motorMowPWMSet;
  #endif
  }

  //########################  set PWM for all motors ############################
  if (fabs(motorLeftRpmSet) < 0.01) motorLeftPWMCurr = 0;
  if (fabs(motorRightRpmSet) < 0.01) motorRightPWMCurr = 0;
  //if (fabs(motorMowPWMSet) < 0.5) motorMowPWMCurr = 0;

  if (!tractionMotorsEnabled)
    motorLeftPWMCurr = motorRightPWMCurr = 0;

  speedPWM(motorLeftPWMCurr, motorRightPWMCurr, motorMowPWMCurr);
}


// check if motor current too high
bool Motor::checkCurrentTooHighError(){
  bool motorLeftFault = (motorLeftSense > MOTOR_FAULT_CURRENT);
  bool motorRightFault = (motorRightSense > MOTOR_FAULT_CURRENT);
  bool motorMowFault = (motorMowSense > MOW_FAULT_CURRENT);
  if (motorLeftFault || motorRightFault || motorMowFault){
    CONSOLE.print("ERROR motor current too high: ");
    CONSOLE.print("  current=");
    CONSOLE.print(motorLeftSense);
    CONSOLE.print(",");
    CONSOLE.print(motorRightSense);
    CONSOLE.print(",");
    CONSOLE.println(motorMowSense);
    return true;
  } 
  return false; 
}


// check if motor current too low
bool Motor::checkCurrentTooLowError(){
  return false;
  //CONSOLE.print(motorRightPWMCurr);
  //CONSOLE.print(",");
  //CONSOLE.println(motorRightSenseLP);
  if  (    ( (fabs(motorMowPWMCurr) > 50) /*&& (abs(motorMowPWMCurrLP) > 50)*/ && (motorMowSenseLP < MOW_TOO_LOW_CURRENT)) 
        ||  ( (fabs(motorLeftPWMCurr) > 50) /*&& (abs(motorLeftPWMCurrLP) > 50)*/ && (motorLeftSenseLP < MOTOR_TOO_LOW_CURRENT))    
        ||  ( (fabs(motorRightPWMCurr) > 50) /*&& (abs(motorRightPWMCurrLP) > 50)*/ && (motorRightSenseLP < MOTOR_TOO_LOW_CURRENT))  ){        
    // at least one motor is not consuming current      
    // first try reovery, then indicate a motor error to the robot control (so it can try an obstacle avoidance)    
    CONSOLE.print("ERROR: motor current too low: pwm (left,right,mow)=");
    CONSOLE.print(motorLeftPWMCurr);
    CONSOLE.print(",");
    CONSOLE.print(motorRightPWMCurr);
    CONSOLE.print(",");
    CONSOLE.print(motorMowPWMCurr);
    CONSOLE.print("  average current amps (left,right,mow)=");
    CONSOLE.print(motorLeftSenseLP);
    CONSOLE.print(",");
    CONSOLE.print(motorRightSenseLP);
    CONSOLE.print(",");
    CONSOLE.println(motorMowSenseLP);
    return true;
  }
  return false;
}


// check motor driver (signal) faults
bool Motor::checkFault() {
  bool fault = false;
  bool leftFault = false;
  bool rightFault = false;
  bool mowFault = false;
  if (ENABLE_FAULT_DETECTION){    
    motorDriver.getMotorFaults(leftFault, rightFault, mowFault);
  }
  if (leftFault) {
    CONSOLE.println("Error: motor driver left signaled fault");
    fault = true;
  }
  if  (rightFault) {
    CONSOLE.println("Error: motor driver right signaled fault"); 
    fault = true;
  }
  if (mowFault) {
    CONSOLE.println("Error: motor driver mow signaled fault");
    fault = true;
  }
  return fault;
}


// check odometry errors
bool Motor::checkOdometryError() {
  if (ENABLE_ODOMETRY_ERROR_DETECTION && false){
    if  (   ( (fabs(motorLeftPWMCurr) > 75) /*&& (fabs(motorLeftPWMCurrLP) > 50)*/ && (fabs(motorLeftRpmCurr) < 0.001))    
        ||  ( (fabs(motorRightPWMCurr) > 75) /*&& (fabs(motorRightPWMCurrLP) > 50)*/ && (fabs(motorRightRpmCurr) < 0.001))  )
    {               
      // odometry error
      CONSOLE.print("ERROR: odometry error - rpm too low (left, right)=");
      CONSOLE.print(motorLeftRpmCurr);
      CONSOLE.print(",");
      CONSOLE.println(motorRightRpmCurr);     
      return true;        
    }
  }
  return false;
}


// check motor overload
void Motor::checkOverload(){
  motorLeftOverload = (motorLeftSenseLP > MOTOR_OVERLOAD_CURRENT);
  motorRightOverload = (motorRightSenseLP > MOTOR_OVERLOAD_CURRENT);
  motorMowOverload = (motorMowSenseLP > MOW_OVERLOAD_CURRENT);
  if (motorLeftOverload || motorRightOverload || motorMowOverload){
    if (motorOverloadDuration == 0){
      CONSOLE.print("ERROR motor overload (average current too high) - duration=");
      CONSOLE.print(motorOverloadDuration);
      CONSOLE.print("  avg current amps (left,right,mow)=");
      CONSOLE.print(motorLeftSenseLP);
      CONSOLE.print(",");
      CONSOLE.print(motorRightSenseLP);
      CONSOLE.print(",");
      CONSOLE.println(motorMowSenseLP);
    }
    motorOverloadDuration += 20;     
  } else {
    motorOverloadDuration = 0;
  }
}


// check mow rpm fault
bool Motor::checkMowRpmFault(){
  if (ENABLE_RPM_FAULT_DETECTION && false){
    if  ( (fabs(motorMowPWMCurr) > 75) /*&& (abs(motorMowPWMCurrLP) > 50)*/ && (fabs(motorMowRpmCurr) < 10.0)) {        
      CONSOLE.print("ERROR: mow motor, average rpm too low: pwm=");
      CONSOLE.print(motorMowPWMCurr);    
      CONSOLE.print("  rpm=");
      CONSOLE.print(motorMowRpmCurr);
      CONSOLE.println("  (NOTE: choose ENABLE_RPM_FAULT_DETECTION=false in config.h, if your mowing motor has no rpm sensor!)");
      return true;
    }
  }  
  return false;
}

void Motor::dumpOdoTicks(int seconds){
  int ticksLeft=0;
  int ticksRight=0;
  int ticksMow=0;
  motorDriver.getMotorEncoderTicks(ticksLeft, ticksRight, ticksMow);  
  motorLeftTicks += ticksLeft;
  motorRightTicks += ticksRight;
  motorMowTicks += ticksMow;
  CONSOLE.print("t=");
  CONSOLE.print(seconds);
  CONSOLE.print("  ticks Left=");
  CONSOLE.print(motorLeftTicks);  
  CONSOLE.print("  Right=");
  CONSOLE.print(motorRightTicks);             
  CONSOLE.print("  current Left=");
  CONSOLE.print(motorLeftSense);
  CONSOLE.print("  Right=");
  CONSOLE.print(motorRightSense);
  CONSOLE.println();               
}


void Motor::test() { }

void Motor::plot() { }
