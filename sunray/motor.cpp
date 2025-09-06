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
  pwmMaxMow = MAX_MOW_PWM;
  
	wheelBaseCm = WHEEL_BASE_CM;    // wheel-to-wheel distance (cm) 36
  wheelDiameter = WHEEL_DIAMETER; // wheel diameter (mm)

  // Motor PIDs -------
  motorLeftPIDv1.Init(&motorLeftRpmCurr, &motorLeftPWMCurr, &motorLeftRpmSet,
    MOTOR_PID_KP, MOTOR_PID_KI, MOTOR_PID_KD,
    P_ON_E, DIRECT);
  motorLeftPIDv1.SetOutputLimits(-MAX_GEAR_PWM_L, MAX_GEAR_PWM_L);
  motorLeftPIDv1.SetMode(AUTOMATIC);
  
  motorRightPIDv1.Init(&motorRightRpmCurr, &motorRightPWMCurr, &motorRightRpmSet,
    MOTOR_PID_KP, MOTOR_PID_KI, MOTOR_PID_KD,
    P_ON_E, DIRECT);
  motorRightPIDv1.SetOutputLimits(-MAX_GEAR_PWM_R, MAX_GEAR_PWM_R);
  motorRightPIDv1.SetMode(AUTOMATIC);


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
  motorDriver.getMotorCurrent(motorLeftSense, motorRightSense, motorMowSense);
  motorLeftSenseLP = motorLeftSense;
  motorRightSenseLP = motorRightSense;
  motorMowSenseLP = motorMowSense;  
  motorMowSenseFLP = motorMowSense;
  motorsSenseLP = motorLeftSense + motorRightSense + motorMowSense;

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
  if (motorLeftSwapDir)  pwmLeft  *= -1;
  if (motorRightSwapDir) pwmRight *= -1;

  // ensure pwm is lower than Max
  pwmLeft  = constrain(pwmLeft , -MAX_GEAR_PWM_L, MAX_GEAR_PWM_L);
  pwmRight = constrain(pwmRight, -MAX_GEAR_PWM_R, MAX_GEAR_PWM_R); 
  pwmMow   = constrain(pwmMow  , -pwmMaxMow     , pwmMaxMow     ); 
  
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
void Motor::setLinearAngularSpeed(float linear, float angular, float linearAcceleration, float angularAcceleration)
{
  accelerationTimer.update();

  setLinearAngularSpeedTimeout = millis() + 1000;
  setLinearAngularSpeedTimeoutActive = true;

  // linear
  if (linearAcceleration < 0.0001)
    linearSpeedSet = linear;
  else
    linearSpeedSet = accelerationTimer.accelerate(linearSpeedSet, linear, linearAcceleration);

  // angular
  if (angularAcceleration < 0.0001)
    angularSpeedSet = angular;
  else
    angularSpeedSet = accelerationTimer.accelerate(angularSpeedSet, angular, angularAcceleration);

  // linear + angular
  float rspeed = linearSpeedSet + angularSpeedSet * (wheelBaseCm / 100.0 / 2.0);          
  float lspeed = linearSpeedSet - angularSpeedSet * (wheelBaseCm / 100.0 / 2.0);   

  // RPM = V / (2*PI*r) * 60
  motorRightRpmSet = rspeed / (PI * (wheelDiameter / 1000.0)) * 60.0;   
  motorLeftRpmSet  = lspeed / (PI * (wheelDiameter / 1000.0)) * 60.0;
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
  //motorLeftPID.reset();
  //motorRightPID.reset();
#if MOW_RPM_CONTROL
  motorMowPID.reset(); 
#endif
  // reset unread encoder ticks
  int ticksLeft, ticksRight, ticksMow;
  motorDriver.getMotorEncoderTicks(ticksLeft, ticksRight, ticksMow);        
}

unsigned long prevTimeLeft, prevTimeRight;
void Motor::run()
{   
    // angular timeout
    if (setLinearAngularSpeedTimeoutActive && millis() > setLinearAngularSpeedTimeout)
    {
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
    if (nextRecoverMotorFaultTime != 0)
    {
      if (millis() > nextRecoverMotorFaultTime)
      {
        if (recoverMotorFault)
        {
          nextRecoverMotorFaultTime = millis() + 10000;
          recoverMotorFaultCounter++;                                               
          CONSOLE.print("motor fault recover counter ");
          CONSOLE.println(recoverMotorFaultCounter);
          motorDriver.resetMotorFaults();
          recoverMotorFault = false;  
          if (recoverMotorFaultCounter >= 10)
          { // too many successive motor faults
            //stopImmediately();
            CONSOLE.println("ERROR: motor recovery failed");
            recoverMotorFaultCounter = 0;
            motorError = true;
          }
        }
        else
        {
          CONSOLE.println("resetting recoverMotorFaultCounter");
          recoverMotorFaultCounter = 0;
          nextRecoverMotorFaultTime = 0;
          motorRecoveryState = false;
        }        
      }
    }
  //}
  

  // -- Calculate motor RPM --
  
  int ticksLeft, ticksRight, ticksMow;
  motorDriver.getMotorEncoderTicks(ticksLeft, ticksRight, ticksMow);

  // does pid need to update
  bool shouldUpdateLeft  = ticksLeft  != 0;
  bool shouldUpdateRight = ticksRight != 0;
  
  motorLeftTicks  += ticksLeft  * (int)sign(motorLeftPWMCurr);
  motorRightTicks += ticksRight * (int)sign(motorRightPWMCurr);
  motorMowTicks   += ticksMow   * (int)sign(motorMowPWMCurr);

#if RESPONSIVE_RPM
  unsigned long timeLeft, timeRight, timeMow;
  motorDriver.getMotorTickTime(timeLeft, timeRight, timeMow);

  // should PID controller run
  shouldUpdateLeft  = timeLeft  != prevTimeLeft  || shouldUpdateLeft;
  shouldUpdateRight = timeRight != prevTimeRight || shouldUpdateRight;
  prevTimeLeft  = timeLeft;
  prevTimeRight = timeRight;

  // calculate speed via tick time
  motorLeftRpmCurr =  (60000000.0 / (timeLeft  * TICKS_PER_REVOLUTION_L)) * sign(motorLeftPWMCurr);
  motorRightRpmCurr = (60000000.0 / (timeRight * TICKS_PER_REVOLUTION_R)) * sign(motorRightPWMCurr);
  motorMowRpmCurr =   (60000000.0 / (timeMow   * 6))                      * sign(motorMowPWMCurr);
# else
  // timeframes
  unsigned long currTime = micros();
  float deltaControlTimeSec =  (double)(currTime - lastControlTime) / 1000.0 / 1000.0;
  lastControlTime = currTime;
  // calculate speed via tick count
  motorLeftRpmCurr =  60.0 * ( ((float)ticksLeft)  / ((float)TICKS_PER_REVOLUTION_L) ) / deltaControlTimeSec;
  motorRightRpmCurr = 60.0 * ( ((float)ticksRight) / ((float)TICKS_PER_REVOLUTION_R) ) / deltaControlTimeSec;
  motorMowRpmCurr =   60.0 * ( ((float)ticksMow)   / ((float)6.0) ) / deltaControlTimeSec; // assuming 6 ticks per revolution
#endif

  // set rpm to 0 if we dont get ticks in 10 cycles
  if (ticksLeft == 0) motorLeftTicksZero++;
  if (ticksLeft != 0) motorLeftTicksZero = 0;
  if (motorLeftTicksZero > 10) { motorLeftRpmCurr = 0; shouldUpdateLeft = true; }

  if (ticksRight == 0) motorRightTicksZero++;
  if (ticksRight != 0) motorRightTicksZero = 0;
  if (motorRightTicksZero > 10) { motorRightRpmCurr = 0; shouldUpdateRight = true; }

  // speed controller
  control(shouldUpdateLeft, shouldUpdateRight, true);
}  

// measure motor currents
Timer currentTimer(MICROS_TIME);
void Motor::sense()
{
  motorDriver.getMotorCurrent(motorLeftSense, motorRightSense, motorMowSense);

  currentTimer.update();

  motorRightSenseLP = currentTimer.lowPass(motorRightSenseLP, motorRightSense, 1.0);
  motorLeftSenseLP  = currentTimer.lowPass(motorLeftSenseLP , motorLeftSense , 1.0);
  motorMowSenseLP   = currentTimer.lowPass(motorMowSenseLP  , motorMowSense  , 1.0);

  motorsSenseLP     = currentTimer.lowPass(motorsSenseLP, motorRightSense + motorLeftSense + motorMowSense, 30.0);

  motorMowSenseFLP  = currentTimer.lowPass(motorMowSenseFLP, motorMowSense, 0.1);
}

Timer motorMowTimer(MICROS_TIME);
Timer debugTimer(MICROS_TIME);
float aa = VELOCITY_COEF_FF_L;
float bb = VELOCITY_COEF_FF_R;
float cc = GRAVITY_COEF_FF_L;
float dd = GRAVITY_COEF_FF_R;
void Motor::control(bool updateLeft, bool updateRight, bool updateMow){
  //########################  Calculate PWM for left driving motor ############################


  //dont let pwm rise if overloaded?
  /*debugTimer.update();
  if (stateOp == OP_MOW)
  { 
    aa = debugTimer.lowPass(aa, min(max(fabs(motorLeftPWMCurr) - FRICTION_FF_L, 0.0) / max(fabs(motorLeftRpmCurr), 0.00001), 10.0), 60.0 * 10.0);
    bb = debugTimer.lowPass(bb, min(max(fabs(motorRightPWMCurr) - FRICTION_FF_R, 0.0) / max(fabs(motorRightRpmCurr), 0.00001), 10.0), 60.0 * 10.0);

    //cc = debugTimer.lowPass(cc, constrain((motorLeftRpmCurr - motorLeftRpmSet) / sin(imuDriver.pitch), -10.0, 10.0), 60.0 * 10.0);
    //dd = debugTimer.lowPass(dd, constrain((motorRightRpmCurr - motorRightRpmSet) / sin(imuDriver.pitch), -10.0, 10.0), 60.0 * 10.0);
    
    float ffPWM = FRICTION_FF_L * sign(motorLeftRpmSet) + VELOCITY_COEF_FF_L * motorLeftRpmSet;
    cc = debugTimer.lowPass(cc, constrain(fabs(fabs(motorLeftPWMCurr) - fabs(ffPWM)) / max(fabs(sin(imuDriver.pitch)), 0.00001), -10.0, 10.0), 60.0 * 10.0);
    ffPWM = FRICTION_FF_R * sign(motorRightRpmSet) + VELOCITY_COEF_FF_R * motorRightRpmSet;
    dd = debugTimer.lowPass(dd, constrain(fabs(fabs(motorRightPWMCurr) - fabs(ffPWM)) / max(fabs(sin(imuDriver.pitch)), 0.00001), -10.0, 10.0), 60.0 * 10.0);
  }


  DEBUG(aa);
  DEBUG(" ");
  DEBUG(bb);
  DEBUG(" ");
  DEBUG(cc);
  DEBUG(" ");
  DEBUGLN(dd);*/

  {
    float kv = 3850.0 / 24.0 * 0.95;
    float kt = 60.0 / (TAU * kv);
    float torque = kt * motorLeftSenseLP * 50.0;
    float power = motorLeftSenseLP * battery.systemVoltage;
    float mechanicalPower = (torque * fabs(motorLeftRpmCurr) * TAU) / 60.0;
    float eff = mechanicalPower / power;

    /*DEBUG("Lpwm:\t");
    DEBUG(motorLeftPWMCurr);
    DEBUG("\tLtqe:\t");
    DEBUG(torque);
    DEBUG("\tLrpm:\t");
    DEBUG(motorLeftRpmCurr);
    DEBUG("\tLAmp:\t");
    DEBUG(motorLeftSenseLP);
    DEBUG("\tLeff:\t");
    DEBUGLN(eff);*/
  }

  {
    float kv = 5000.0 / 30.0 * 0.95;
    float kt = 60.0 / (TAU * kv);
    float torque = kt * motorRightSenseLP * 99.5;
    float power = motorRightSenseLP * battery.systemVoltage;
    float mechanicalPower = (torque * fabs(motorRightRpmCurr) * TAU) / 60.0;
    float eff = mechanicalPower / power;

    /*DEBUG("pwm:\t");
    DEBUG(motorRightPWMCurr);
    DEBUG("\tRtqe:\t");
    DEBUG(torque);
    DEBUG("\tRrpm:\t");
    DEBUG(motorRightRpmCurr);
    DEBUG("\tRAmp:\t");
    DEBUG(motorRightSenseLP);
    DEBUG("\tReff:\t");
    DEBUGLN(eff);*/
  }

  
  #if PRINT_PWM_VALUES
    float e = motorLeftRpmSet - motorLeftRpmCurr;
    DEBUG("RPM error: ");
    DEBUG(e);
    DEBUG(" , Set RPM: ");
    DEBUG(motorLeftRpmSet);
    DEBUG(" ,  motorPWM: ");
    DEBUGLN(motorLeftPWMCurr);
  #endif


  if (updateLeft)
  {
    
    // PID
    float pid = motorLeftPIDv1.Compute();
    
    // Feed-fordward -> Friction + Velocity + Gravity
    float ffPWM = FRICTION_FF_L      * sign(motorLeftRpmSet)
                + VELOCITY_COEF_FF_L * motorLeftRpmSet;
                + GRAVITY_COEF_FF_L  * sign(motorRightRpmSet) * -sin(imuDriver.pitch);
    
    // PID + FF
    float finalPWM = ffPWM + pid;

    // PWM change limiter
    float pwmStep = 1.9; // make acceleration relative to ff_vel_coef?
    motorLeftPWMCurr = constrain(finalPWM, motorLeftPWMCurr - pwmStep, motorLeftPWMCurr + pwmStep);
    motorLeftPWMCurr = constrain(motorLeftPWMCurr, -MAX_GEAR_PWM_L, MAX_GEAR_PWM_L);
  }

  //########################  Calculate PWM for right driving motor ############################
  if (updateRight)
  {
    // PID
    float pid = motorRightPIDv1.Compute();

    // Feed-fordward -> Friction + Velocity + Gravity
    float ffPWM = FRICTION_FF_R      * sign(motorRightRpmSet)
                + VELOCITY_COEF_FF_R * motorRightRpmSet;
                + GRAVITY_COEF_FF_R  * sign(motorRightRpmSet) * -sin(imuDriver.pitch);
    
    // PID + FF
    float finalPWM = ffPWM + pid;

    // PWM change limiter
    float pwmStep = 1.9;
    motorRightPWMCurr = constrain(finalPWM, motorRightPWMCurr - pwmStep, motorRightPWMCurr + pwmStep);
    motorRightPWMCurr = constrain(motorRightPWMCurr, -MAX_GEAR_PWM_R, MAX_GEAR_PWM_R);
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
    motorMowTimer.update();

    // 2 second acceleration
    float a = 0.5 * fabs(motorMowPWMSet);
    motorMowPWMCurr = motorMowTimer.accelerate(motorMowPWMCurr, motorMowPWMSet, a);
  #endif
  }

  //########################  set PWM for all motors ############################
  if (fabs(motorLeftRpmSet)  < 0.001) motorLeftPWMCurr = 0;
  if (fabs(motorRightRpmSet) < 0.001) motorRightPWMCurr = 0;

  if (!tractionMotorsEnabled)
    motorLeftPWMCurr = motorRightPWMCurr = 0;

  speedPWM(motorLeftPWMCurr, motorRightPWMCurr, motorMowPWMCurr);
}


// check if motor current too high
bool Motor::checkCurrentTooHighError()
{
  bool motorLeftFault  = motorLeftSense  > MOTOR_FAULT_CURRENT;
  bool motorRightFault = motorRightSense > MOTOR_FAULT_CURRENT;
  bool motorMowFault   = motorMowSense   > MOW_FAULT_CURRENT;
  
  if (motorLeftFault || motorRightFault || motorMowFault)
  {
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
bool Motor::checkCurrentTooLowError()
{
  return false;
  if ((fabs(motorMowPWMCurr) > 50 && motorMowSenseLP < MOW_TOO_LOW_CURRENT) 
  ||  (fabs(motorLeftPWMCurr) > 50 && motorLeftSenseLP < MOTOR_TOO_LOW_CURRENT)    
  ||  (fabs(motorRightPWMCurr) > 50 && motorRightSenseLP < MOTOR_TOO_LOW_CURRENT))
  {        
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
bool Motor::checkOdometryError()
{
  //if (!ENABLE_ODOMETRY_ERROR_DETECTION )
    return false;

  if ((fabs(motorLeftPWMCurr) > 30.0 && fabs(motorLeftRpmCurr) < 0.001)    
  ||  (fabs(motorRightPWMCurr) > 30.0 && (fabs(motorRightRpmCurr) < 0.001)))
  {               
    // odometry error
    CONSOLE.print("ERROR: odometry error - rpm too low (left, right)=");
    CONSOLE.print(motorLeftRpmCurr);
    CONSOLE.print(",");
    CONSOLE.println(motorRightRpmCurr);     
    return true;        
  }

  return false;
}


// check motor overload
void Motor::checkOverload()
{
  motorLeftOverload  = (motorLeftSenseLP  > MOTOR_OVERLOAD_CURRENT);
  motorRightOverload = (motorRightSenseLP > MOTOR_OVERLOAD_CURRENT);
  motorMowOverload   = (motorMowSenseLP   > MOW_OVERLOAD_CURRENT);

  if (motorLeftOverload || motorRightOverload || motorMowOverload)
  {
    if (motorOverloadDuration == 0)
    {
      CONSOLE.print("ERROR motor overload (average current too high) - duration=");
      CONSOLE.print(motorOverloadDuration);
      CONSOLE.print("  avg current amps (left,right,mow)=");
      CONSOLE.print(motorLeftSenseLP);
      CONSOLE.print(",");
      CONSOLE.print(motorRightSenseLP);
      CONSOLE.print(",");
      CONSOLE.println(motorMowSenseLP);
    }
    motorOverloadDuration += 2;     
  }
  else
    motorOverloadDuration = 0;
}


// check mow rpm fault
bool Motor::checkMowRpmFault()
{
  //if (!ENABLE_RPM_FAULT_DETECTION)
    return false;

  if (fabs(motorMowPWMCurr) > 30.0 && fabs(motorMowRpmCurr) < 0.1)
  {        
    CONSOLE.print("ERROR: mow motor, average rpm too low: pwm=");
    CONSOLE.print(motorMowPWMCurr);    
    CONSOLE.print("  rpm=");
    CONSOLE.print(motorMowRpmCurr);
    CONSOLE.println("  (NOTE: choose ENABLE_RPM_FAULT_DETECTION=false in config.h, if your mowing motor has no rpm sensor!)");
    return true;
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
