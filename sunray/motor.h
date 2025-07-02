// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)

#ifndef MOTOR_H
#define MOTOR_H

#include "pid.h"
#include "config.h"


// selected motor
enum MotorSelect {MOTOR_LEFT, MOTOR_RIGHT, MOTOR_MOW} ;
typedef enum MotorSelect MotorSelect;


class Motor {
  public:
    float wheelBaseCm;  // wheel-to-wheel diameter
    int wheelDiameter;   // wheel diameter (mm)
    int ticksPerRevolution; // ticks per revolution
    float ticksPerCm;  // ticks per cm
    bool toggleMowDir; // toggle mowing motor direction each mow motor start?    
    bool motorLeftSwapDir;
    bool motorRightSwapDir;
    bool motorError;
    bool motorLeftOverload; 
    bool motorRightOverload; 
    bool motorMowOverload; 
    bool tractionMotorsEnabled;       
    bool enableMowMotor;
    bool motorMowForwardSet; 
    bool odometryError;    
    unsigned long motorOverloadDuration; // accumulated duration (ms)
    int pwmMax;
    int pwmMaxMow;  
    float mowMotorCurrentAverage;
    float currentFactor;
    bool pwmSpeedCurveDetection;
    unsigned long motorLeftTicks;
    unsigned long motorRightTicks;
    unsigned long motorMowTicks;    
    float linearSpeedSet; // m/s
    float angularSpeedSet; // rad/s
    float motorLeftSense; // left motor current (amps)
    float motorRightSense; // right  motor current (amps)
    float motorMowSense;  // mower motor current (amps)         
    float motorLeftSenseLP; // left motor current (amps, low-pass)
    float motorRightSenseLP; // right  motor current (amps, low-pass)
    float motorMowSenseLP;  // mower motor current (amps, low-pass)      
    float motorMowSenseFLP;  // mower motor current (amps, fast low-pass)   
    float motorsSenseLP; // all motors current (amps, low-pass)
    float motorLeftSenseLPNorm; 
    float motorRightSenseLPNorm;
    unsigned long motorMowSpinUpTime;
    bool motorRecoveryState; 
    PID motorLeftPID;
    PID motorRightPID;   
    PIDv1 motorLeftPIDv1;
    PIDv1 motorRightPIDv1;   
  #if MOW_RPM_CONTROL
    PID motorMowPID; 
  #endif
    void begin();
    void run();      
    void test();
    void plot();
    void enableTractionMotors(bool enable);
    void setLinearAngularSpeed(float linear, float angular, bool useLinearRamp = true, bool useAngularRamp = false);
    void setMowState(bool switchOn);   
    void setMowMaxPwm( int val );
    void stopImmediately(bool includeMowerMotor);
    float motorLeftRpmCurr;
    float motorRightRpmCurr;
    float motorMowRpmCurr;    
  protected: 
    float motorLeftRpmSet; // set speed
    float motorRightRpmSet;  
    float motorMowRpmCurrLP;    
    float motorMowPWMSet;  
    float motorMowPWMCurr; 
    float motorLeftPWMCurr;
    float motorRightPWMCurr;    
    /*float motorMowPWMCurrLP; 
    float motorLeftPWMCurrLP;
    float motorRightPWMCurrLP; */   
    unsigned long lastControlTime;    
    //unsigned long nextSenseTime;          
    bool recoverMotorFault;
    int recoverMotorFaultCounter;
    unsigned long nextRecoverMotorFaultTime;
    int motorLeftTicksZero;    
    int motorRightTicksZero;           
    bool setLinearAngularSpeedTimeoutActive;
    unsigned long setLinearAngularSpeedTimeout;    
    void speedPWM ( int pwmLeft, int pwmRight, int pwmMow );
    void control(bool updateLeft, bool updateRight, bool updateMow);    
    bool checkFault();
    void checkOverload();
    bool checkOdometryError();
    bool checkMowRpmFault();
    bool checkCurrentTooHighError();    
    bool checkCurrentTooLowError();
    void sense();
    void dumpOdoTicks(int seconds);    
};


#endif
