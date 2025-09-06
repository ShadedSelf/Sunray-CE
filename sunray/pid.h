#ifndef PID_H
#define PID_H

#include <Arduino.h>


/*
  digital PID controller
*/

class PID
{
  public:
    PID();
    PID(float Kp, float Ki, float Kd);
    void reset(void);
    float compute();
    double TaMax; // maximum expected sample time
    double Ta; // sampling time	
    float w; // set value
    float x; // current value
    float esum; // error sum
    float eold; // last error
    float y;   // control output
    float y_min; // minimum control output
    float y_max; // maximum control output
    float max_output; // maximum output 
    float Kp;   // proportional control
    float Ki;   // integral control
    float Kd;   // differential control
    unsigned long lastControlTime;
    unsigned long consoleWarnTimeout;
};


class VelocityPID
{
  public:
    VelocityPID();
    VelocityPID(float Kp, float Ki, float Kd);
    float compute();
    double Ta; // sampling time 
    float w; // set value
    float x; // current value
    float eold1; // last error
    float eold2; // error n-2   
    int y;   // control output
    int yold;   // last control output    
    int y_min; // minimum control output
    int y_max; // maximum control output
    int max_output; // maximum output 
    float Kp;   // proportional control
    float Ki;   // integral control
    float Kd;   // differential control
    unsigned long lastControlTime;
};



#endif

#ifndef PID_v1_h
#define PID_v1_h
#define LIBRARY_VERSION	1.2.1

class PIDv1
{


  public:

  //Constants used in some of the functions below
  #define AUTOMATIC	1
  #define MANUAL	0
  #define DIRECT  0
  #define REVERSE  1
  #define P_ON_M 0
  #define P_ON_E 1

  //commonly used functions **************************************************************************
    void Init(float*, float*, float*,        // * constructor.  links the PID to the Input, Output, and 
        float, float, float, int, int);//   Setpoint.  Initial tuning parameters are also set here.
                                          //   (overload for specifying proportional mode)

    void Init(float*, float*, float*,        // * constructor.  links the PID to the Input, Output, and 
        float, float, float, int);     //   Setpoint.  Initial tuning parameters are also set here
	
    void SetMode(int Mode);               // * sets PID to either Manual (0) or Auto (non-0)

    float Compute();                       // * performs the PID calculation.  it should be
                                          //   called every time loop() cycles. ON/OFF and
                                          //   calculation frequency can be set using SetMode
                                          //   SetSampleTime respectively

    void SetOutputLimits(float, float); // * clamps the output to a specific range. 0-255 by default, but
										                      //   it's likely the user will want to change this depending on
										                      //   the application
	


  //available but not commonly used functions ********************************************************
    void SetTunings(float, float,       // * While most users will set the tunings once in the 
                    float);         	    //   constructor, this function gives the user the option
                                          //   of changing tunings during runtime for Adaptive control
    void SetTunings(float, float,       // * overload for specifying proportional mode
                    float, int);
    void SetP(float);
    void SetI(float);
    void SetD(float);

	void SetControllerDirection(int);	  // * Sets the Direction, or "Action" of the controller. DIRECT
										  //   means the output will increase when error is positive. REVERSE
										  //   means the opposite.  it's very unlikely that this will be needed
										  //   once it is set in the constructor.
										  
										  
										  
  //Display functions ****************************************************************
	float GetKp();						  // These functions query the pid for interal values.
	float GetKi();						  //  they were created mainly for the pid front-end,
	float GetKd();						  // where it's important to know what is actually 
	int GetMode();						  //  inside the PID.
	int GetDirection();					  //

  private:
	void Initialize();
	
	float dispKp;				// * we'll hold on to the tuning parameters in user-entered 
	float dispKi;				//   format for display purposes
	float dispKd;				//
    
	float kp;                  // * (P)roportional Tuning Parameter
    float ki;                  // * (I)ntegral Tuning Parameter
    float kd;                  // * (D)erivative Tuning Parameter

	int controllerDirection;
	int pOn;

    float *myInput;              // * Pointers to the Input, Output, and Setpoint variables
    float *myOutput;             //   This creates a hard link between the variables and the 
    float *mySetpoint;           //   PID, freeing the user from having to constantly tell us
                                  //   what these values are.  with pointers we'll just know.
			  
	unsigned long lastTime;
	float outputSum, lastInput;

	unsigned long SampleTime;
	float outMin, outMax;
	bool inAuto, pOnE;
};
#endif