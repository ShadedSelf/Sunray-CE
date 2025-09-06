/*  
   How to find out P,I,D:
    1. Increase P until system starts to oscillate
    2. Set I =0.6 * P and D = 0.125 * P 
   
*/

#include "pid.h"
#include "config.h"

PID::PID()
{
  consoleWarnTimeout = 0;
  lastControlTime = 0;
}
    
PID::PID(float Kp, float Ki, float Kd){
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;
}


void PID::reset(void) {
  this->eold = 0;
  this->esum = 0;
  lastControlTime = micros();
}

float PID::compute()
{
  unsigned long now = micros();
  Ta = (double)(now - lastControlTime) / 1000000.0;
  lastControlTime = now;
  
  Ta = max(Ta, 0.000001);

  // compute error
  float e = (w - x);
  // integrate error
  esum += e;
  // anti wind-up
  if (esum < -max_output)  esum = -max_output;
  if (esum > max_output)  esum = max_output;

  y = Kp * e
    + Ki * Ta * esum
    + Kd/Ta * (e - eold);

  eold = e;

  // restrict output to min/max
  if (y > y_max) y = y_max;
  if (y < y_min) y = y_min;

  return y;
}


// ---------------------------------

VelocityPID::VelocityPID()
{
}
    
VelocityPID::VelocityPID(float Kp, float Ki, float Kd){
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;
}


float VelocityPID::compute()
{   
  unsigned long now = micros();
  Ta = ((now - lastControlTime) / 1000000.0);
  lastControlTime = now;
  if (Ta > 1.0) Ta = 1.0;   // should only happen for the very first call

  // compute error
  float e = (w - x);

  // compute max/min output
  if (w < 0) { y_min = -max_output; y_max = 0; }
  if (w > 0) { y_min = 0; y_max = max_output; }     

  y = yold
      + Kp * (e - eold1)
      + Ki * Ta * e
      + Kd/Ta * (e - 2* eold1 + eold2);
     
  // restrict output to min/max 
  if (y > y_max) y = y_max;
  if (y < y_min) y = y_min; 

  // save variable for next time
  eold2 = eold1;
  eold1 = e;
  yold = y ;  
  
  return y;
}






/**********************************************************************************************
 * Arduino PID Library - Version 1.2.1
 * by Brett Beauregard <br3ttb@gmail.com> brettbeauregard.com
 *
 * This Library is licensed under the MIT License
 **********************************************************************************************/

/*Constructor (...)*********************************************************
 *    The parameters specified here are those for for which we can't set up
 *    reliable defaults, so we need to have the user set them.
 ***************************************************************************/
void PIDv1::Init(float* Input, float* Output, float* Setpoint,
        float Kp, float Ki, float Kd, int POn, int ControllerDirection)
{
    myOutput = Output;
    myInput = Input;
    mySetpoint = Setpoint;
    inAuto = false;

    PIDv1::SetOutputLimits(0, 255);
    PIDv1::SetControllerDirection(ControllerDirection);
    PIDv1::SetTunings(Kp, Ki, Kd, POn);

    lastTime = micros();
}

/*Constructor (...)*********************************************************
 *    To allow backwards compatability for v1.1, or for people that just want
 *    to use Proportional on Error without explicitly saying so
 ***************************************************************************/

void PIDv1::Init(float* Input, float* Output, float* Setpoint,
        float Kp, float Ki, float Kd, int ControllerDirection)
{   
      PIDv1::Init(Input, Output, Setpoint, Kp, Ki, Kd, P_ON_E, ControllerDirection);
}


/* Compute() **********************************************************************
 *     This, as they say, is where the magic happens.  this function should be called
 *   every time "void loop()" executes.  the function will decide for itself whether a new
 *   pid Output needs to be computed.  returns true when the output is computed,
 *   false when nothing has been done.
 **********************************************************************************/
float PIDv1::Compute()
{
    if(!inAuto)
      return 0.0;

    unsigned long now = micros();
    if (now - lastTime < 1000)
      return 0.0;

    double timeChange = (now - lastTime) / 1000000.0;

    /*Compute all the working error variables*/
    float input = *myInput;
    float error = (*mySetpoint - input);
    float dInput = (input - lastInput);

    outputSum *= max(1.0 - timeChange * 0.01, 0.0);
    //outputSum *= exp(-timeChange);
    outputSum += (ki * timeChange * error);

    /*Add Proportional on Measurement, if P_ON_M is specified*/
    if(!pOnE) outputSum -= kp * dInput;

    outputSum = constrain(outputSum, outMin, outMax);

    /*Add Proportional on Error, if P_ON_E is specified*/
    float output;
    if(pOnE) output = kp * error;
    else output = 0;

    /*Compute Rest of PID Output*/
    output += outputSum - kd / timeChange * dInput;

    output = constrain(outputSum, outMin, outMax);
    //*myOutput = output;

    /*Remember some variables for next time*/
    lastInput = input;
    lastTime = now;

    return output;
}

/* SetTunings(...)*************************************************************
 * This function allows the controller's dynamic performance to be adjusted.
 * it's called automatically from the constructor, but tunings can also
 * be adjusted on the fly during normal operation
 ******************************************************************************/
void PIDv1::SetTunings(float Kp, float Ki, float Kd, int POn)
{
   if (Kp<0 || Ki<0 || Kd<0) return;

   pOn = POn;
   pOnE = POn == P_ON_E;

   dispKp = Kp; dispKi = Ki; dispKd = Kd;

   kp = Kp;
   ki = Ki;
   kd = Kd;

  if(controllerDirection ==REVERSE)
   {
      kp = (0 - kp);
      ki = (0 - ki);
      kd = (0 - kd);
   }
}

/* SetTunings(...)*************************************************************
 * Set Tunings using the last-rembered POn setting
 ******************************************************************************/
void PIDv1::SetTunings(float Kp, float Ki, float Kd){
    SetTunings(Kp, Ki, Kd, pOn); 
}

void PIDv1::SetP(float Kp){
    kp = dispKp = Kp;
}
void PIDv1::SetI(float Ki){
    ki = dispKi = Ki;
}
void PIDv1::SetD(float Kd){
    kd = dispKd = Kd;
}

/* SetOutputLimits(...)****************************************************
 *     This function will be used far more often than SetInputLimits.  while
 *  the input to the controller will generally be in the 0-1023 range (which is
 *  the default already,)  the output will be a little different.  maybe they'll
 *  be doing a time window and will need 0-8000 or something.  or maybe they'll
 *  want to clamp it from 0-125.  who knows.  at any rate, that can all be done
 *  here.
 **************************************************************************/
void PIDv1::SetOutputLimits(float Min, float Max)
{
   if(Min >= Max) return;
   outMin = Min;
   outMax = Max;

   if(inAuto)
   {
	   if(*myOutput > outMax) *myOutput = outMax;
	   else if(*myOutput < outMin) *myOutput = outMin;

	   if(outputSum > outMax) outputSum= outMax;
	   else if(outputSum < outMin) outputSum= outMin;
   }
}

/* SetMode(...)****************************************************************
 * Allows the controller Mode to be set to manual (0) or Automatic (non-zero)
 * when the transition from manual to auto occurs, the controller is
 * automatically initialized
 ******************************************************************************/
void PIDv1::SetMode(int Mode)
{
    bool newAuto = (Mode == AUTOMATIC);
    if(newAuto && !inAuto)
    {  /*we just went from manual to auto*/
        PIDv1::Initialize();
    }
    inAuto = newAuto;
}

/* Initialize()****************************************************************
 *	does all the things that need to happen to ensure a bumpless transfer
 *  from manual to automatic mode.
 ******************************************************************************/
void PIDv1::Initialize()
{
   outputSum = *myOutput;
   lastInput = *myInput;
   if(outputSum > outMax) outputSum = outMax;
   else if(outputSum < outMin) outputSum = outMin;
}

/* SetControllerDirection(...)*************************************************
 * The PID will either be connected to a DIRECT acting process (+Output leads
 * to +Input) or a REVERSE acting process(+Output leads to -Input.)  we need to
 * know which one, because otherwise we may increase the output when we should
 * be decreasing.  This is called from the constructor.
 ******************************************************************************/
void PIDv1::SetControllerDirection(int Direction)
{
   if(inAuto && Direction !=controllerDirection)
   {
	    kp = (0 - kp);
      ki = (0 - ki);
      kd = (0 - kd);
   }
   controllerDirection = Direction;
}

/* Status Funcions*************************************************************
 * Just because you set the Kp=-1 doesn't mean it actually happened.  these
 * functions query the internal state of the PID.  they're here for display
 * purposes.  this are the functions the PID Front-end uses for example
 ******************************************************************************/
float PIDv1::GetKp(){ return  dispKp; }
float PIDv1::GetKi(){ return  dispKi;}
float PIDv1::GetKd(){ return  dispKd;}
int PIDv1::GetMode(){ return  inAuto ? AUTOMATIC : MANUAL;}
int PIDv1::GetDirection(){ return controllerDirection;}
