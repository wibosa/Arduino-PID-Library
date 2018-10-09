/**********************************************************************************************
 * Arduino PPID Library - Version 1.2.1
 * Pseudo-PID Controller: Design, Tuning and Applications 
 * //by Brett Beauregard <br3ttb@gmail.com> brettbeauregard.com
 *
 * This Library is licensed under the MIT License
 **********************************************************************************************/

#if ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

#include <PPID_v1.h>

/*Constructor (...)*********************************************************
 *    The parameters specified here are those for for which we can't set up
 *    reliable defaults, so we need to have the user set them.
 ***************************************************************************/
PPID::PPID(double* Input, double* Output, double* Setpoint,
        double Kc, int POn, int ControllerDirection)
{
    myOutput = Output;
    myInput = Input;
    mySetpoint = Setpoint;
    inAuto = false;
	// initialise shift array
	Yr=*mySetpoint;
	Yt2 = *myInput;
	Yt1 = *myInput;
	Yt = *myInput;
    PPID::SetOutputLimits(0, 255);				//default output limit corresponds to
												//the arduino pwm limits

    //SampleTime = 100;							//default Controller Sample Time is 0.1 seconds

    PPID::SetControllerDirection(ControllerDirection);
    PPID::SetTunings(Kc, POn);

    //lastTime = millis()-SampleTime;
}

/*Constructor (...)*********************************************************
 *    To allow backwards compatability for v1.1, or for people that just want
 *    to use Proportional on Error without explicitly saying so
 ***************************************************************************/

PPID::PPID(double* Input, double* Output, double* Setpoint, double Kc, int ControllerDirection)
    :PPID::PPID(Input, Output, Setpoint, Kc, P_ON_E, ControllerDirection)
{

}


/* Compute() **********************************************************************
 *     This, as they say, is where the magic happens.  this function should be called
 *   every time "void loop()" executes.  the function will decide for itself whether a new
 *   pid Output needs to be computed.  returns true when the output is computed,
 *   false when nothing has been done.
 **********************************************************************************/
bool PPID::Compute()
{
   if(!inAuto) return false;
      //double input = *myInput;

  /*PPID control law:
	  u(t)=u(t-1) + Kpp* ( 0.1 * Yref - 3.6* y(t) + 6* y(t -1) - 2.5* y(t-1));
                */	
	Yr  = *mySetpoint;
	Yt2 = Yt1;
	Yt1 = Yt;
	Yt  = *myInput;
			
	double output = *myOutput + kc *( 0.1 *Yr - 3.6 * Yt + 6 * Yt1 -2.5 * Yt2);
				
	if(output > outMax) output = outMax;
    else if(output < outMin) output = outMin;
	*myOutput = output;
    
	return true;
}

/* SetTunings(...)*************************************************************
 * This function allows the controller's dynamic performance to be adjusted.
 * it's called automatically from the constructor, but tunings can also
 * be adjusted on the fly during normal operation
 ******************************************************************************/
void PPID::SetTunings(double Kc, int POn)
{
   if (Kc<0 ) return;

   pOn = POn;
   pOnE = POn == P_ON_E;

   dispKc = Kc;
   
   //double SampleTimeInSec = ((double)SampleTime)/1000;
   kc = Kc;

  if(controllerDirection ==REVERSE)
   {
      kc = (0 - kc);
   }
}

/* SetTunings(...)*************************************************************
 * Set Tunings using the last-rembered POn setting
 ******************************************************************************/
void PPID::SetTunings(double Kc){
    SetTunings(Kc); 
}

/* SetOutputLimits(...)****************************************************
 *     This function will be used far more often than SetInputLimits.  while
 *  the input to the controller will generally be in the 0-1023 range (which is
 *  the default already,)  the output will be a little different.  maybe they'll
 *  be doing a time window and will need 0-8000 or something.  or maybe they'll
 *  want to clamp it from 0-125.  who knows.  at any rate, that can all be done
 *  here.
 **************************************************************************/
void PPID::SetOutputLimits(double Min, double Max)
{
   if(Min >= Max) return;
   outMin = Min;
   outMax = Max;

   if(inAuto)
   {
	   if(*myOutput > outMax) *myOutput = outMax;
	   else if(*myOutput < outMin) *myOutput = outMin;

   }
}

/* SetMode(...)****************************************************************
 * Allows the controller Mode to be set to manual (0) or Automatic (non-zero)
 * when the transition from manual to auto occurs, the controller is
 * automatically initialized
 ******************************************************************************/
void PPID::SetMode(int Mode)
{
    bool newAuto = (Mode == AUTOMATIC);
    if(newAuto && !inAuto)
    {  /*we just went from manual to auto*/
        PPID::Initialize();
    }
    inAuto = newAuto;
}

/* Initialize()****************************************************************
 *	does all the things that need to happen to ensure a bumpless transfer
 *  from manual to automatic mode.
 ******************************************************************************/
void PPID::Initialize()
{
   //outputSum = *myOutput;
   //stInput = *myInput;
   //if(outputSum > outMax) outputSum = outMax;
   //else if(outputSum < outMin) outputSum = outMin;
}

/* SetControllerDirection(...)*************************************************
 * The PPID will either be connected to a DIRECT acting process (+Output leads
 * to +Input) or a REVERSE acting process(+Output leads to -Input.)  we need to
 * know which one, because otherwise we may increase the output when we should
 * be decreasing.  This is called from the constructor.
 ******************************************************************************/
void PPID::SetControllerDirection(int Direction)
{
   if(inAuto && Direction !=controllerDirection)
   {
	    kc = (0 - kc);
   }
   controllerDirection = Direction;
}

/* Status Funcions*************************************************************
 * Just because you set the Kp=-1 doesn't mean it actually happened.  these
 * functions query the internal state of the PPID.  they're here for display
 * purposes.  this are the functions the PPID Front-end uses for example
 ******************************************************************************/
double PPID::GetKc(){ return  dispKc; }
int PPID::GetMode(){ return  inAuto ? AUTOMATIC : MANUAL;}
int PPID::GetDirection(){ return controllerDirection;}

