#ifndef PPID_v1_h
#define PPID_v1_h
#define LIBRARY_VERSION	1.2.1

class PPID
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
    PPID(double*, double*, double*,        // * constructor.  links the PPID to the Input, Output, and 
        double, int, int);//   Setpoint.  Initial tuning parameters are also set here.
                                          //   (overload for specifying proportional mode)

    PPID(double*, double*, double*,        // * constructor.  links the PPID to the Input, Output, and 
        double,  int);     //   Setpoint.  Initial tuning parameters are also set here
	
    void SetMode(int Mode);               // * sets PPID to either Manual (0) or Auto (non-0)

    bool Compute();                       // * performs the PPID calculation.  it should be
                                          //   called every time loop() cycles. ON/OFF and
                                          //   calculation frequency can be set using SetMode
                                          //   SetSampleTime respectively

    void SetOutputLimits(double, double); // * clamps the output to a specific range. 0-255 by default, but
										                      //   it's likely the user will want to change this depending on
										                      //   the application
	


  //available but not commonly used functions ********************************************************
    void SetTunings(double);         	    //   constructor, this function gives the user the option
                                          //   of changing tunings during runtime for Adaptive control
    void SetTunings(double, int);         	  

	void SetControllerDirection(int);	  // * Sets the Direction, or "Action" of the controller. DIRECT
										  //   means the output will increase when error is positive. REVERSE
										  //   means the opposite.  it's very unlikely that this will be needed
										  //   once it is set in the constructor.
    void SetSampleTime(int);              // * sets the frequency, in Milliseconds, with which 
                                          //   the PPID calculation is performed.  default is 100
										  
										  
										  
  //Display functions ****************************************************************
	double GetKc();						  // These functions query the pid for interal values.
	int GetMode();						  //  inside the PPID.
	int GetDirection();					  //

  private:
	void Initialize();
	
	double dispKc;				// * we'll hold on to the tuning parameters in user-entered 
    
	double kc;                  // * (P)roportional Tuning Parameter

	int controllerDirection;
	int pOn;

    double *myInput;              // * Pointers to the Input, Output, and Setpoint variables
    double *myOutput;             //   This creates a hard link between the variables and the 
    double *mySetpoint;           //   PPID, freeing the user from having to constantly tell us
                                  //   what these values are.  with pointers we'll just know.
			  
	//unsigned long lastTime;
	//double outputSum, lastInput;

	//unsigned long SampleTime;
	double outMin, outMax;
	bool inAuto, pOnE;
	double 		Yr;			// PPID shift	
	double 		Yt2;			// PPID shift
	double 		Yt1;			// PPID shift	
	double 		Yt;			// PPID shift

};
#endif

