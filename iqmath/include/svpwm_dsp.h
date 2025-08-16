#ifndef __SVPWM_H__
#define __SVPWM_H__
#include "IQmathLib.h" 
typedef struct 
{              
		short SvpwmControlState;
	  short  DcCoeff;
		short  As;  		// Input: phase-a stator variable
		short  Bs;			// Input: phase-b stator variable 
		short  Alpha;		// Output: stationary d-axis stator variable 
		short  Beta;		// Output: stationary q-axis stator variable
		short  Angle;		// Input: rotating angle (pu)  
	  short  Angle2;		// Input: rotating angle (pu)  
		short  CurrentRef;
		short  CurrentFdb;

		short  Sin;
		short  Cos;
		short  IDs;			// Output: rotating d-axis stator variable 
		short  IQs;			// Output: rotating q-axis stator variable 
		short  UDs;			 
		short  UQs;		
    short  UQstmp1;			
	  short  UQstmp2;		
		short  Ualpha; 			// Input: reference alpha-axis phase voltage 
		short  Ubeta;			// Input: reference beta-axis phase voltage 
		short  Ta;				// Output: reference phase-a switching function		
		short  Tb;				// Output: reference phase-b switching function 
		short  Tc;				// Output: reference phase-c switching function

		short  Va;
		short  Vb;
		short  Vc;

		short  PeriodMax;     // Parameter: PWM Half-Period in CPU clock cycles (Q0)
		short  MfuncPeriod;    // Input: Period scaler (Q15) 
		short  MfuncC1;        // Input: EPWM1 A&B Duty cycle ratio (Q15)
		short  MfuncC2;        // Input: EPWM2 A&B Duty cycle ratio (Q15) 
		short  MfuncC3;        // Input: EPWM3 A&B Duty cycle ratio (Q15)
} SVPVM;	  
				 				 
			           


/*-----------------------------------------------------------------------------
Default initalizer for the CLARKE object.
-----------------------------------------------------------------------------*/                     
#define SVPVM_DEFAULTS { _IQ15(0.99), \
		0, \
		0, \
		0, \
	  0, \
		0, \
		0, \
		0, \
		0, \
		0, \
		0, \
		0, \
		0, \
		0, \
		0, \
		0, \
		0, \
		0, \
		0, \
		0, \
		0, \
		0, \
		0, \
		0, \
		0, \
		0, \
		0, \
		0, \
		0}
#endif // __SVPWM_H__
