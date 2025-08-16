#ifndef __PIDpos_H__
#define __PIDpos_H__

typedef struct
{
    int  Ref;   		// Input: Reference input
    int  Fdb;   		// Input: Feedback input
	  int  Speedref;   		// Input: Reference input
	  int  Speedfdb;   		// Input: Reference input
    int  Err;			// Variable: Error
    int  Kp;			// Parameter: Proportional gain
    int  Ki;			// Parameter: Integral gain
    int  Up;			// Variable: Proportional output
    int  Ui;			// Variable: Integral output
    int  OutPreSat;	// Variable: Pre-saturated output
    short OutMax;		// Parameter: Maximum output
    short OutMin;		// Parameter: Minimum output
    int  UiMax;		// Parameter: Integral Maximum output
    int  UiMin;		// Parameter: Integral Minimum output
    short   Out;   		// Output: PID output
	  short Iqs;
	  long long SumIqs;
    void (*calc)();	  	// Pointer to calculation function
} PIDpos;

typedef PIDpos* PIDpos_handle;

/*-----------------------------------------------------------------------------
Default initalizer for the PIDREG3 object.
-----------------------------------------------------------------------------*/
#define PIDpos_DEFAULTS { 0, \
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
		(void (*)(unsigned int))pidposition_calc }

/*------------------------------------------------------------------------------
Prototypes for the functions in PIDREG3.C
------------------------------------------------------------------------------*/
void pidposition_calc(PIDpos_handle);
#endif



