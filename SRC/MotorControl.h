#ifndef __MOTORCONTROL_H__
#define __MOTORCONTROL_H__

typedef struct
{
    short State;
    short SpeedRef;
    short SpeedFdb;
    short SpeedAcc;
    short SpeedDcc;
    short TorqueRef;
    short TorqueFdb;
    int PositionRef;
    int PositionFdb;
    short Error;
    int MotorActivePostion;
    int RotorCount;
    short MaxTorque;
    short RatedTorque;
    short MaxSpeed;

    short SpeedFdbpFilter1;
    short SpeedFdbpFilter2;
    short SpeedFdbpFilter3;

    short SpeedFdbp;
    short MotorPoles;
		
		
    unsigned int spiread_anlge;
    short AngleFromMT6835;
    short AngleFromMT6835Offset;
    short AngleFromMT6835Offset1;		
		
		
		

} MOTORCONTROL;

typedef MOTORCONTROL* MOTORCONTROL_handle;

/*-----------------------------------------------------------------------------
Default initalizer for the PIDREG3 object.
-----------------------------------------------------------------------------*/
#define PID_MOTORCONTROL_DEFAULTS { 0, \
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
        (void (*)(unsigned int))pid_calc }

/*------------------------------------------------------------------------------
Prototypes for the functions in PIDREG3.C
------------------------------------------------------------------------------*/
//void pid_calc(PID_handle);
				
#endif

