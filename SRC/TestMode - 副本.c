
#include "ExternGlobals.h"
#include "SpeedPlan.h"
#include "pidc.h"
#include "IQmathLib.h"
#include "ExternGlobals.h"
#include "FLASH_WR.h"
#include "UartMode.h"

int TestSpeedTable[20] = {10, 300, 50, -10, 0, -3000, 50, 500, 800, -1500, 3000, -3000, 500, 0};
int TestSpeedCount = 0;
int TestSpeedStep = 0;
void TestMode_Runing(void)
{


    if(UartMode.Mode == 3) //速度控制
    {
        switch(UartMode.State1)
        {
            case 0 :
            {
                pidpv.OutMax = pidpv_OutMax;
                pidpv.OutMin = pidpv_OutMin;
                pidpv.UiMax = pidpv_UiMax;
                pidpv.UiMin = pidpv_UiMin;
                pidpv.Kp = pidpv_Kp;
                pidpv.Ki = pidpv_Ki;

                pidpv.Err = 0;
                MotorControler.SpeedRef = 0;

                MotorControler.State = 0;
                pidpv.Ref = 0;
                pidpv.Ui = 0;


                if(UartMode.CMD == 15)
                {
                    UartMode.State1 = 1;
                    UartMode.CMD = 0;
                }
            }
            break;

            case 1 :
            {
                if(UartMode.CMD == 31)
                {
                    UartMode.State1 = 2;
                    UartMode.CMD = 0;
                }

                MotorControler.State = 6;
                MotorControler.SpeedRef = 0;
            }
            break;

            case 2 :  //电机正在运行过程中
            {
                if(UartMode.CMD == 128)
                {
                    UartMode.State1 = 0;
                    UartMode.CMD = 0;
                }

                TestSpeedCount++;

                if(TestSpeedCount >= 50)
                {
                    TestSpeedCount = 0;
                    TestSpeedStep++;

                    if(TestSpeedStep >= 10)
                    {
                        TestSpeedStep = 0;
                    }
                }

                MotorControler.SpeedRef = TestSpeedTable[TestSpeedStep];
                MotorControler.TorqueRef  = 600;
                MotorControler.SpeedAcc = 3000;
                MotorControler.SpeedDcc = 3000;
            }
            break;
        }
    }

}




short	a_IQ15abs(short a)
{
    if(a >= 0)
    {
        return a;
    }
    else
    {
        return 0 - a;
    }
}


short a_IQ15mpy(short a, short b)
{
    int c = 0;
    short d = 0;
    c = ((int)a)  * ((int)b);
    d = c >> 15;
    return d;
}



int a_IQ30mpy(int a, int  b)
{
    long int c = 0;
    int  d = 0;
    c = ((long int)a)  * ((long int)b);
    d = c >> 30;
    return d;
}



int a_IQdiv2(int a)
{
	   
	return a>>1;
	  
}


#define TYPE_DEFAULT    (0)

int_fast32_t a__IQNdiv(int_fast32_t iqNInput1, int_fast32_t iqNInput2, const uint8_t type, const int8_t q_value);

int32_t a_IQ15div(int32_t a, int32_t b)
{
    return a__IQNdiv(a, b, TYPE_DEFAULT, 15);
}


  uint_fast32_t a__mpyf_ul(uint_fast32_t arg1, uint_fast32_t arg2)
{
    return (uint_fast32_t)(((uint_fast64_t)arg1 * (uint_fast64_t)arg2) >> 31);

}

int_fast32_t a__mpyf_ul_reuse_arg1(uint_fast32_t arg1, uint_fast32_t arg2)
{
    /* This is identical to __mpyf_ul */
    return (uint_fast32_t)(((uint_fast64_t)arg1 * (uint_fast64_t)arg2) >> 31);
}

 int_fast32_t a__IQNdiv(int_fast32_t iqNInput1, int_fast32_t iqNInput2, const uint8_t type, const int8_t q_value)
{
    uint8_t ui8Index, ui8Sign = 0;
    uint_fast32_t ui32Temp;
    uint_fast32_t uiq30Guess;
    uint_fast32_t uiqNInput1;
    uint_fast32_t uiqNInput2;
    uint_fast32_t uiqNResult;
    uint_fast64_t uiiqNInput1;
    uint_fast16_t ui16IntState;
    uint_fast16_t ui16MPYState;
	
	
	const uint8_t _IQ6div_lookup[65] = {
    0x7F, 0x7D, 0x7B, 0x79, 0x78, 0x76, 0x74, 0x73, 
    0x71, 0x6F, 0x6E, 0x6D, 0x6B, 0x6A, 0x68, 0x67, 
    0x66, 0x65, 0x63, 0x62, 0x61, 0x60, 0x5F, 0x5E, 
    0x5D, 0x5C, 0x5B, 0x5A, 0x59, 0x58, 0x57, 0x56, 
    0x55, 0x54, 0x53, 0x52, 0x52, 0x51, 0x50, 0x4F, 
    0x4E, 0x4E, 0x4D, 0x4C, 0x4C, 0x4B, 0x4A, 0x49, 
    0x49, 0x48, 0x48, 0x47, 0x46, 0x46, 0x45, 0x45, 
    0x44, 0x43, 0x43, 0x42, 0x42, 0x41, 0x41, 0x40, 0x40
};
	
	
	
	
	
	
	
	

    if (type == TYPE_DEFAULT) {
        /* save sign of denominator */
        if (iqNInput2 <= 0) {
            /* check for divide by zero */
            if (iqNInput2 == 0) {
                return INT32_MAX;
            }
            else {
                ui8Sign = 1;
                iqNInput2 = -iqNInput2;
            }
        }

        /* save sign of numerator */
        if (iqNInput1 < 0) {
            ui8Sign ^= 1;
            iqNInput1 = -iqNInput1;
        }
    }
    else {
        /* Check for divide by zero */
        if (iqNInput2 == 0) {
            return INT32_MAX;
        }
    }

    /* Save input1 and input2 to unsigned IQN and IIQN (64-bit). */
    uiiqNInput1 = (uint_fast64_t)iqNInput1;
    uiqNInput2 = (uint_fast32_t)iqNInput2;

    /* Scale inputs so that 0.5 <= uiqNInput2 < 1.0. */
    while (uiqNInput2 < 0x40000000) {
        uiqNInput2 <<= 1;
        uiiqNInput1 <<= 1;
    }

    /*
     * Shift input1 back from iq31 to iqN but scale by 2 since we multiply
     * by result in iq30 format.
     */
    if (q_value < 31) {
        uiiqNInput1 >>= (31 - q_value - 1);
    }
    else {
        uiiqNInput1 <<= 1;
    }

    /* Check for saturation. */
    if (uiiqNInput1 >> 32) {
        if (ui8Sign) {
            return INT32_MIN;
        }
        else {
            return INT32_MAX;
        }
    }
    else {
        uiqNInput1 = (uint_fast32_t)uiiqNInput1;
    }

    /* use left most 7 bits as ui8Index into lookup table (range: 32-64) */
    ui8Index = uiqNInput2 >> 24;
    ui8Index -= 64;
    uiq30Guess = (uint_fast32_t)_IQ6div_lookup[ui8Index] << 24;

    /*
     * Mark the start of any multiplies. This will disable interrupts and set
     * the multiplier to fractional mode. This is designed to reduce overhead
     * of constantly switching states when using repeated multiplies (MSP430
     * only).
     */
    //__mpyf_start(&ui16IntState, &ui16MPYState);
		/* Do nothing. */

    /* 1st iteration */
    ui32Temp = a__mpyf_ul(uiq30Guess, uiqNInput2);
    ui32Temp = -((uint_fast32_t)ui32Temp - 0x80000000);
    ui32Temp = ui32Temp << 1;
    uiq30Guess = a__mpyf_ul_reuse_arg1(uiq30Guess, ui32Temp);

    /* 2nd iteration */
    ui32Temp = a__mpyf_ul(uiq30Guess, uiqNInput2);
    ui32Temp = -((uint_fast32_t)ui32Temp - 0x80000000);
    ui32Temp = ui32Temp << 1;
    uiq30Guess = a__mpyf_ul_reuse_arg1(uiq30Guess, ui32Temp);

    /* 3rd iteration */
    ui32Temp = a__mpyf_ul(uiq30Guess, uiqNInput2);
    ui32Temp = -((uint_fast32_t)ui32Temp - 0x80000000);
    ui32Temp = ui32Temp << 1;
    uiq30Guess = a__mpyf_ul_reuse_arg1(uiq30Guess, ui32Temp);

    /* Multiply 1/uiqNInput2 and uiqNInput1. */
    uiqNResult = a__mpyf_ul(uiq30Guess, uiqNInput1);

    /*
     * Mark the end of all multiplies. This restores MPY and interrupt states
     * (MSP430 only).
     */
    //__mpy_stop(&ui16IntState, &ui16MPYState);
		/* Do nothing. */

    /* Saturate, add the sign and return. */
    if (type == TYPE_DEFAULT) {
        if (uiqNResult > INT32_MAX) {
            if (ui8Sign) {
                return INT32_MIN;
            }
            else {
                return INT32_MAX;
            }
        }
        else {
            if (ui8Sign) {
                return -(int_fast32_t)uiqNResult;
            }
            else {
                return (int_fast32_t)uiqNResult;
            }
        }
    }
    else {
        return uiqNResult;
    }
}
