#include "DSP2803x_Device.h"     // Headerfile Include File
#include "DSP2803x_Examples.h"   // Examples Include File
#include "ExternGlobals.h"
#define TESTMAX  500
int testtable1[TESTMAX];
int testtable2[TESTMAX];
int clearfalg = 0;
int i = 0;
int testcount = 0;
void TestLine(void)
{
    if(clearfalg)
    {
        for(i = 0; i < TESTMAX; i++)
        {
            testtable1[i] = 0;
            testtable2[i] = 0;
        }
        clearfalg = 0;
    }
    if((MotorState == M_MOTOR_RUNING))
    {
        testtable1[testcount] = SpeedFdb;
        testtable2[testcount] = _IQ30toIQ(pidc1.Ui);
        testcount++;
        if(testcount >= (TESTMAX - 1))
        {
            testcount = (TESTMAX - 1);
        }
    }
    else
    {
        testcount = 0;
    }
}
