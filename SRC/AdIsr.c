#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File
#include "DeviceConfig.h"     // DSP2802x Headerfile Include File
#include "ExternGlobals.h"
int tmp0 = 0;	 //E_CURRENT
int tmp1 = 0;	 //B_VOLTAGE
int tmp2 = 0;	 //W_S
int tmp3 = 0;	 //U_S
int tmp4 = 0;	 //FootStoolSpeed
int tmp5 = 0;	 //Oil
interrupt void  adc_isr(void)
{
    tmp0 = AdcResult.ADCRESULT0 >> 2;
    tmp1 = AdcResult.ADCRESULT1 >> 2;
    tmp2 = AdcResult.ADCRESULT2 >> 2;
    tmp3 = AdcResult.ADCRESULT3 >> 2;
    tmp4 = AdcResult.ADCRESULT4 >> 2;
    tmp5 = AdcResult.ADCRESULT5 >> 2;

    YouLiang = tmp5;

    AdEle.input = tmp0;
    AdEle.calc(&AdEle);
    EleCurrent = AdEle.output;

    AdHall.input = tmp4;
    AdHall.calc(&AdHall);
    Hmeas = AdHall.output;

    AdV.input = tmp2;
    AdV.calc(&AdV);
    ImeasB = AdV.output - ImeasBOffset;

    AdU.input = tmp3;
    AdU.calc(&AdU);
    ImeasA = AdU.output - ImeasAOffset;

    GenVolt.input = tmp1;
    GenVolt.calc(&GenVolt);
    AdDcVolt = GenVolt.output;

    PieCtrlRegs.PIEACK.all = PIEACK_GROUP10;   // Acknowledge interrupt to PIE
}
