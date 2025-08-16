#include "DSP2803x_Device.h"     // Headerfile Include File
#include "DSP2803x_Examples.h"   // Examples Include File
#include "ExternGlobals.h"
interrupt void Tzint_isr(void)
{
    SysErr = M_SYSERR_IPM;
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP2;
}

