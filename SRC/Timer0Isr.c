#include "DSP2803x_Device.h"     // Headerfile Include File
#include "ExternGlobals.h"
extern void EleAct2(void);
extern int SouDongDaoFengNeedleAddFalg;

int Timer20msCount = 0;

interrupt void CPU_timer0_isr(void)
{
    SysClk++;
    if(!SysErr)
    {
        if(NeedleAddFalg || SouDongDaoFengNeedleAddFalg)
        {
        }
        else if(SewModelStart == 0)
        {
        }
        else if(TestModelFlag)  TestModel();
        else if(SewModel == 1)    FreeModel();
        else if(SewModel == 2)    Fixed1Model();
        else if(SewModel == 3)    Fixed4Model();
        else if(SewModel == 4)    Fixed7Model();
        else if(SewModel == 5)    Fixed8Model();
        else if(SewModel == 6)    FoldWModel();

    }
    else
    {
        DaoFengFlag = 0;
        MotorState = M_MOTOR_IDLE;
        svpwm.UQs = 0;
        CutFlag = 0;
        SaoxianFlag = 0;
        ELERung = 0;
        DaoFengELEState = 0;
        SewProcessState = 1;
    }

    pida1.Ref = svpwm.SpeedRef;   //
    pida1.Fdb = SpeedFdb;
    pida1.calc(&pida1);

    pidc1.Fdb = SpeedFdb;
    pidc1.calc(&pidc1);

    pidc2.Fdb = SpeedFdb;
    pidc2.calc(&pidc2);


    SpeedCalculate();

    TorsionRectify();

    EEProm(&Eeprom1);

    Timeing(&timer);

    EleAct2();

    TestLine();

    Timer20msCount++;
    if(Timer20msCount >= 20)
    {
        Timer20msCount = 0;
        AnalyseTxCommand();
    }

    PieCtrlRegs.PIEACK.all |= PIEACK_GROUP1;
}
