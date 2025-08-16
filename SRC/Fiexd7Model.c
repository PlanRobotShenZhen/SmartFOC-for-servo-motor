#include "DSP2803x_Device.h"     // Headerfile Include File
#include "DSP2803x_Examples.h"   // Examples Include File
#include "ExternGlobals.h"
extern int Sewing_AB(int ReinforceNum, int NeedleA, int NeedleB);
extern int Sewing_CD(int ReinforceNum, int NeedleC, int NeedleD);
void Fixed7Model(void);
void Fixed7Model(void)
{
    switch(SewProcessState)
    {
        case 1 :
        {
            SewingIdle(Fixed7StartReinforce);
        }
        break;


        case 2:
        {
            if(SewingABOk == 1)
            {
                if((LedTieg) && (!StartReinforcePause))
                {
                    MotorState = M_MOTOR_RUNING;
                    FixedTirgStartFalg = 1;
                    SewProcessState = 4;
                    RotorCount = 0;
                    if((Fixed7A < 2) || (Fixed7B < 2))
                    {
                        RotorCountTmp = RotorCount + 1;
                    }
                    else
                    {
                        RotorCountTmp = RotorCount + 2;
                    }

                    AdMotorIdle = 0;
                    StartReinforceSnub = Fixed7B;
                    if(StartReinforceSnub > 2)
                    {
                        StartReinforceSnub = 2;
                    }
                }
                else if((AdSpdRef == 0) && (MotorState == M_MOTOR_IDLE))
                {
                    AdMotorIdle = 1;
                }
                else if((AdSpdRef >= SpeedRefMin) && (AdMotorIdle))
                {
                    SewProcessState = 4;
                    RotorCount = 0;
                    RotorCountTmp = RotorCount - 1;
                    AdMotorIdle = 0;
                }
                else if((!StartReinforcePause) && (AdSpdRef >= SpeedRefMin) && ((RotorCount - RotorCountTmp) >= StartReinforcePauseOffset))
                {
                    MotorState = M_MOTOR_RUNING;
                    FixedTirgStartFalg = 1;
                    SewProcessState = 4;
                    RotorCount = 0;
                    if((Fixed7A < 2) || (Fixed7B < 2))
                    {
                        RotorCountTmp = RotorCount + 1;
                    }
                    else
                    {
                        RotorCountTmp = RotorCount;
                    }
                    AdMotorIdle = 0;
                    if(AdSpdRef == 0)
                    {
                        StartReinforceSnub = 0;
                    }
                    else
                    {
                        StartReinforceSnub = Fixed7B;
                        if(StartReinforceSnub > 2)
                        {
                            StartReinforceSnub = 2;
                        }
                    }
                }
                else
                {
                    if((RotorCount - RotorCountTmp) >= StartReinforcePauseOffset)
                    {
                        if(MotorState == M_MOTOR_RUNING)
                        {
                            MotorStopFlag = 1;
                            MotorStopHorL = (LedHaltAngle);
                        }
                    }
                }
            }
            else if(Sewing_AB(Fixed7StartReinforce, Fixed7A, Fixed7B) == 0)
            {
                SewingABOk = 1; //返回后不要立及停车，而是再等一两针再等车。
                RotorCount = 0;
                RotorCountTmp = RotorCount;
                if(Fixed7A > 1)
                {
                    StartReinforcePauseOffset = 2;
                }
                else
                {
                    StartReinforcePauseOffset = 0;
                }
            }
        }
        break;

        case 4 :
        {
            if(Fixed1(Fixed7E, LedTieg, 0, 0) == 0)
            {
                SewingWait(5);
            }
        }
        break;

        case 5 :
        {
            if(Fixed1(Fixed7F, LedTieg, 0, 0) == 0)
            {
                SewingWait(6);
            }
        }
        break;

        case 6 :
        {
            if(Fixed1(Fixed7G, LedTieg, 0, 0) == 0)
            {
                SewingWait(7);
            }
        }
        break;

        case 7 :
        {
            if(Fixed1(Fixed7H, LedTieg, 0, 0) == 0)
            {
                SewingWait(8);
            }
        }
        break;

        case 8 :
        {
            if(Fixed1(Fixed7G, LedTieg, 0, 0) == 0)
            {
                SewingWait(9);
            }
        }
        break;

        case 9 :
        {
            if(Fixed1(Fixed7F, LedTieg, 0, 0) == 0)
            {
                SewingWait(10);
            }
        }
        break;

        case 10 :
        {
            if(EndReinforcePause) //暂停
            {
                if(Fixed1((Fixed7E), LedTieg, 5, 0) == 0) //暂停  //区分有没有后固缝纫
                {
                    SewProcessState = 102;
                }
            }
            else if(Fixed7EndReinforce) //不暂停，并且有后固缝纫
            {
                if(Fixed((Fixed7E), LedTieg, Fixed7C, Fixed7D) == 0)
                {
                    //SewProcessState=11;
                    if((AdSpdRef >= SpeedRefMin))
                    {
                        SewProcessState = 11;
                    }
                    else if(LedTieg)
                    {
                        SewProcessState = 11;
                    }
                }
            }
            else   //不暂停，并且无后固缝纫
            {
                if(Fixed1((Fixed7E), LedTieg, 5, 1) == 0)
                {
                    SewProcessState = 11;
                }
            }
        }
        break;

        case 102 :
        {
            BeforeEndReinforce(Fixed7EndReinforce, 11);
        }
        break;

        case 11 :
        {
            if(Fixed7EndReinforce == 0)
            {
                MotorStopFlag = 1;
                MotorStopHorL = 0;
                SewProcessState = 12;
                SpeedRef = 0;
                DaoFengFlag = 0;
                if(LedCut)
                {
                    if((MotorState == M_MOTOR_RUNING) || (MotorState == M_MOTOR_ARREST))
                    {
                        StopingCutFalg = 2;
                    }
                    else
                    {
                        MotorState = M_MOTOR_PROCUTING;
                    }
                }
            }
            else if(Sewing_CD(Fixed7EndReinforce, Fixed7C, Fixed7D) == 0)
            {
                SewProcessState = 12;
                if(LedCut)
                {
                    DaoFengFlag = 0;
                    StopingCutFalg = 1;
                    SpeedRef = SpdCuting;
                }
                else if(MotorState == M_MOTOR_RUNING)
                {
                    SpeedRef = 0;
                    MotorStopFlag = 1;
                    MotorStopHorL = (LedHaltAngle);
                }
            }
            AdCutFlag = 50;
        }
        break;

        case 12:
        {
            if(MotorState == M_MOTOR_IDLE)
            {
                SewProcessState = 13;
            }
        }
        break;

        case 500 :
        {
            ExigenceCutDone(12);
        }
        break;

        case 13 :
        {
            AdSpdDelay();
        }
        break;
        default :
        {

        }
        break;
    }
    ExigenceCut(12);
}

