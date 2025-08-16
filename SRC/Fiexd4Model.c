#include "DSP2803x_Device.h"     // Headerfile Include File
#include "DSP2803x_Examples.h"   // Examples Include File
#include "ExternGlobals.h"
extern int Sewing_AB(int ReinforceNum, int NeedleA, int NeedleB);
extern int Sewing_CD(int ReinforceNum, int NeedleC, int NeedleD);
void Fixed4Model(void);
void Fixed4Model(void)
{
    switch(SewProcessState)
    {
        case 1 :
        {
            SewingIdle(Fixed4StartReinforce);
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
                    if((Fixed4A < 2) || (Fixed4B < 2))
                    {
                        RotorCountTmp = RotorCount + 1;
                    }
                    else
                    {
                        RotorCountTmp = RotorCount + 2;
                    }

                    AdMotorIdle = 0;
                    StartReinforceSnub = Fixed4B;
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
                    if((Fixed4A < 2) || (Fixed4B < 2))
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
                        StartReinforceSnub = Fixed4B;
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
            else if(Sewing_AB(Fixed4StartReinforce, Fixed4A, Fixed4B) == 0)
            {
                SewingABOk = 1; //返回后不要立及停车，而是再等一两针再等车。
                RotorCount = 0;
                RotorCountTmp = RotorCount;
                if(Fixed4A > 1)
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
            if(Fixed1(Fixed4E, LedTieg, 0, 0) == 0)
            {
                SewingWait(5);
            }
        }
        break;

        case 5 :
        {
            if(Fixed1(Fixed4F, LedTieg, 0, 0) == 0)
            {
                SewingWait(6);
            }
        }
        break;


        case 6 :
        {
            if(Fixed1(Fixed4G, LedTieg, 0, 0) == 0)
            {
                SewingWait(7);
            }
        }
        break;


        case 7 :
        {
            if(EndReinforcePause) //暂停
            {
                if(Fixed1((Fixed4H), LedTieg, 5, 0) == 0) //暂停  //区分有没有后固缝纫
                {
                    SewProcessState = 102;
                }
            }

            else if(Fixed4EndReinforce) //不暂停，并且有后固缝纫
            {
                if(Fixed((Fixed4H), LedTieg, Fixed4C, Fixed4D) == 0)
                {
                    //SewProcessState=8;
                    if((AdSpdRef >= SpeedRefMin))
                    {
                        SewProcessState = 8;
                    }
                    else if(LedTieg)
                    {
                        SewProcessState = 8;
                    }
                }
            }
            else   //不暂停，并且无后固缝纫
            {
                if(Fixed1((Fixed4H), LedTieg, 5, 1) == 0)
                {
                    SewProcessState = 8;
                }
            }
        }
        break;

        case 102 :
        {
            BeforeEndReinforce(Fixed4EndReinforce, 8);
        }
        break;

        case 8 :
        {
            if(Fixed4EndReinforce == 0)
            {
                MotorStopFlag = 1;
                MotorStopHorL = 0;
                SewProcessState = 9;
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
            else if(Sewing_CD(Fixed4EndReinforce, Fixed4C, Fixed4D) == 0)
            {
                SewProcessState = 9;
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
                }
            }
            AdCutFlag = 50;
        }
        break;

        case 9:
        {
            if(MotorState == M_MOTOR_IDLE)
            {
                SewProcessState = 10;
            }
        }
        break;
        case 500 :
        {
            ExigenceCutDone(9);
        }
        break;

        case 10 :
        {
            AdSpdDelay();
        }
        break;

        default :
        {
        }
        break;
    }

    ExigenceCut(9);

}

