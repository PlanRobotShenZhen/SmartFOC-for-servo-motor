#include "DSP2803x_Device.h"     // Headerfile Include File
#include "ExternGlobals.h"

int SpeedTestRef = 0;
int TestTaiYaJiaoTimer = 0;
int TestTaiYaJiaoTimerMax = 1000;
int  TestClk = 0;
int  TestModelRun = 0;
int  TestModelState = 0;
int TestSewProcessState = 1;
void TestRun(void);
void TestModel(void)
{


    if(AdSpdRef >= SpeedRefMin)
    {
        TestModelRun = 1;
    }
    else if((AdSpdRef == 2) || SysErr) //强制解除跑合测试
    {
        TestModelRun = 0;
        TestModelStopingFlag = 1;
    }

    if(TestModelStopingFlag == 1)
    {
        if(MotorState == M_MOTOR_IDLE)
        {
            TestModelFlag = 0;
            TestModelStopingFlag = 0;
            TestSewProcessState = 1;
        }
    }
    SewProcessState = TestSewProcessState;

    if(TestModelRun == 1)
    {
        TestClk++;
        switch(TestModelState)
        {
            case 1:
            {
                SpeedTestRef = TestSpeed;
                if(TestSewProcessState == 4)
                {
                    TestModelState = 2;
                    TestClk = 0;
                }
            }
            break;

            case 2 :
            {
                SpeedTestRef = TestSpeed;
                if(TestClk >= TestRunTime)
                {
                    TestModelState = 3;
                    SpeedTestRef = 3;
                }
            }
            break;

            case 3 :
            {
                SpeedTestRef = 3;
                if(TestSewProcessState == 1)
                {
                    TestModelState = 4;
                    TestClk = 0;
                }
            }
            break;

            case 4 :
            {
                SpeedTestRef = 0;
                if(TestClk >= TestStopTime)
                {
                    TestModelState = 1;
                }
            }
            break;
        }
    }
    else if(TestModelRun == 0)
    {
        TestClk = 0;
        SpeedTestRef = 2;
        TestModelState = 1;
    }
    TestRun();
}



void TestRun(void)
{
    switch(TestSewProcessState)
    {
        case 1 :
        {
            if(SpeedTestRef > SpeedRefMin)
            {

                if(JiaXianQiEleOn)
                {
                    JaiXianQiFlag = 1;
                }


                if(TestStartReinforce != 0)
                {
                    SpeedRef = StartReinforceSpeed;
                    TestSewProcessState = 2;
                    MotorState = M_MOTOR_RUNING;
                    RotorCountTmp = RotorCount;
                }
                else
                {
                    TestSewProcessState = 4;
                    RotorCountTmp = RotorCount;
                }
            }
        }
        break;

        case 2:
        {
            SpeedRef = StartReinforceSpeed;
            if(Sewing_AB(TestStartReinforce, TestA, TestB) == 0)
            {
                TestSewProcessState = 4;
                RotorCountTmp = RotorCount;
            }
        }
        break;

        case 4 :
        {
            TorsionConTrol = 2;
            MotorState = M_MOTOR_RUNING;
            SpeedRef = (((long)SpeedTestRef) << 14) / PositiveDcCoeff;
            if((SpeedTestRef <= 3) && (SpeedTestRef > 0))
            {
                if(FreeEndReinforce == 0)
                {
                    TestSewProcessState = 5;
                }
                else
                {
                    TestSewProcessState = 55;
                    RotorCount = 0;
                    RotorCountTmp = RotorCount;
                }
            }
        }
        break;

        case 55 :
        {
            if(TestSpeed > EndReinforceSpeed)
            {
                SpeedRef = EndReinforceSpeed;
            }
            else
            {
                SpeedRef = TestSpeed;
            }

            if((RotorCount - RotorCountTmp) > (TestSpeed >> 5))
            {
                TestSewProcessState = 5;
                RotorCount = 0;
                RotorCountTmp = RotorCount;
            }

        }
        break;

        case 5 :
        {
            if(FreeEndReinforce == 0)
            {
                TestSewProcessState = 7;
                MotorStopFlag = 1;
                if(LedCut)
                {
                    DaoFengFlag = 0;
                    StopingCutFalg = 2; ////2013.04.10
                    SpeedRef = SpdCuting;
                }
            }
            else if(Sewing_CD(TestEndReinforce, TestC, TestD) == 0)
            {
                TestSewProcessState = 7;
                MotorStopFlag = 1;
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
            else
            {
                SpeedRef = EndReinforceSpeed;
                MotorState = M_MOTOR_RUNING;
            }
        }
        break;

        case 7:
        {
            if(MotorState == M_MOTOR_IDLE)
            {
                TestSewProcessState = 8;
                StopingCutFalg = 0;
                DaoFengFlag = 0;
                AdCutFlag = 0;
                TestTaiYaJiaoTimer = 0;
            }
        }
        break;
        case 8 :
        {
            TestTaiYaJiaoTimer++;
            TaiYaJiaoStart = 1;
            if(SpeedTestRef > 3)
            {
                TestSewProcessState = 1;
                TaiYaJiaoStart = 0;
            }
            else if(TestTaiYaJiaoTimer > TestTaiYaJiaoTimerMax)
            {
                TestSewProcessState = 1;
                TaiYaJiaoStart = 0;
            }
        }
        break;
    }
    if((SpeedTestRef == 2) && (MotorState == M_MOTOR_RUNING) && (AdCutFlag == 0))
    {
        MotorStopFlag = 1;
        TestSewProcessState = 7;
        AdCutFlag = 1;
        DaoFengFlag = 0;
    }
}






























