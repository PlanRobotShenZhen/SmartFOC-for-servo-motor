#include "main.h"
void main(void)
{
//  Step 1. Initialize System Control:
    InitSysCtrl();
//  Step 2. Initalize GPIO:
    InitEPwm1Gpio();
    InitEPwm2Gpio();
    InitEPwm3Gpio();
    InitEPwm5Gpio();
    InitEPwm6Gpio();
    InitSpiaGpio();
    InitSciaGpio();
    InitI2CGpio();
    InitTzGpio();
    UserIoInit();

    RLY_OFF;
    asm(" RPT #7 || NOP");
    RG_OFF;
    asm(" RPT #7 || NOP");
    RG_OFF;
    asm(" RPT #7 || NOP");
    ELOCK_ON;
    asm(" RPT #7 || NOP");
    JIANXIAN_0FF;
    asm(" RPT #7 || NOP");
    JIAXIANQI_OFF;
    asm(" RPT #7 || NOP");
    SONGXINA_OFF;
    asm(" RPT #7 || NOP");
    TAIYAJIAO_OFF;
    asm(" RPT #7 || NOP");
    DAOFENG_OFF;
    //Step 3. Clear all interrupts and initialize PIE vector table:
    //Disable CPU interrupts
    DINT;
    IER = 0x0000;
    IFR = 0x0000;
    InitPieCtrl();
    InitPieVectTable();
    //Step 4. Initialize all the Device Peripherals:
    scia_fifo_init();
    InitAdc();
    ADInit();
    SpiInit();
    I2CA_Init();
    InitCpuTimers();
    ConfigCpuTimer(&CpuTimer0, 60, 1000);
    StartCpuTimer0();
    F280X_PWM_InitA();
    F280X_PWM_InitB();
    MemCopy(&RamfuncsLoadStart, &RamfuncsLoadEnd, &RamfuncsRunStart);
    InitFlash();
//  Step 5. User specific code, enable interrupts:

    EALLOW;  // This is needed to write to EALLOW protected registers
    PieVectTable.TINT0 = &CPU_timer0_isr;
    PieVectTable.EPWM1_TZINT = &Tzint_isr;
    PieVectTable.EPWM1_INT = &MainISRA;
    PieVectTable.ADCINT3 = &adc_isr;
    PieVectTable.SCIRXINTA = &SciaRxIsr;
    PieVectTable.SCITXINTA = &SciaTxIsr;
    PieVectTable.SPIRXINTA = &SPI_isr;

    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;           //TIMER0
    PieCtrlRegs.PIEIER2.bit.INTx1 = 1;           //EPWM1TZ
    PieCtrlRegs.PIEIER6.bit.INTx1 = 1;           //spi
    PieCtrlRegs.PIEIER9.bit.INTx1 = 1;           //SCI
    PieCtrlRegs.PIEIER9.bit.INTx2 = 1;           //SCI
    PieCtrlRegs.PIEIER10.bit.INTx3 = 1;          //ad
    IER = (M_INT1 | M_INT2 | M_INT3 | M_INT6 | M_INT9 | M_INT10);
    EDIS;
    EINT;
    ERTM;

    SysInit();

    while(1)
    {
        if(PowerDirection_ON)
        {
            LossACTime = 0;
        }

        if(JaiXianQiFlag)
        {
            if((RotorMacAngle >> 6) == (JiaXainStartMinAngle >> 6))
            {
                JiaXianTimerStart = 1;
                JiaXianTimer = 0;
                JaiXianQiFlag = 0;
                JIAXIANQI_ON;
            }
        }

        if(JiaXianTimerStart)
        {
            if((RotorMacAngle >> 6) == (JiaXainEndMinAngle >> 6))
            {
                JIAXIANQI_OFF;
                JiaXianTimerStart = 0;
            }
            else
            {
                JIAXIANQI_ON;
            }
        }
        else
        {
            JIAXIANQI_OFF;
        }

        if(RxFinish)
        {
            AnalyseRxCommand();
            RxFinish = 0;
            AnalyseRxCommandFinish = 1;
        }
        Rotor();
        /**************************************1ms***************************************/
        if(timer.Flag1ms)
        {
            timer.Flag1ms = M_DISABLE;
            AdDeal();
            DixianManage();
            LostPhase();
        }//end timer.Flag1ms
        /**************************************10ms**************************************/
        if(timer.Flag10ms)
        {
            timer.Flag10ms = M_DISABLE;
            PowerManage();
            EleCurrentManage();
            LostCoder();
        }//end timer.Flag10ms
        /**************************************100ms**************************************/
        if(timer.Flag100ms)
        {
            timer.Flag100ms = M_DISABLE;
            TorsionAnalyse();
            SaveVar();
            if((SewProcessState > 1) && (ProcessRuning == 0))
            {
                ProcessRuning = 1;
                ProcessRuningAsk = 1;
            }
            else if((SewProcessState == 1) && (ProcessRuning == 1))
            {
                ProcessRuning = 0;
                ProcessRuningAsk = 1;
            }
            AddNeedle();
            LedLiangDuDeLay--;
            if(LedLiangDuDeLay < 0)
            {
                LedLiangDuDeLay = 0;
            }
        }//end timer.Flag100ms


        /**************************************500ms**************************************/
        if(timer.Flag500ms)
        {
            timer.Flag500ms = M_DISABLE;
            SysErrManage();
        }//end timer.Flag500ms

        /**************************************1s****************************************/

        if(timer.Flag1s)
        {
            timer.Flag1s = M_DISABLE;
            if(ReadEeprom0flag == 0)
            {
                ReadEeprom0flag = 1;
                ReadEeprom(0);
            }
            TaiYaJiaoProtectCount++;
            if(SciaRegs.SCIRXST.bit.RXERROR)
            {
                SciaRegs.SCICTL1.bit.SWRESET = 0;
            }
            SciaRegs.SCICTL1.bit.SWRESET = 1;

            if(SpeedFdb > 15)
            {
                MotorRunTime1s++;
                if(MotorRunTime1s > 60)
                {
                    MotorRunTime1m++;
                    MotorRunTime1s = 0;
                }
                if(MotorRunTime1m > 60)
                {
                    MotorRunTime1h++;
                    MotorRunTime1m = 0;
                    if(MotorRunTime1h > 9999)
                    {
                        MotorRunTime1h = 0;
                    }
                }
            }
        }//end timer.Flag1s
        if(timer.Flag1m)
        {
            timer.Flag1m = M_DISABLE;
            ProcessVar[80] = JianXianJiShu;
            ProcessVar[81] = DangQianDixiZhongShu;

            ProcessVar[85] = MotorRunTime1h;
            ProcessVar[86] = SystemPowerOnTime1h;

            for(SysErrSaveTmp = 0; SysErrSaveTmp < 16; SysErrSaveTmp++)
            {
                ProcessVar[90 + SysErrSaveTmp] = SysErrSave[SysErrSaveTmp];
            }
            for(SysErrSaveTmp = 0; SysErrSaveTmp < 15; SysErrSaveTmp++)
            {
                ProcessVar[106 + SysErrSaveTmp] = SysErrCountSave[SysErrSaveTmp];
            }
        } //end timer.Flag1m

        if(timer.Flag1h)
        {
            timer.Flag1h = M_DISABLE;
            SystemPowerOnTime1h++;
            if(SystemPowerOnTime1h > 9999)
            {
                SystemPowerOnTime1h = 0;
            }
        }

    }//end while(1)

}//end main()
