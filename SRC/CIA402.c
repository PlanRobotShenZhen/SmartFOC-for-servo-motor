
//    No mode change /no mode assigned模式未变更/模式未设定
//    Profile position mode轮廓位置控制模式        pp
//    Velocity mode (速度控制模式)                 vl
//    Profile position mode轮廓位置控制模式)       pv
//    Torque profile mod (轮廓转矩控制模式)        tq

//   Homing mode (原点回归控制模式)                hm
//   Interpolated position mode (插补位置控制模式)      ip
//   Cyclic synchronous position mode (周期位置控制模式)  csp
//   Cyclic synchronous velocity mode (周期速度控制模式)  csv
//   Cyclic synchronous torque mode  (周期转矩控制模式)   cst


//   NOT READY TO SWITCH ON 仅对驱动芯片供电，驱动正在初始化和自检，驱动功能未启用，此状态为内部状态。
//   SWITCH ON DISABLED 驱动初始化完成，驱动参数建立并可以被修改，此状态没有对电机供电，此状态为用户能够操作到的最低状态，驱动上电后，用户接触到的状态。
//   READY TO SWITCH ON 驱动参数可以被修改，驱动功能未启用，等待进入 SWITCH ON状态。
//   SWITCH ON 对驱动提供高电压，功率放大器就绪，驱动参数可以被修改，驱动功能未启用。
//   OPERATION ENABLE 没有检测到故障，驱动功能启用，并对电机上电驱动参数可以被修改，根据 BP[N]参数， 决定此状态刹车是否会自动释放。
//   QUICK STOP ACTIVE 驱动参数可以被修改，急停功能启用，驱动功能启用，电机处于上电状态。
//   FAULT REACTION ACTIVE 驱动参数可以被修改，驱动产生了故障，故障反应功能启用，驱动功能停用，此状态不能手动进入，驱动发生故障时自动进入。





//0 Rcady to switchon 伺服无故障
//1 Switchedon 等待伺服使能
//2 Operation enabled 伺服动作
//3 Fault 故障
//4 Voltage enabled 接通主回路
//5 Quick stop 快速停机
//6 Switch on disabled 伺服准备好
//7 Warning 警告
//8 Manufacturer specific 厂家自定义，空
//9 Remote 远程控制选择
//10 Target reached 目标位置或速度是否到达
//11 Intermal limmit active 软件内部位置限位
//12~13 Operation mode specific 模式相关
//14 Manufacturer specific
//15 Manufacturer specific

extern unsigned int Profile_acceleration;		/* Mapped at index 0x6083, subindex 0x00 */
extern unsigned int Profile_deceleration;		/* Mapped at index 0x6084, subindex 0x00 */
extern int Velocity_sensor_actual_value;		/* Mapped at index 0x6069, subindex 0x00*/
extern int Velocity_demand_value;		        /* Mapped at index 0x606B, subindex 0x00*/
extern int Velocity_actual_value;		        /* Mapped at index 0x606C, subindex 0x00*/
extern short Current_actual_value;		      /* Mapped at index 0x6078, subindex 0x00*/
extern int Position_actual_value;

extern int Pos_actual_value_enc;	/* Mapped at index 0x6063, subindex 0x00 */
extern int Position_actual_value;		/* Mapped at index 0x6064, subindex 0x00*/
extern int Target_velocity;		/* Mapped at index 0x60FF, subindex 0x00*/
extern signed char Modes_of_operation_display;
extern signed char Modes_of_operation;   //0x6060
extern unsigned short  Controlword ;		/* Mapped at index 0x6040, subindex 0x00 */
extern  short Targetl_torque;		//6071
extern  short Torque_actual_value;/* Mapped at index 0x6077, subindex 0x00 */

#include "SpeedPlan2.h"
#include "pidc.h"
#include "IQmathLib.h"
#include "ExternGlobals.h"
#include "Ds402_Slave.h"
#include "CanOpenMode.h"
/***********位置S曲线规划*****************/
SpeedPlant SP;
Set_SP_Para PostionPlanCiA402Mode_1;
int PostionPlanStepCount = 1;
int PostionPlanStepMax = 0;
int PostionPlanStep = 1;
//SP_Error error;


enum SP_Error error;
/****************************************/
short tmp600 = 600;

short shifttest = 1;
short shifttest1 = 1;
void CiA402Mode_Runing(void)
{
//    CanOpenMode.CiA402_State=(CanOpenMode.CiA402_ManufacturerSpecific1<<15) + (CanOpenMode.CiA402_ManufacturerSpecific0<<14)
//	                           + (CanOpenMode.CiA402_OperationModeSpecific1<<13) + (CanOpenMode.CiA402_OperationModeSpecific0<<12)
//	                           + (CanOpenMode.CiA402_IntermalLimmitActive<<11)+(CanOpenMode.CiA402_TargetReached<<10)
//	                           + (CanOpenMode.CiA402_Remote<<9)+ (CanOpenMode.CiA402_ManufacturerSpecific<<8)
//														 + (CanOpenMode.CiA402_Warning<<7)+(CanOpenMode.CiA402_SwitchOnDisabled<<6)
//														 + (CanOpenMode.CiA402_QuickStop<<5)+(CanOpenMode.CiA402_VoltageEnabled<<4)
//														 + (CanOpenMode.CiA402_Fault<<3)+(CanOpenMode.CiA402_OperationEnabled<<2)
//														 + (CanOpenMode.CiA402_SwitchedOn<<1)+(CanOpenMode.CiA402_RcadyToSwitchOn<<0);

//	  CanOpenMode.CiA402_CMD = Controlword;
//	  CanOpenMode.CiA402_RunMode = Modes_of_operation;
//	  CanOpenMode.CiA402_SpeedRef = Target_velocity;
//	  CanOpenMode.CiA402_TargetPosition = Target_position;
//	  CanOpenMode.CiA402_RunMode=3;
//	  Statusword = CanOpenMode.CiA402_State;

    CanOpenMode.CiA402_RunMode = Modes_of_operation;

    if(Controlword == 0)
    {
        CanOpenMode.CanOpen_Mode1State = 0;
    }

    if(Controlword == 128)
    {
        SystemError.SysClearFlag = 1;
        CanOpenMode.CanOpen_Mode1State = 0;  //状态要清零
        Target_velocity = 0;
        Targetl_torque = 0;
    }

//    if(UartMode.CMD == 128) //清除所有错误
//    {
//        UartMode.State1 = 0; //强制清除所有错误
//        UartMode.CMD = 0;
//    }

//    if(SystemError.SysErr)   //出现故障后，
//    {
//        UartMode.State1 = 0; //强制清除所有错误
//        UartMode.CMD = 0;
//    }		
		
	
    if(CanOpenMode.CiA402_RunMode == 1)  //速度      //根据运行模式进行相应操作
    {
//        if(Controlword == 0)
//        {
//            CanOpenMode.CanOpen_Mode1State = 0;
//        }
//        if(Controlword == 0x02)//注意装态位可能无法改
//        {
//            CanOpenMode.CanOpen_Mode1State = 7;
//           // CanOpenMode.CiA402_State = 0x617;
//        }
			
			  SystemError.RuningModeFdb = 11;
        switch(CanOpenMode.CanOpen_Mode1State)
        {
            case 0 :                   //下使能
            {
                CanOpenMode.CiA402_State = 0x260;
                sendOnePDOevent(&Ds402_Slave_Data, 0);

                if(Controlword == 0x06)
                {

                    CanOpenMode.CanOpen_Mode1State = 1;
                }
            }
            break;

            case 1 :                  //上使能
            {
                CanOpenMode.CiA402_State = 0x231;
                sendOnePDOevent(&Ds402_Slave_Data, 0);

                CanOpenMode.CanOpen_Mode1State = 2;
            }
            break;


            case 2 :
            {
                if(Controlword == 0x07)
                {

                    CanOpenMode.CanOpen_Mode1State = 3;
                    CanOpenMode.CiA402_State = 0x233;
                    sendOnePDOevent(&Ds402_Slave_Data, 0);
                }
            }
            break;

            case 3 :
            {
                if(Controlword == 0x0f)
                {

                    CanOpenMode.CanOpen_Mode1State = 4;
                    CanOpenMode.CiA402_State = 0x237;
                    sendOnePDOevent(&Ds402_Slave_Data, 0);
                }
            }
            break;

            case 4 :                  //上使能
            {

                CanOpenMode.CanOpen_Mode1State = 5;
            }
            break;

            case 5 :                  //上使能完成
            {
                CanOpenMode.CanOpen_Mode1State = 6;
                CanOpenMode.CiA402_State = 0x637;
                sendOnePDOevent(&Ds402_Slave_Data, 0);
            }
            break;

            case 6:   //只有速度变，没有状态切换
            {
                //Current_actual_value++;
                //Pos_actual_value_enc++;
                //Position_actual_value++;
                if(Controlword == 0x02)
                {
                    CanOpenMode.CanOpen_Mode1State = 7;
                }
            }
            break;

            case 7:
            {
                CanOpenMode.CiA402_State = 0x617;
                sendOnePDOevent(&Ds402_Slave_Data, 0);
                CanOpenMode.CanOpen_Mode1State = 8;
            }
            break;


            case 8:
            {
                CanOpenMode.CanOpen_Mode1State = 9;
            }
            break;

            case 9:
            {

                CanOpenMode.CiA402_State = 0x240;
                sendOnePDOevent(&Ds402_Slave_Data, 0);
                CanOpenMode.CanOpen_Mode1State = 10;
            }
            break;

            case 10:
            {
                CanOpenMode.CanOpen_Mode1State = 0;

            }
            break;
        }
    }


    if(CanOpenMode.CiA402_RunMode == 3)  //速度
    {
//        if(Controlword == 0)
//        {
//            CanOpenMode.CanOpen_Mode1State = 0;
//        }
        SystemError.RuningModeFdb = 13;
        switch(CanOpenMode.CanOpen_Mode1State)
        {
            case 0:
            {
                pidpv.OutMax = pidpv_OutMax;
                pidpv.OutMin = pidpv_OutMin;
                pidpv.UiMax = pidpv_UiMax;
                pidpv.UiMin = pidpv_UiMin;
                pidpv.Kp = pidpv_Kp;
                pidpv.Ki = pidpv_Ki;

                pidpv.Err = 0;
                pidpv.Ref = 0;
                pidpv.Ui = 0;

                Target_velocity = 0;
                MotorControler.State = 0;

                CanOpenMode.CiA402_State = 0x260;
                sendOnePDOevent(&Ds402_Slave_Data, 0);

                if(Controlword == 0x02)         //注意状态值的切换
                {
                    CanOpenMode.CanOpen_Mode1State = 11;
                    CanOpenMode.CiA402_State = 0x1240;
                    sendOnePDOevent(&Ds402_Slave_Data, 0);
                }
                else if(Controlword == 0x06)
                {
                    CanOpenMode.CanOpen_Mode1State = 1;
                }
                else
                {



                }
            }
            break;

            case 1 :                  //上使能
            {
                CanOpenMode.CiA402_State = 0x231;
                sendOnePDOevent(&Ds402_Slave_Data, 0);

                CanOpenMode.CanOpen_Mode1State = 2;
            }
            break;


            case 2 :
            {
                if(Controlword == 0x07)
                {

                    CanOpenMode.CanOpen_Mode1State = 3;
                    CanOpenMode.CiA402_State = 0x233;
                    sendOnePDOevent(&Ds402_Slave_Data, 0);
                }
            }
            break;

            case 3 :

            {

                if(Controlword == 0x0f)
                {

                    CanOpenMode.CanOpen_Mode1State = 4;
                    CanOpenMode.CiA402_State = 0x237;
                    sendOnePDOevent(&Ds402_Slave_Data, 0);
                }
            }
            break;

            case 4 :                  //上使能

            {

                CanOpenMode.CanOpen_Mode1State = 5;
            }
            break;

            case 5 :                  //上使能完成
            {

                CanOpenMode.CanOpen_Mode1State = 6;
                CanOpenMode.CiA402_State = 0x637;
                sendOnePDOevent(&Ds402_Slave_Data, 0);
            }
            break;


            case 11 :
            {
                if(Controlword == 0x06)
                {
                    CanOpenMode.CanOpen_Mode1State = 12;
                    CanOpenMode.CiA402_State = 0x1231;
                    sendOnePDOevent(&Ds402_Slave_Data, 0);
                }
            }
            break;

            case 12 :
            {
                if(Controlword == 0x07)
                {
                    CanOpenMode.CanOpen_Mode1State = 13;
                    CanOpenMode.CiA402_State = 0x1233;
                    sendOnePDOevent(&Ds402_Slave_Data, 0);
                }
            }
            break;

            case 13 :
            {
                if(Controlword == 0x0f)
                {
                    CanOpenMode.CanOpen_Mode1State = 14;
                    CanOpenMode.CiA402_State = 0x1237;
                    sendOnePDOevent(&Ds402_Slave_Data, 0);
                }
            }
            break;

            case 14 :
            {

                CanOpenMode.CanOpen_Mode1State = 15;

            }
            break;

            case 15 :
            {

                CanOpenMode.CanOpen_Mode1State = 6;
                CanOpenMode.CiA402_State = 0x1637;
                sendOnePDOevent(&Ds402_Slave_Data, 0);

            }
            break;

            case 6:   //只有速度变，没有状态切换
            {
                if(Controlword == 0x02)
                {
                    CanOpenMode.CanOpen_Mode1State = 7;
                }

                MotorControler.State = 6;

                if(Target_velocity > 3000)
                {
                    Target_velocity = 3000;
                }

                if(Target_velocity < -3000)
                {
                    Target_velocity = -3000;
                }

                MotorControler.SpeedRef = Target_velocity;
                MotorControler.TorqueRef  =  tmp600;//Targetl_torque;

                //	Profile_acceleration  加速时间（ms）  0 ～ 2000
                //	Profile_deceleration  减速时间（ms）  0 ～ 2000
                //1==>2000

                //2001-Profile_acceleration;
                //MotorControler.SpeedAcc = 11 - Profile_acceleration / 200;
                //MotorControler.SpeedDcc = 11 - Profile_deceleration / 200;

                MotorControler.SpeedAcc = 1;
                MotorControler.SpeedDcc = 1;

                if(MotorControler.SpeedAcc < 1) MotorControler.SpeedAcc = 1;

                if(MotorControler.SpeedAcc > 10) MotorControler.SpeedAcc = 10;

                if(MotorControler.SpeedDcc < 1) MotorControler.SpeedDcc = 1;

                if(MotorControler.SpeedDcc > 10) MotorControler.SpeedDcc = 10;
            }
            break;

            case 7:
            {
                CanOpenMode.CiA402_State = 0x617;
                sendOnePDOevent(&Ds402_Slave_Data, 0);
                CanOpenMode.CanOpen_Mode1State = 8;
            }
            break;

            case 8:
            {
                CanOpenMode.CanOpen_Mode1State = 9;
            }
            break;

            case 9:
            {
                CanOpenMode.CiA402_State = 0x240;
                sendOnePDOevent(&Ds402_Slave_Data, 0);
                CanOpenMode.CanOpen_Mode1State = 10;
            }
            break;

            case 10:
            {
                CanOpenMode.CanOpen_Mode1State = 0;
            }
            break;
        }
    }

    shifttest1++;

    if(shifttest1 >= 40)
    {
        shifttest = !shifttest;
			  shifttest1=0;
    }

    if(SystemError.SysErr)
    {
        Statusword = CanOpenMode.CiA402_State + 0x8 + (shifttest << 14); //6041  出现故障，增加故障标志位
    }
    else
    {
        Statusword = CanOpenMode.CiA402_State + (shifttest << 14); //6041
    }

    Modes_of_operation_display = Modes_of_operation;//6061
    Position_actual_value = MotorControler.MotorActivePostion;//6064
    Velocity_actual_value	=	MotorControler.SpeedFdbp;///;//0x606c; Current_actual_value
    Torque_actual_value = svpwm.IQs; //6077;
}

