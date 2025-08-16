
//    No mode change /no mode assignedģʽδ���/ģʽδ�趨
//    Profile position mode����λ�ÿ���ģʽ        pp
//    Velocity mode (�ٶȿ���ģʽ)                 vl
//    Profile position mode����λ�ÿ���ģʽ)       pv
//    Torque profile mod (����ת�ؿ���ģʽ)        tq

//   Homing mode (ԭ��ع����ģʽ)                hm
//   Interpolated position mode (�岹λ�ÿ���ģʽ)      ip
//   Cyclic synchronous position mode (����λ�ÿ���ģʽ)  csp
//   Cyclic synchronous velocity mode (�����ٶȿ���ģʽ)  csv
//   Cyclic synchronous torque mode  (����ת�ؿ���ģʽ)   cst


//   NOT READY TO SWITCH ON ��������оƬ���磬�������ڳ�ʼ�����Լ죬��������δ���ã���״̬Ϊ�ڲ�״̬��
//   SWITCH ON DISABLED ������ʼ����ɣ������������������Ա��޸ģ���״̬û�жԵ�����磬��״̬Ϊ�û��ܹ������������״̬�������ϵ���û��Ӵ�����״̬��
//   READY TO SWITCH ON �����������Ա��޸ģ���������δ���ã��ȴ����� SWITCH ON״̬��
//   SWITCH ON �������ṩ�ߵ�ѹ�����ʷŴ��������������������Ա��޸ģ���������δ���á�
//   OPERATION ENABLE û�м�⵽���ϣ������������ã����Ե���ϵ������������Ա��޸ģ����� BP[N]������ ������״̬ɲ���Ƿ���Զ��ͷš�
//   QUICK STOP ACTIVE �����������Ա��޸ģ���ͣ�������ã������������ã���������ϵ�״̬��
//   FAULT REACTION ACTIVE �����������Ա��޸ģ����������˹��ϣ����Ϸ�Ӧ�������ã���������ͣ�ã���״̬�����ֶ����룬������������ʱ�Զ����롣





//0 Rcady to switchon �ŷ��޹���
//1 Switchedon �ȴ��ŷ�ʹ��
//2 Operation enabled �ŷ�����
//3 Fault ����
//4 Voltage enabled ��ͨ����·
//5 Quick stop ����ͣ��
//6 Switch on disabled �ŷ�׼����
//7 Warning ����
//8 Manufacturer specific �����Զ��壬��
//9 Remote Զ�̿���ѡ��
//10 Target reached Ŀ��λ�û��ٶ��Ƿ񵽴�
//11 Intermal limmit active ����ڲ�λ����λ
//12~13 Operation mode specific ģʽ���
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
/***********λ��S���߹滮*****************/
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
        CanOpenMode.CanOpen_Mode1State = 0;  //״̬Ҫ����
        Target_velocity = 0;
        Targetl_torque = 0;
    }

//    if(UartMode.CMD == 128) //������д���
//    {
//        UartMode.State1 = 0; //ǿ��������д���
//        UartMode.CMD = 0;
//    }

//    if(SystemError.SysErr)   //���ֹ��Ϻ�
//    {
//        UartMode.State1 = 0; //ǿ��������д���
//        UartMode.CMD = 0;
//    }		
		
	
    if(CanOpenMode.CiA402_RunMode == 1)  //�ٶ�      //��������ģʽ������Ӧ����
    {
//        if(Controlword == 0)
//        {
//            CanOpenMode.CanOpen_Mode1State = 0;
//        }
//        if(Controlword == 0x02)//ע��װ̬λ�����޷���
//        {
//            CanOpenMode.CanOpen_Mode1State = 7;
//           // CanOpenMode.CiA402_State = 0x617;
//        }
			
			  SystemError.RuningModeFdb = 11;
        switch(CanOpenMode.CanOpen_Mode1State)
        {
            case 0 :                   //��ʹ��
            {
                CanOpenMode.CiA402_State = 0x260;
                sendOnePDOevent(&Ds402_Slave_Data, 0);

                if(Controlword == 0x06)
                {

                    CanOpenMode.CanOpen_Mode1State = 1;
                }
            }
            break;

            case 1 :                  //��ʹ��
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

            case 4 :                  //��ʹ��
            {

                CanOpenMode.CanOpen_Mode1State = 5;
            }
            break;

            case 5 :                  //��ʹ�����
            {
                CanOpenMode.CanOpen_Mode1State = 6;
                CanOpenMode.CiA402_State = 0x637;
                sendOnePDOevent(&Ds402_Slave_Data, 0);
            }
            break;

            case 6:   //ֻ���ٶȱ䣬û��״̬�л�
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


    if(CanOpenMode.CiA402_RunMode == 3)  //�ٶ�
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

                if(Controlword == 0x02)         //ע��״ֵ̬���л�
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

            case 1 :                  //��ʹ��
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

            case 4 :                  //��ʹ��

            {

                CanOpenMode.CanOpen_Mode1State = 5;
            }
            break;

            case 5 :                  //��ʹ�����
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

            case 6:   //ֻ���ٶȱ䣬û��״̬�л�
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

                //	Profile_acceleration  ����ʱ�䣨ms��  0 �� 2000
                //	Profile_deceleration  ����ʱ�䣨ms��  0 �� 2000
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
        Statusword = CanOpenMode.CiA402_State + 0x8 + (shifttest << 14); //6041  ���ֹ��ϣ����ӹ��ϱ�־λ
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

