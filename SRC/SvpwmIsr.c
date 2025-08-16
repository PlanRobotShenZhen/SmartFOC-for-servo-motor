
#include "ExternGlobals.h"
#include "Spi.h"
#include "Ds402_Slave.h"
#include "CanOpenMode.h"
#include "Function.h"
extern Set_SP_Para PostionPlanCiA402Mode_1;
extern int PostionPlanStepCount;
extern int PostionPlanStepMax;
extern int PostionPlanStep;
extern bool postion_direction_flag;
int UQs_test=0;
int Torque_limit=1000;
short svpwm_test=0;



Current_value value;
/************************ SVPWM����״̬�궨�� ************************/
#define SVPWM_STATE_DISABLE           0   // �ر�״̬����ʼ������������
#define SVPWM_STATE_VF_MODE           1   // V/F����ģʽ
#define SVPWM_STATE_FIND_ZERO         2   // Ѱ�����
#define SVPWM_STATE_BASIC_TEST        3   // �������ܲ���ģʽ
#define SVPWM_STATE_POS_LOCK          4   // ʹ�ܺ�λ������ģʽ
#define SVPWM_STATE_CURRENT_MODE      5   // ��������ģʽ
#define SVPWM_STATE_SPEED_MODE        6   // �ٶȿ���ģʽ����Ӧ��λ���ٶȿ��ƣ�
#define SVPWM_STATE_SPEED_MODE2       61  // �ٶȿ���ģʽ2��δʹ�ã�
#define SVPWM_STATE_POS_MODE          7   // λ�ñջ�����ģʽ
#define SVPWM_STATE_POS_LOCK_TEST     10  // λ����������ģʽ���������ã�

/************************ �������״̬�궨�� ************************/
#define MOTOR_STATE_STOP              0   // ���ֹͣ״̬����ʼ��������
#define MOTOR_STATE_VF_RUN            1   // V/Fģʽ����
#define MOTOR_STATE_FIND_ZERO         2   // Ѱ����� 
#define MOTOR_STATE_BASIC_TEST        3   // �������ܲ���
#define MOTOR_STATE_LOCK_AFTER_ENABLE 4   // ʹ�ܺ��λ������
#define MOTOR_STATE_TORQUE_CONTROL    5   // ת�ؿ���ģʽ
#define MOTOR_STATE_SPEED_CONTROL     6   // �ٶȱջ�����
#define MOTOR_STATE_SPEED_CONTROL2    61  // �ٶȱջ�����2�����ã�
#define MOTOR_STATE_POS_HOLD          7   // λ�ñ���ģʽ
#define MOTOR_STATE_POS_MOVE          71  // λ�ù滮�˶�ģʽ��ִ��λ�ù켣��

int Luenberger_Observer(int Theta,int Iq);
int angle=0;
int test_omega=0;

#define Speed_Loop_ADRC_Controller 0
#define Speed_Loop_PI_Controller   1



float ADRC(float v,float y);


void SpeedCalculate(void);   // 1K    HZ
void MotorControl(void);     // 1K    HZ
void SvpwmControl(void) ;    // 16K   HZ
void CurrentLoopISR(void);   // 16K   HZ
void PID_IdIq(void);         // 16K   HZ
short  Moving_Average_Window_Filter(short input, short CHANNEL_ID,short width);
short  Moving_Average_Window_Filter_4096(short input, short CHANNEL_ID,short width);
short tmp20 = 0;             //��������
short speedref_test=0;
int As_mean;
int Bs_mean;
int TEMP_A;
int TEMP_B;
int TEMP_Uq_filter32;
int TEMP_Uq_filter8;

void CurrentLoopISR(void)   //CurrentLoopISR 16K HZ
{

    if(SystemError.ImeasOffsetFlag < 10000)
    {
        SystemError.ImeasAOffset =  Ids_filter(SystemError.ImeasA);
        SystemError.ImeasBOffset =  Ids_filter2(SystemError.ImeasB);
        SystemError.ImeasCOffset =  Ids_filter3(SystemError.ImeasC);
        SystemError.ImeasOffsetFlag++;
        SystemError.SysErr = 0;
    }
    else
    {
        svpwm.As = SystemError.ImeasA - SystemError.ImeasAOffset;
        svpwm.Bs = SystemError.ImeasB - SystemError.ImeasBOffset;
    }

    svpwm.Angle2 = ((int)(((MotorControler.AngleFromMT6835 + MotorControler.AngleFromMT6835Offset1) & 0x7fff) * MotorControler.MotorPoles)) & 0x7fff;
	angle=((MotorControler.AngleFromMT6835 + MotorControler.AngleFromMT6835Offset1) & 0x7fff)*360/32768;
	
    clarke(&svpwm);

    PID_IdIq();

    SvpwmControl(); // 16K   HZ


    if(svpwm.UQs < svpwm.UQstmp1)
    {
        svpwm.UQs = svpwm.UQs + svpwm.UQstmp2;

    }

    if(svpwm.UQs > svpwm.UQstmp1)
    {
        svpwm.UQs = svpwm.UQs - svpwm.UQstmp2;

    }

//    if((SystemError.SysErr) && (SystemError.SysErr != M_SYSERR_CODER)) //����ϵͳ���ϣ�ǿ�ƹر����
//    {
//        svpwm.UQs = 0;
//        svpwm.UDs = 0;
//    }

    if((svpwm.UQs < -650) || (svpwm.UQs > 650))
    {
        SystemError.MotorRunFlag	= 1;
    }
    else
    {
        SystemError.MotorRunFlag	= 0;
    }
	
    ipark(&svpwm);
    svgendq(&svpwm);
    PWM(&svpwm);
    TIM1->CCDAT1 = svpwm.Va; // U��ռ�ձ�
    TIM1->CCDAT2 = svpwm.Vb; // V��ռ�ձ�
    TIM1->CCDAT3 = svpwm.Vc; // W��ռ�ձ�
}


void PID_IdIq(void)
{
	if(UartMode.Mode == 2)
	{
		Torque_limit=UartMode.Torque;
	}
	else
	{
		Torque_limit=1000;
	}
	
    //D�������
    UdsPid.Ref = 0;

    if((svpwm.UQs > 500) && (MotorControler.SpeedFdbp > 0))
    {
        UdsPid.Fdb = 0 - svpwm.IDs;
    }
    else if((svpwm.UQs < -500) && (MotorControler.SpeedFdbp < 0))
    {
        UdsPid.Fdb = svpwm.IDs;
    }
    else
    {
        UdsPid.Fdb = 0;
        UdsPid.Ref = 0;
    }

    UdsPid.calc(&UdsPid);

    //Q�������
    UqsPid.Fdb        = svpwm.IQs;
    UqsPid.CurrentRef = svpwm.CurrentRef;
    UqsPid.SpeedFdbp  = MotorControler.SpeedFdbp;
    UqsPid.UQs        = svpwm.UQs;
	UqsPid.Ref        = pidpv.Out*1000/pidpv_OutMax;//iq15(0.9)=29,491
    UqsPid.calc(&UqsPid);//���ڵ����޷�
    MotorControler.AngleFromMT6835Offset = MotorControler.AngleFromMT6835Offset1;// + UdsPid.Out
}



void SvpwmControl(void)   // 16K HZ
{
    static short RotorElectricalMacAngle = 0;
    int   RotorMacAngleTemp = 0;

    switch(svpwm.SvpwmControlState)
    {
        case 0 ://��ʹ��
            PwmShut();
            SystemError.PwmOn = 0;
            svpwm.DcCoeff = SystemError.DcCoeff;//32400; //_IQ(0.99);
            svpwm.PeriodMax = 4499;
            svpwm.MfuncPeriod = 0x7fff;

            svpwm.UDs = 0;
            svpwm.UQstmp1 = 0;
            svpwm.UQs = 0;
            RotorElectricalMacAngle = 0;

            pidpv.Ui = 0;
            pidholding.Ui = 0;
            //pidc_position.Ui = 0;
            UdsPid.Ui = 0;
            UqsPid.Ui = 0;

            UdsPid.OutMax = _IQ15(0.3);
            UdsPid.OutMin = _IQ15(-0.3);
            UdsPid.UiMax = _IQ30(0.2);
            UdsPid.UiMin = _IQ30(-0.2);
            UdsPid.Kp = 10;
            UdsPid.Ki = 30;
            UdsPid.Err = 0;

            UqsPid.OutMax = _IQ15(0.95); // UqsPid  ���޷�����
            UqsPid.OutMin = _IQ15(-0.95);

            UqsPid.Kp = 80;
            UqsPid.Ki = 50;
            UqsPid.Err = 0;

            UqsPid.UiMax = pidpv_UiMax;
            UqsPid.UiMin = pidpv_UiMin;

			
            svpwm.UQstmp2 = 30;

            break;

        case 1 :    //  V/Fģʽ
            svpwm.DcCoeff = SystemError.DcCoeff;
            SystemError.PwmOn = 1;
            PwmOpen();
            svpwm.UQstmp1 = SystemVar.VF_Voltage;
            svpwm.Angle = SystemVar.VF_ElectricalAngle;

            break;

        case 2 :    //  Ѱ�����
            svpwm.DcCoeff = SystemError.DcCoeff;
            SystemError.PwmOn = 1;
            PwmOpen();

            RotorElectricalMacAngle = RotorElectricalMacAngle + 10;
            RotorElectricalMacAngle = RotorElectricalMacAngle & 0x7fff;
            svpwm.Angle = RotorElectricalMacAngle;
            break;

        case 3 :    //  ��������
            svpwm.DcCoeff = SystemError.DcCoeff;
            SystemError.PwmOn = 1;
            PwmOpen();

            RotorMacAngleTemp = (int)(((MotorControler.AngleFromMT6835 + MotorControler.AngleFromMT6835Offset) & 0x7fff) * MotorControler.MotorPoles);
            RotorElectricalMacAngle = RotorMacAngleTemp & 0x7fff;
            svpwm.UQs = SystemVar.test_uqs;
            svpwm.Angle = RotorElectricalMacAngle;
            break;

        case 4 ://  ��ʹ��  λ������

            svpwm.DcCoeff = SystemError.DcCoeff;
			pidholding.OutMax = pidholding_OutMax;
            pidholding.OutMin = pidholding_OutMin;
            pidholding.UiMax = pidholding_UiMax;
            pidholding.UiMin = pidholding_UiMin;

            pidholding.Kp = 30;
            pidholding.Ki = pidholding_Ki;                
            SystemError.PwmOn = 1;
            PwmOpen();
            RotorMacAngleTemp = (int)(((MotorControler.AngleFromMT6835 + MotorControler.AngleFromMT6835Offset) & 0x7fff) * MotorControler.MotorPoles);
            RotorElectricalMacAngle = RotorMacAngleTemp & 0x7fff;
            svpwm.Angle = RotorElectricalMacAngle;

		
			pidholding.Ref = MotorControler.PositionRef;
		
            pidholding.Fdb = MotorControler.MotorActivePostion;
            pidholding.Speedref = 0;
            pidholding.Speedfdb = MotorControler.SpeedFdbp;
            pidholding.Iqsfdb = svpwm.IQs;

            pidholding.calc(&pidholding);

            svpwm.UQstmp1 = pidholding.Out;
            UqsPid.Ui = 0;
            UqsPid.Out = 0;			//�����󣬻���һ���ֵ���
            break;

        case 5 :   // ����ģʽ
            svpwm.DcCoeff = SystemError.DcCoeff;
            SystemError.PwmOn = 1;
            PwmOpen();
            RotorMacAngleTemp = (int)(((MotorControler.AngleFromMT6835 + MotorControler.AngleFromMT6835Offset) & 0x7fff) * MotorControler.MotorPoles);
            RotorElectricalMacAngle = RotorMacAngleTemp & 0x7fff;
            svpwm.Angle = RotorElectricalMacAngle;
			svpwm.UQstmp1 = UqsPid.Out;
            break;

        case 6 :   // �ٶ�ģʽ
            svpwm.DcCoeff = SystemError.DcCoeff;
            SystemError.PwmOn = 1;
            PwmOpen();
            RotorMacAngleTemp = (int)(((MotorControler.AngleFromMT6835 + MotorControler.AngleFromMT6835Offset) & 0x7fff) * MotorControler.MotorPoles);
            RotorElectricalMacAngle = RotorMacAngleTemp & 0x7fff;
            svpwm.Angle = RotorElectricalMacAngle;
			svpwm.UQstmp1 = UqsPid.Out;

            break;



        case 7 :   //λ�û�ģʽ


		svpwm.DcCoeff = SystemError.DcCoeff;
        PwmOpen();
        RotorMacAngleTemp = ((int)(((MotorControler.AngleFromMT6835 + MotorControler.AngleFromMT6835Offset) & 0x7fff))) * ((
                                int)MotorControler.MotorPoles);
        RotorElectricalMacAngle = RotorMacAngleTemp & 0x7fff;
        svpwm.Angle = RotorElectricalMacAngle;
		svpwm.UQstmp1=UqsPid.Out;
            break;

        case 10 ://  ��ʹ��  λ������  ������
            svpwm.DcCoeff = SystemError.DcCoeff;
            SystemError.PwmOn = 1;
            PwmOpen();
            RotorMacAngleTemp = (int)(((MotorControler.AngleFromMT6835 + MotorControler.AngleFromMT6835Offset) & 0x7fff) * MotorControler.MotorPoles);
            RotorElectricalMacAngle = RotorMacAngleTemp & 0x7fff;
            svpwm.Angle = RotorElectricalMacAngle;

            pidholding.Ref = 0;
            pidholding.Fdb = MotorControler.MotorActivePostion;
            pidholding.calc(&pidholding);
            svpwm.UQs = pidholding.Out;
            break;

        default:
        {
        }
        break;
    }
}




void MotorControl(void)  //1KHZ
{
    static short ExampleCount = 0;
    static short VF_Dir = 0;              //����ģʽ
    static int RotorMacAngleTemp1 = 0;    //����ϱ�
    static int RotorMacAngleTemp2 = 0;    //����ϱ�
    static short RotorMacAngleTemp3 = 0;  //����ϱ�

     ExampleCount++;

    if(ExampleCount >= 10)
    {
        ExampleCount = 0;

        switch(MotorControler.State)
        {
            case 0 :
            {
                svpwm.SvpwmControlState = 0; //
                MotorControler.Error = 0;
                /*****VF��������**************/
                SystemVar.VF_ElectricalAngleStep = 0;
                SystemVar.VF_ElectricalAngle = 0;
                VF_Dir = 0;
                SystemVar.VF_Voltage = 0;

                RotorMacAngleTemp3 = 0;
                RotorMacAngleTemp2 = 0;
                svpwm.CurrentRef = 0;  //���ת������
            }
            break;

            case 1 :         //��������
            {
                svpwm.SvpwmControlState = 1;

                SystemVar.VF_ElectricalAngleStepCount++;

                if(SystemVar.VF_ElectricalAngleStepCount >= 100)
                {
                    SystemVar.VF_ElectricalAngleStepCount = 0;

                    if((SystemVar.VF_ElectricalAngleStep < SystemVar.VF_ElectricalAngleStepMax) && (SystemVar.VF_ElectricalAngleStepMin == 0))
                    {
                        SystemVar.VF_ElectricalAngleStep = SystemVar.VF_ElectricalAngleStep + 2;
                        VF_Dir = 1;
                    }

                    if((SystemVar.VF_ElectricalAngleStep > SystemVar.VF_ElectricalAngleStepMin) && (SystemVar.VF_ElectricalAngleStepMax == 0))
                    {
                        SystemVar.VF_ElectricalAngleStep = SystemVar.VF_ElectricalAngleStep - 2;
                        VF_Dir = -1;
                    }

                    if((SystemVar.VF_ElectricalAngleStepMax != 0) && (SystemVar.VF_ElectricalAngleStepMin != 0))
                    {
                        SystemVar.VF_ElectricalAngleStep = 0;
                        SystemVar.VF_ElectricalAngle = 0;
                        VF_Dir = 0;
                        SystemVar.VF_Voltage = 0;
                    }

                    if((SystemVar.VF_ElectricalAngleStepMax == 0) && (SystemVar.VF_ElectricalAngleStepMin == 0))
                    {
                        SystemVar.VF_ElectricalAngleStep = 0;
                        SystemVar.VF_ElectricalAngle = 0;
                        VF_Dir = 0;
                        SystemVar.VF_Voltage = 0;
                    }
                    //����ʵ��Ч���������޷�������
                }
                SystemVar.VF_ElectricalAngle = (SystemVar.VF_ElectricalAngle + ((short)((((float)SystemVar.VF_ElectricalAngleStep)) * 2.731f))) & 0x7FFF;
                SystemVar.VF_Voltage = (SystemVar.VF_ElectricalAngleStep >> 1) * SystemVar.VF_Coefficient + (SystemVar.VF_Coefficient_B * VF_Dir);
            }
            break;

            case 2 : //�����
            {
                RotorMacAngleTemp1 = ((int)(((MotorControler.AngleFromMT6835 + RotorMacAngleTemp2) & 0x7fff) * MotorControler.MotorPoles)) & 0x7fff;
                SystemVar.VF_ElectricalAngle = 0;
                SystemVar.VF_Voltage = 2100;
                svpwm.SvpwmControlState = 1;
                RotorMacAngleTemp3++;

                if(RotorMacAngleTemp3 >= 1000)
                {
                    RotorMacAngleTemp3 = 1000;
                    RotorMacAngleTemp2 = RotorMacAngleTemp2 + 1;
                    RotorMacAngleTemp2 = RotorMacAngleTemp2 & 0x7fff;

                    if((RotorMacAngleTemp1 > 8188) && (RotorMacAngleTemp1 < 8196))
                    {
                        MotorControler.AngleFromMT6835Offset1 = RotorMacAngleTemp2;
                        MotorControler.State = 0;
                    }
                }
            }
            break;

            case 3 :                //��������
            {
                svpwm.SvpwmControlState = 3;
            }
            break;

            case 4 :               //ʹ�ܺ�����λ��
            {
                svpwm.SvpwmControlState = 4;
                pidholding.Ref = MotorControler.PositionRef;

                pidpv.OutMax = pidpv_OutMax;
                pidpv.OutMin = pidpv_OutMin;
                pidpv.UiMax = pidpv_UiMax;
                pidpv.UiMin = pidpv_UiMin;
                pidpv.Kp = 10;
                pidpv.Ki = pidpv_Ki;
            }
            break;

            case 5:
            {
                svpwm.SvpwmControlState = 5;
                pidpv.Fdb = MotorControler.SpeedFdbp;
				pidpv.calc(&pidpv);
			}
            break;

            case 6:                       //�ٶȻ�
            {
                svpwm.SvpwmControlState = 6;
                svpwm.CurrentRef = MotorControler.TorqueRef; // //ת������

                if(pidpv.Ref < MotorControler.SpeedRef)
                {
                    pidpv.Ref = pidpv.Ref + MotorControler.SpeedAcc;

                    if(pidpv.Ref >= MotorControler.SpeedRef)
                    {
                        pidpv.Ref = MotorControler.SpeedRef;
                    }
                }
                else if(pidpv.Ref > MotorControler.SpeedRef)
                {
                    pidpv.Ref = pidpv.Ref - MotorControler.SpeedDcc;

                    if(pidpv.Ref <= MotorControler.SpeedRef)
                    {
                        pidpv.Ref = MotorControler.SpeedRef;
                    }
                }

                pidpv.Fdb = MotorControler.SpeedFdbp;
					float k1=0.5;
					float k2=0.5;
					if(MotorControler.SpeedRef<=50&&MotorControler.SpeedRef>=-50)
					{
						k1=0.5;
						k2=0.5;
					}
					if(MotorControler.SpeedRef>50&&MotorControler.SpeedRef<100)
					{
						k1=MotorControler.SpeedRef/100;
						k2=-MotorControler.SpeedRef/100+1;
					}
					if(MotorControler.SpeedRef<-50&&MotorControler.SpeedRef>-100)
					{
						k1=-MotorControler.SpeedRef/100;
						k2=MotorControler.SpeedRef/100+1;
					}
					if(MotorControler.SpeedRef>=100||MotorControler.SpeedRef<=-100)
					{
						k1=1;
						k2=0;
					}
					
					pidpv.calc(&pidpv);
					pidpv.Out=k1*pidpv.Out+k2*ADRC(MotorControler.SpeedRef, MotorControler.SpeedFdbp);

		
                
            }
            break;


            case 7 :
                svpwm.SvpwmControlState = 4;
                pidholding.Ref = (int)(value.position);
                break;


            case 71:
            {

            //pidc_position
            value = SpeedPlant_positionControl(&PostionPlanCiA402Mode_1, PostionPlanStepCount);
            //�ٶ�ת��ʱ������rpmת��cnt/ms������S�����ڲ���ʱ�䵥λ��ms�����������1�����Ǽ�0.001
            PostionPlanStepCount = PostionPlanStepCount + PostionPlanStep;
            svpwm.SvpwmControlState = SVPWM_STATE_POS_MODE;
            svpwm.CurrentRef = MotorControler.TorqueRef; //ת������
            pidc_position.Ref = value.position;
            pidc_position.Speedref = (short)(value.vel * 1.831f);
            pidc_position.Speedfdb = MotorControler.SpeedFdbp;
            pidc_position.Fdb = MotorControler.MotorActivePostion;
            pidc_position.Iqs = svpwm.IQs;
            pidc_position.calc(&pidc_position);
			
            //λ�û������ֵ�ٶȻ�Ref
			pidpv.Kp = 30;
			pidpv.Ki = 30;
            pidpv.Ref = 1000*pidc_position.Out/pidc_position.OutMax;
              

	
			 pidpv.Fdb=MotorControler.SpeedFdbp;
             pidpv.calc(&pidpv);

//						
            if ((PostionPlanStepCount > (PostionPlanStepMax))||((MotorControler.MotorActivePostion>UartMode.TargetPosition)&&(postion_direction_flag==1))||((MotorControler.MotorActivePostion<UartMode.TargetPosition)&&(postion_direction_flag==0))) 
				{
					MotorControler.State = MOTOR_STATE_POS_HOLD;
					svpwm.SvpwmControlState = SVPWM_STATE_POS_LOCK;
					pidholding.Ui =	0;
					MotorControler.PositionRef = MotorControler.MotorActivePostion;
				}
            }
            break;

            default:
            {

            }
            break;
        }
    }
}
//1�������һ��
//1Ȧ32767

void SpeedCalculate(void)
{
    static short t1 = 0;
    static short t2 = 0;

    static int   sum1 = 0;
    static int   sum2 = 0;
    static int   sum3 = 0;
    static short buffer1[16];
    static short buffer2[32];
    static short buffer3[64];

    static short point_a1 = 0;
    static short flag1 = 0;

    static short point_a2 = 0;
    static short flag2 = 0;

    static short point_a3 = 0;
    static short flag3 = 0;

    static short SpeedLoop = 0;

    int speedfdb_tmp = 0;

    if((MotorControler.AngleFromMT6835 >> 12) == 0)
    {
        if(t2 == 1)
        {
            MotorControler.RotorCount++;
            t2 = 0;
        }

        t1 = 1;
    }
    else if((MotorControler.AngleFromMT6835 >> 12) == 7)
    {
        if(t1 == 1)
        {
            MotorControler.RotorCount--;
            t1 = 0;
        }

        t2 = 1;
    }
    else if((MotorControler.AngleFromMT6835 >> 12) == 3)
    {
        t1 = 0;
        t2 = 0;
    }


    MotorControler.MotorActivePostion = (MotorControler.RotorCount * 0x7fff) + ((MotorControler.AngleFromMT6835 + 0) & 0x7fff);
    SpeedLoop++;

    if(SpeedLoop >= 10)
    {
        SpeedLoop = 0;

        speedfdb_tmp = MotorControler.AngleFromMT6835 - SystemError.RotorMacAnglePastp;

        SystemError.RotorMacAnglePastp = MotorControler.AngleFromMT6835;

        if(speedfdb_tmp > 5000)
        {
            speedfdb_tmp = SystemError.SpeedFdbpPost;
        }

        if(speedfdb_tmp < -5000)
        {
            speedfdb_tmp = SystemError.SpeedFdbpPost;
        }

        SystemError.SpeedFdbpPost = speedfdb_tmp;

        MotorControler.SpeedFdbp = (short)((speedfdb_tmp * 18311) / 10000);

        /*************************************************************/
        short temp = buffer1[point_a1];
        sum1 = sum1 + MotorControler.SpeedFdbp;
        buffer1[point_a1] = MotorControler.SpeedFdbp;
        point_a1++;

        if(flag1 == 0)
        {
            MotorControler.SpeedFdbpFilter1 = (short)(sum1 / point_a1);

            if(point_a1 >= 16)
            {
                flag1 = 1;
                point_a1 = 0;
            }
        }
        else
        {
            sum1 = sum1 - temp;
            MotorControler.SpeedFdbpFilter1 = (short)(sum1 / 16);

            if(point_a1 >= 16)
                point_a1 = 0;
        }

        /*************************************************************/
        temp = buffer2[point_a2];
        sum2 = sum2 + MotorControler.SpeedFdbp;
        buffer2[point_a2] = MotorControler.SpeedFdbp;
        point_a2++;

        if(flag2 == 0)
        {
            MotorControler.SpeedFdbpFilter2 = (short)(sum2 / point_a2);

            if(point_a2 >= 32)
            {
                flag2 = 1;
                point_a2 = 0;
            }
        }
        else
        {
            sum2 = sum2 - temp;
            MotorControler.SpeedFdbpFilter2 = (short)(sum2 / 32);

            if(point_a2 >= 32)
                point_a2 = 0;
        }

        /*************************************************************/
        temp = buffer3[point_a3];
        sum3 = sum3 + MotorControler.SpeedFdbp;
        buffer3[point_a3] = MotorControler.SpeedFdbp;
        point_a3++;

        if(flag3 == 0)
        {
            MotorControler.SpeedFdbpFilter3 = (short)(sum3 / point_a3);

            if(point_a3 >= 64)
            {
                flag3 = 1;
                point_a3 = 0;
            }
        }
        else
        {
            sum3 = sum3 - temp;
            MotorControler.SpeedFdbpFilter3 = (short)(sum3 / 64);

            if(point_a3 >= 64)
                point_a3 = 0;
        }


    }

    Velocity_actual_value = MotorControler.SpeedFdbp;
}
