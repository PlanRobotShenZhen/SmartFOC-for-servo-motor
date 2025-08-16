#include "IQmathLib.h"
#include "pid.h"
#include "ExternGlobals.h"
/*
#define USE_ONELOAD_TABLE 1
#if (USE_ONELOAD_TABLE == 1)
#define Uqs_Speed_Load_Table    Uqs_Speed_1_0_Load_Table
#define Offset_Speed_Load_Table Offset_Speed_1_0_Load_Table
#else
#define Uqs_Speed_Load_Table    Uqs_Speed_1_5_Load_Table
#define Offset_Speed_Load_Table Offset_Speed_1_5_Load_Table
#endif
//1��������µ������ٶ�-Uq��
short Uqs_Speed_1_0_Load_Table[31] =
{
    3500, 3500, 4320, 5020, 5750, 6540, 7310, 7910, 8710, 9320, 10260, 10980, 11740, 12420, 13410, 13740, //0-1500rpm
    14630, 15420, 16210, 16520, 17040, 18310, 19490, 19380, 20710, 21150, 22410, 22130, 23250, 23920, 25010	 //1600-3000rpm
};
//1��������µ������ٶ�-ƫ�Ʊ�
short Offset_Speed_1_0_Load_Table[31] =
{
    150, 150, 150, 250, 300, 300, 350, 400, 400, 450, 450, 450, 450, 500, 500, 550,//0-1500rpm
    550, 550, 550, 600, 600, 600, 600, 600, 600, 650, 650, 700, 700, 700, 700    //1600-3000rpm
};
//1.5��������µ������ٶ�-Uq��
short Uqs_Speed_1_5_Load_Table[31] =
{
    4930, 4930, 5630, 6460, 7220, 8010, 8600, 9450, 10210, 10950, 11880, 12610, 13310, 14280, 15350, 15700, //0-1500rpm
    16720, 17610, 18420, 18670, 19650, 20630, 21050, 21330, 21910, 23210, 24110, 25330, 25610, 27340, 27350 //1600-3000rpm
};
//1.5��������µ������ٶ�-ƫ�Ʊ�
short Offset_Speed_1_5_Load_Table[31] =
{
    150, 150, 200, 400, 400, 400, 450, 500, 500, 550, 550, 600, 600, 600, 600, 650, //0-1500rpm
    650, 650, 650, 700, 700, 700, 800, 800, 800, 800, 800, 800, 850, 850, 850     //1600-3000rpm
};
*/

//��ϵõ���Uqs-�ٶȹ�ʽ����ڶ���{626,1110}��ʾ100rpm��Uqs=626x+1110������x�ĵ�λ��1/4�ת��
short Uqs_Speed_Equation_Table[31][2] =
{
    // 0 �� 3000rpm��ÿһ����100rpm
    {626, 1110}, {626, 1110}, {617, 1876}, {659, 2460}, {667, 3147}, {666, 3934}, {687, 4540}, {715, 5109}, {725, 5846}, {746, 6496}, {803, 7029},
    {818, 7641}, {858, 8219}, {829, 9224}, {935, 9713}, {877, 10457}, {1000, 10780}, {972, 11557}, {970, 12440}, {921, 13057}, {930, 13811},
    {1001, 14607}, {965, 15253}, {831, 16344}, {1105, 16036}, {966, 17524}, {1017, 17897}, {1287, 17242}, {1175, 18616}, {1296, 19161}, {959, 21605}
};
//��ϵõ���Offest-�ٶȹ�ʽ���������{16,114}��ʾ200rpm��Offest=16x+114������x�ĵ�λ��1/4�ת��
short Offest_Speed_Equation_Table[31][2] =
{
    {0, 150}, {0, 150}, {16, 114}, {48, 57}, {50, 100}, {50, 100}, {50, 150}, {59, 150}, {59, 150}, {66, 157}, {59, 200},
    {66, 200}, {66, 200}, {66, 207}, {64, 222}, {75, 207}, {66, 257}, {64, 271}, {66, 271}, {75, 257}, {75, 257},
    {75, 257}, {79, 293}, {79, 293}, {79, 293}, {80, 300}, {87, 303}, {81, 840}, {89, 323}, {89, 323}, {89, 323}
};

void FindTable(short speed, short torquecoeff, short* result1, short* result2)
{
    short index = speed / 100; // �����ٶ����ڵı���������� (100 rpm Ϊһ������)
    short frac = speed - index * 100; // ��ȡʮλ�͸�λ��
    if(index < 30)
    {
        *result1 = (Uqs_Speed_Equation_Table[index][0] + (Uqs_Speed_Equation_Table[index + 1][0] - Uqs_Speed_Equation_Table[index][0]) * frac / 100) * torquecoeff;
			
        *result1 += Uqs_Speed_Equation_Table[index][1] + (Uqs_Speed_Equation_Table[index + 1][1] - Uqs_Speed_Equation_Table[index][1]) * frac / 100;
			
        *result2 = (Offest_Speed_Equation_Table[index][0] + (Offest_Speed_Equation_Table[index + 1][0] - Offest_Speed_Equation_Table[index][0]) * frac / 100) * torquecoeff;
			
        *result2 += Offest_Speed_Equation_Table[index][1] + (Offest_Speed_Equation_Table[index + 1][1] - Offest_Speed_Equation_Table[index][1]) * frac / 100;
    }
    else if(index >= 30)
    {
        *result1 = Uqs_Speed_Equation_Table[30][0] * torquecoeff + Uqs_Speed_Equation_Table[30][1];
        *result2 = Offest_Speed_Equation_Table[30][0] * torquecoeff + Offest_Speed_Equation_Table[30][1];
    }
}

//��������
//uint16_t KpGainCoeff = 30; //�л�PID��Kp�ı��ϵ��
//uint16_t KiGainCoeff = 30; //�л�PID��Ki�ı��ϵ��
//uint16_t ThresholdCoeff = 115; //�ż��л�ϵ�����������ֵ128���Եõ��ٷֱ�
//uint16_t TorqueCoeff = 6;       //���ڸı��ٶȵ����Ť��ֵ������ֵ������λΪ1/4�
//uint16_t TorquePIDLimitCoeff = 4;  //����PID�׶��޷������Ť��ֵ����λΪ1/4�
//uint16_t TorqueDECLimitCoeff = 0;  //����ͬ�����ʱ�޷������Ť��ֵ����λΪ1/4�

extern short SpeedFdbpFilter1;


void MixPID_calc(PIDC * v1, PID * v2, short vel_ref)
{
    static short LastSpeed = 0;     //��һ�ε�Ŀ���ٶ�
    static short SpeedThreshold = 0;//�ٶ��л��ż�
    static short SpeedDir = 0;      //Ŀ���ٶȷ���
    static short SpeedChangeDir = 0;//�ٶȸı䷽��
    static short Run_PID_Mode = 0;  //Ĭ��Ϊ0,Ϊ1��ֱ�Ӹ���,Ϊ2������PID
    static short Run_Mode = 0;      //Ϊ0��λ������Ϊ1����٣�Ϊ2�����
    static short v1_Value, v2_Value;//��Ų�����
    static short v1_ValueLimit, v2_ValueLimit;
    short vel_fb = SpeedFdbpFilter1;//MotorControler.SpeedFdbp;//SpeedFdbpFilter1;  //�����ٶ�

    if(LastSpeed != vel_ref) //Ŀ���ٶ��л�
    {
        SpeedChangeDir = LastSpeed < vel_ref ? 1 : -1;  //�ٶȱ仯����
        SpeedDir = vel_ref < 0 ? (short) -1 : (short)1;  //Ŀ���ٶȷ���

        //�����л��ż���������������л���ֻʹ�ã�Ŀ���ٶ�*�ż�ϵ������Ϊ�ż�������ʹ��ǰ���ٶȹ�ͬ����
        if(((vel_ref > 0) && (LastSpeed < 0)) || ((vel_ref < 0) && (LastSpeed > 0)))
        {
            SpeedThreshold = (short)(((int)SystemError.ThresholdCoeff * vel_ref) >> 7); //ThresholdCoeffΪ128ʱ��Ӧϵ��Ϊ1
        }
        else
        {
            SpeedThreshold = (short)((((int)SystemError.ThresholdCoeff * (vel_ref - LastSpeed)) >> 7) + LastSpeed);
        }

        Run_PID_Mode = 1;
        Run_Mode = 0;
        LastSpeed = vel_ref;

        //���
        short v1_ValueMax, v2_ValueMax;
        FindTable(SpeedDir * vel_ref, SystemError.TorquePIDLimitCoeff, &v1_ValueMax, &v2_ValueMax);

        //��PID�޷����޷�ֵֻ��Ŀ���ٶȾ���ֵ���޷�ϵ���й�
        v1->UiMax = ((int)(v1_ValueMax) << 15);
        v1->UiMin = -v1->UiMax;
        v1->OutMax = v1_ValueMax;
        v1->OutMin = -v1_ValueMax;
        v2->UiMax = ((int)(v2_ValueMax) << 15);
        v2->UiMin = -v2->UiMax;
        v2->OutMax = v2_ValueMax;
        v2->OutMin = -v2_ValueMax;

        //ÿ���ٶȸı䶼���¸�λPI����
        v1->Kp = pidpv_Kp / 10;
        v1->Ki = pidpv_Ki / 10;
        v2->Kp = 100;
        v2->Ki = 10;
    }

    // ȷ������ģʽ
    if(Run_PID_Mode == 1)
    {
        if(vel_fb == 0) //�����ٶ�Ϊ0
        {
            if(vel_ref == 0)        //��λ
            {
                Run_Mode = 0;
                Run_PID_Mode = 0;
            }
            else                    //��Ծ����
            {
                Run_Mode = 1;
            }
        }
        else if(Run_Mode == 4) //�ٶ������ı�ʱ������û�е���0��ʱ�̣�����ֱ�Ӵ������������෴������ʱ��Ҫ�ı�ģʽΪ����
        {
            if(((vel_fb > 0) && (vel_ref > 0)) || ((vel_fb < 0) && (vel_ref < 0)))
            {
                Run_Mode = 1;
            }
        }
        else           //�����ٶȲ�Ϊ0
        {
            if(Run_Mode == 0)
            {
                if(((vel_fb > 0) && (vel_ref > vel_fb)) || ((vel_fb < 0) && (vel_ref < vel_fb))) //ͬ�����
                {
                    Run_Mode = 1;
                }
                else if(((vel_ref > 0) && (vel_ref < vel_fb)) || ((vel_ref < 0) && (vel_ref > vel_fb)))  //ͬ�����
                {
                    Run_Mode = 2;
                    FindTable(SpeedDir * vel_ref, SystemError.TorqueDECLimitCoeff, &v1_ValueLimit, &v2_ValueLimit); //���õ�ͬ����ٵ��޷�ֵ
                }
                else if(vel_ref == 0)  //����Ϊ0
                {
                    Run_Mode = 3;
                }
                else if(((vel_ref > 0) && (vel_fb < 0)) || ((vel_ref < 0) && (vel_fb > 0))) //�ȼ����ٷ������
                {
                    Run_Mode = 4;
                }
            }
        }
    }

    //ȷ�������ٶȷ�����������ٶ�Ϊ0����ʹ��Ŀ���ٶȵķ���
    short fb_dir = SpeedDir;
    if(vel_fb != 0)
    {
        fb_dir = vel_fb < 0 ? (short) -1 : (short)1;
    }

    // ��������ģʽ��������ֵ
    if(Run_Mode == 0)
    {
        v1_Value = 0;
        v2_Value = 0;
        SpeedThreshold = 0;
        SpeedChangeDir = 0;
    }
    else
    {
        short torqueCoeff = 0;
        if(Run_Mode == 1)        //����
        {
            torqueCoeff = (short)(SystemError.TorqueCoeff);
        }
        else if(Run_Mode >= 2)  //����
        {
            torqueCoeff = (short)(-SystemError.TorqueCoeff);
        }

        //�����ٶȲ��
        FindTable(fb_dir * vel_fb, torqueCoeff, &v1_Value, &v2_Value);
        v1_Value = fb_dir * v1_Value;
        v2_Value = fb_dir * v2_Value;

        if(Run_Mode == 2)   //ͬ�������Ҫ�޷����
        {
            if(fb_dir > 0)
            {
                v1_Value = v1_Value <  v1_ValueLimit ? v1_ValueLimit : v1_Value;
                v2_Value = v2_Value <  v2_ValueLimit ? v2_ValueLimit : v2_Value;
            }
            else
            {
                v1_Value = v1_Value > (-v1_ValueLimit) ? (-v1_ValueLimit) : v1_Value;
                v2_Value = v2_Value > (-v2_ValueLimit) ? (-v2_ValueLimit) : v2_Value;
            }
        }
    }

    // ���ٶȽ����ж���ֱ�Ӹ�����������л�PID
    if(Run_PID_Mode == 1)      //Ϊ1��ֱ�Ӹ���
    {
        //����仯���ٶ�С����ֵ �� ����仯���ٶȴ�����ֵ
        if(((SpeedChangeDir == 1) && (vel_fb <= SpeedThreshold)) || ((SpeedChangeDir == -1) && (vel_fb >= SpeedThreshold)))
        {
            v1->Out = v1_Value;
            v2->Out = v2_Value;
        }
        else   //�л�
        {
            v1->Ui = (int)(v1_Value << 15);
            v1->Out = v1_Value;
            v2->Ui = (int)(v2_Value << 15);
            v2->Out = v2_Value;

            v1->Ki = (short)(v1->Ki * SystemError.KiGainCoeff);
            v1->Kp = (short)(v1->Kp * SystemError.KpGainCoeff);
            v2->Ki = (short)(v2->Ki * SystemError.KiGainCoeff);
            v2->Kp = (short)(v2->Kp * SystemError.KpGainCoeff);
            Run_PID_Mode = 2;
            Run_Mode = 0;
        }
    }

    // ����PID
    if(Run_PID_Mode == 2)
    {
        v2->Ref = 0;
        if((svpwm.UQs > 500) && (MotorControler.SpeedFdbp > 0))
        {
            v2->Fdb = 0 - svpwm.IDs;
        }
        else if((svpwm.UQs < -500) && (MotorControler.SpeedFdbp < 0))
        {
            v2->Fdb = svpwm.IDs;
        }
        else
        {
            v2->Fdb = 0;
            v2->Ref = 0;
        }
        v2->calc(v2);

        //���Ť���޷�
        UqsPid.Fdb = svpwm.IQs;
        if((svpwm.UQs >= 0) && (MotorControler.SpeedFdbp >= 0))
        {
            UqsPid.Ref = svpwm.CurrentRef;
            if(UqsPid.Ui > 0)
            {
                UqsPid.Ui = 0;
            }
            if(UqsPid.Out > 0)
            {
                UqsPid.Out = 0;
            }
        }
        else if((svpwm.UQs < 0) && (MotorControler.SpeedFdbp < 0))
        {
            UqsPid.Ref = 0 - svpwm.CurrentRef;
            if(UqsPid.Ui < 0)
            {
                UqsPid.Ui = 0;
            }
            if(UqsPid.Out < 0)
            {
                UqsPid.Out = 0;
            }
        }
        UqsPid.calc(&UqsPid);  //���ڵ����޷�
    }
    else
    {
        UqsPid.Out = 0;
        UqsPid.Ui = 0;
    }
}

void pid_calc(PID *v)
{
    static short SpeedFdbpPost3 = 0;
    v->Err = v->Ref - v->Fdb;

    if(v->Err > 26000)
    {
        v->Err = SpeedFdbpPost3;
    }

    if(v->Err < -26000)
    {
        v->Err = SpeedFdbpPost3;
    }

    SpeedFdbpPost3 = v->Err;

    v->Up = v->Kp * v->Err;

    v->OutPreSat = (v->Up + v->Ui) >> 15;

    if(v->OutPreSat > v->OutMax)
    {
        v->Out =  v->OutMax;
    }
    else if(v->OutPreSat < v->OutMin)
    {
        v->Out =  v->OutMin;
    }
    else
    {
        v->Out = v->OutPreSat;
    }

    v->Ui += v->Ki * v->Err;
    if(v->Ui < v->UiMin)
    {
        v->Ui = v->UiMin;
    }

    if(v->Ui > v->UiMax)
    {
        v->Ui = v->UiMax;
    }
}
