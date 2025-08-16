#include "pidtorque.h"
#include "ExternGlobals.h"


short  SpeedFdbpPost222 = 0;
short  tesetaa = 5;
short Errtmp = 0;
short  Normal_Mode_Flag = 0;
short v1_ValueMax, v2_ValueMax;

extern void FindTable(short speed, short torquecoeff, short* result1, short* result2);
void pidtorque_calc(PIDTorque *v)
{
    v->Err = v->Ref - v->Fdb;
    v->Up = ((int)v->Kp * (int)v->Err ) >> 2;

    // �������ο�����Ϊ800
    // ���Ť������ĵ�ѹΪ750*4
    Errtmp = SystemError.ThresholdCoeff * 3;

    Normal_Mode_Flag = 0;

    if((v->Ref > 0 ) && (v->Fdb > 0)) //ͬ���� �Ӽ���
    {
        if(v->Err > Errtmp)  //����  3000 -500 =2500 >1000
        {
            FindTable( v->Fdb, SystemError.TorquePIDLimitCoeff, &v1_ValueMax, &v2_ValueMax); //���ٶ�  �����ķ�ʽ  ���в��

            if(v->Out < v2_ValueMax)
            {
                v->Ui = ((int)(v1_ValueMax) << 15);
                v->Out = v2_ValueMax;
            }
            else  //�ظ��أ�������ͨ��PID
            {
                Normal_Mode_Flag = 1;
            }
        }
        else if(v->Err < (0 - Errtmp)) //����  500- 3000 = -2500 <-1000
        {
            FindTable( v->Fdb, 0 - SystemError.TorquePIDLimitCoeff, &v1_ValueMax, &v2_ValueMax); //���ٶ�  �����ķ�ʽ  ���в��

            if(v->Out > v2_ValueMax)
            {
                v->Ui = ((int)(v1_ValueMax) << 15);
                v->Out = v2_ValueMax;
            }
            else  //�ظ��أ�������ͨ��PID
            {
                Normal_Mode_Flag = 1;
            }
        }
        else   //����
        {
            Normal_Mode_Flag = 1;
        }
    }
    else if((v->Ref < 0 ) && (v->Fdb < 0)) //ͬ�� �Ӽ���
    {
        if(v->Err < 0 - Errtmp) //����   -3000 - -500 =-2500 <-1000
        {
            FindTable( v->Fdb, SystemError.TorquePIDLimitCoeff, &v1_ValueMax, &v2_ValueMax); //���ٶ�  �����ķ�ʽ  ���в��

            if(v->Out > 0 - v2_ValueMax)
            {
                v->Ui = 0 - ((int)(v1_ValueMax) << 15);
                v->Out = 0 - v2_ValueMax;
            }
            else  //�ظ��أ�������ͨ��PID
            {
                Normal_Mode_Flag = 1;
            }
        }
        else if(v->Err > Errtmp)  //����  -500 - -300  =2500 >1000
        {
            FindTable( v->Fdb, 0 - SystemError.TorquePIDLimitCoeff, &v1_ValueMax, &v2_ValueMax); //���ٶ�  �����ķ�ʽ  ���в��

            if(v->Out < 0 - v2_ValueMax)
            {
                v->Ui = 0 - ((int)(v1_ValueMax) << 15);
                v->Out = 0 - v2_ValueMax;
            }
            else  //�ظ��أ�������ͨ��PID
            {
                Normal_Mode_Flag = 1;
            }
        }
        else   //����
        {
            Normal_Mode_Flag = 1;
        }
    }
    else if((v->Ref < 0 ) && (v->Fdb > 0)) //���������
    {
        if(v->Err < (0 - Errtmp)) //���� -500 - 2000 =-2500  <-1000
        {

            FindTable( v->Fdb, 0 - SystemError.TorquePIDLimitCoeff, &v1_ValueMax, &v2_ValueMax); //���ٶ�  �����ķ�ʽ  ���в��

            if(v->Out > v2_ValueMax)
            {
                v->Ui = ((int)(v1_ValueMax) << 15);
                v->Out = v2_ValueMax;
            }
            else  //�ظ��أ�������ͨ��PID
            {
                Normal_Mode_Flag = 1;
            }
        }
        else   //����
        {
            Normal_Mode_Flag = 1;
        }
    }
    else if((v->Ref > 0 ) && (v->Fdb < 0))
    {
        if(v->Err > Errtmp)  //���� 500 - -2000 =2500  >1000
        {
            FindTable( v->Fdb, 0 - SystemError.TorquePIDLimitCoeff, &v1_ValueMax, &v2_ValueMax); //���ٶ�  �����ķ�ʽ  ���в��

            if(v->Out < 0 - v2_ValueMax)
            {
                v->Ui = 0 - ((int)(v1_ValueMax) << 15);
                v->Out = 0 - v2_ValueMax;
            }
            else  //�ظ��أ�������ͨ��PID
            {
                Normal_Mode_Flag = 1;
            }
        }
        else   //����
        {
            Normal_Mode_Flag = 1;
        }
    }


    if (Normal_Mode_Flag == 1)

    {
        if(v->Up > 3000) //��ֹ����ʱ��PD ���������ѹ�����MOS �𻵡�
        {
            v->Up = 3000;
        }
        else if(v->Up < -3000)
        {
            v->Up = -3000 ;
        }

        v->Ui += v->Ki * v->Err * tesetaa;

        if(v->Ui < v->UiMin)
        {
            v->Ui = v->UiMin;
        }

        if(v->Ui > v->UiMax)
        {
            v->Ui = v->UiMax;
        }

        v->OutPreSat = v->Up + (v->Ui >> 15);

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
    }
}
