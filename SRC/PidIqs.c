#include "pidIqs.h"
#include "ExternGlobals.h"

void pidIqs_calc(PIDIqs *v)
{
    v->Err = v->Ref - v->Fdb;
    v->Up = v->Kp * v->Err;

    if(v->Up > 3000) //��ֹ����ʱ��PD ���������ѹ�����MOS �𻵡�
    {
        v->Up = 3000 ;
    }
    else if(v->Up < -3000)
    {
        v->Up = -3000;
    }

    v->Ui += v->Ki * v->Err * 100;

    if(v->Ui > v->UiMax)
    {
        v->Ui = v->UiMax;
    }

    if(v->Ui < v->UiMin)
    {
        v->Ui = v->UiMin;
    }

//    if((v->Fdb < 300) && ( v->Fdb > -300))//ֹͣʱ�� ����
//    {
//        v->Up = 0;
//        v->Ui = 0;
//    }

//    if((v->UQs >= 0) && (v->SpeedFdbp >= 0))  //����
//    {
//        v->Ref = v->CurrentRef;

//        if(v->Ui > 0)
//        {
//            v->Ui = 0;
//        }

//        if(v->Up > 0)
//        {
//            v->Up = 0;
//        }
//    }
//    else if((v->UQs < 0) && (v->SpeedFdbp <= 0))  // ����
//    {
//        v->Ref = 0 - v->CurrentRef;

//        if(v->Ui < 0)
//        {
//            v->Ui = 0;
//        }

//        if(v->Up < 0)
//        {
//            v->Up = 0;
//        }
//    }

    v->OutPreSat = v->Up + ((v->Ui) >> 15);

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

