#include "pidIqs.h"
#include "ExternGlobals.h"

void pidIqs_calc(PIDIqs *v)
{
    v->Err = v->Ref - v->Fdb;
    v->Up = v->Kp * v->Err;

    if(v->Up > 3000) //防止抖动时，PD 产生极大电压，造成MOS 损坏。
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

//    if((v->Fdb < 300) && ( v->Fdb > -300))//停止时误 动作
//    {
//        v->Up = 0;
//        v->Ui = 0;
//    }

//    if((v->UQs >= 0) && (v->SpeedFdbp >= 0))  //正向
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
//    else if((v->UQs < 0) && (v->SpeedFdbp <= 0))  // 反向
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

