#include "pidids.h"
 
//d轴电流曲线待测试
void pidids_calc(PIDIds *v)
{
    v->Err = v->Ref - v->Fdb;
    v->Up = (v->Kp * v->Err)>>2;  //待测试

    if(v->Up > 1000) //防止抖动时，PD 产生极大电压，造成MOS 损坏。
    {
        v->Up = 1000 ;
    }
    else if(v->Up < -1000)
    {
        v->Up = -1000;
    }

    v->Ui += v->Ki * v->Err*10; //待测试
		
    if(v->Ui > v->UiMax)
    {
        v->Ui = v->UiMax;
    }

    if(v->Ui < v->UiMin)
    {
        v->Ui = v->UiMin;
    }
		
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
