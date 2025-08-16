#include "pidcspeed.h"
#include "ExternGlobals.h"
short  SpeedFdbpPost23 = 0;
int tmp2100a = 750 * 32767;
int tmp2100b = -750 * 32767;
short  maxt = 15; //用于最大转矩的限制
short  maxtt = -10; //用于最大转矩的限制
short last_speedpid_ref=0;
extern int Torque_limit;

short  resulttmp = 0;
short tempspeeda1, tempspeedb1;

void FindTable(short speed, short torquecoeff, short* result1, short* result2)//查表，用年是反馈速度 //要考虑 重负载时的工况
{
    if( speed > 0)
        tempspeeda1 = 750;
    else if( speed < 0)
        tempspeeda1 = -750;
    else tempspeeda1 = 0;

    //6.5+665
    //7.5 +750
    tempspeedb1 = ((speed * 15) >> 1) + tempspeeda1;  //速度
    resulttmp =	torquecoeff * 300;  //300==>1  //转矩

    *result1 = tempspeedb1;
    *result2  = *result1 +  resulttmp;
}


void pidspeed_calc(PIDSpeed *v)
{

		if(UartMode.Mode == 2)//转矩模式下速度环输出限幅（限转矩）
		{	
			if(Torque_limit>0)
			{
			v->OutMax=Torque_limit*30;
			v->OutMin=-Torque_limit*30;
			}
			if(Torque_limit<0)
			{
			v->OutMax=-Torque_limit*30;
			v->OutMin=Torque_limit*30;
			}
		}

		v->Err = v->Ref - v->Fdb;
		v->Up = v->Kp * v->Err; 
		v->Up = (v->Up > 13000) ? 13000 : ((v->Up < -13000) ? -13000 : v->Up);



		if(((last_speedpid_ref>0)&&(v->Ref<0))||((last_speedpid_ref<0)&&(v->Ref>0)))
		{
			v->Ui=0;
		}//换向时积分清零
		
		last_speedpid_ref=v->Ref;
		
		if((v->OutPreSat > v->OutMax)||(v->OutPreSat < v->OutMin))
		{}
		else
		{
			v->Ui += v->Ki * v->Err; // 合并常数项:20*3=60
		}//输出饱和时停止积分

       
       if (v->Ref == 0 && v->Fdb == 0) { // 速度为0清零积分
           v->Ui = 0;
       }
		
       v->Ui = (v->Ui > 25000 * 32768) ? 25000 * 32768 : ((v->Ui < -25000 * 32768) ? -25000 * 32768 : v->Ui);

       // 合并输出并限幅
       v->OutPreSat = v->Up + (v->Ui >> 15);
       v->Out = (v->OutPreSat > v->OutMax) ? v->OutMax : ((v->OutPreSat < v->OutMin) ? v->OutMin : v->OutPreSat);
}

