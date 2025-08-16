#include "pidcspeed.h"
#include "ExternGlobals.h"
short  SpeedFdbpPost23 = 0;
int tmp2100a = 750 * 32767;
int tmp2100b = -750 * 32767;
short  maxt = 15; //�������ת�ص�����
short  maxtt = -10; //�������ת�ص�����
short last_speedpid_ref=0;
extern int Torque_limit;

short  resulttmp = 0;
short tempspeeda1, tempspeedb1;

void FindTable(short speed, short torquecoeff, short* result1, short* result2)//��������Ƿ����ٶ� //Ҫ���� �ظ���ʱ�Ĺ���
{
    if( speed > 0)
        tempspeeda1 = 750;
    else if( speed < 0)
        tempspeeda1 = -750;
    else tempspeeda1 = 0;

    //6.5+665
    //7.5 +750
    tempspeedb1 = ((speed * 15) >> 1) + tempspeeda1;  //�ٶ�
    resulttmp =	torquecoeff * 300;  //300==>1  //ת��

    *result1 = tempspeedb1;
    *result2  = *result1 +  resulttmp;
}


void pidspeed_calc(PIDSpeed *v)
{

		if(UartMode.Mode == 2)//ת��ģʽ���ٶȻ�����޷�����ת�أ�
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
		}//����ʱ��������
		
		last_speedpid_ref=v->Ref;
		
		if((v->OutPreSat > v->OutMax)||(v->OutPreSat < v->OutMin))
		{}
		else
		{
			v->Ui += v->Ki * v->Err; // �ϲ�������:20*3=60
		}//�������ʱֹͣ����

       
       if (v->Ref == 0 && v->Fdb == 0) { // �ٶ�Ϊ0�������
           v->Ui = 0;
       }
		
       v->Ui = (v->Ui > 25000 * 32768) ? 25000 * 32768 : ((v->Ui < -25000 * 32768) ? -25000 * 32768 : v->Ui);

       // �ϲ�������޷�
       v->OutPreSat = v->Up + (v->Ui >> 15);
       v->Out = (v->OutPreSat > v->OutMax) ? v->OutMax : ((v->OutPreSat < v->OutMin) ? v->OutMin : v->OutPreSat);
}

