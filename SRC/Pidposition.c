#include "Pidposition.h"
#include "ExternGlobals.h"

short v1_ValueMax12, v2_ValueMax12;
short v1_ValueMax13, v2_ValueMax13;
int atest12345, atest12346, atest12347, atest12348, atest12349, atest1234a;
short  maxt2 = 15; //用于最大转矩的限制
short  maxtt2 = -15; //用于最大转矩的限制

short  tempspeeda = 0;
short  tempspeedb = 0;
short  tempspeedb2 = 0;
int  errtmp2 = 0;
extern void FindTable(short speed, short torquecoeff, short* result1, short* result2);

void pidposition_calc(PIDpos *v)
{
    v->Err = v->Ref - v->Fdb;
    v->Up = ((int)v->Kp * ((int)v->Err) >> 4);

    if(v->Up > 2500 ) //防止抖动时，PD 产生极大电压，造成MOS 损坏。
    {
        v->Up = 2500 ;
    }
    else if(v->Up < -2500 )
    {
        v->Up = -2500;
    }

    v->UiMax = 3000 * 32768;
    v->UiMin = -3000 * 32768;

//    if(v->Err > 0) errtmp2 = v->Err;
//    else if(v->Err < 0) errtmp2 = 0 - v->Err;
//    else  errtmp2 = 1;

//    if(errtmp2 > 2500)
//        v->Ui += (v->Ki * (v->Err >> 6) * (errtmp2 >> 7)) ;

//    else if((errtmp2 < 2500) && (errtmp2 >= 1000))

//        v->Ui += (v->Ki * (v->Err >> 4) * (errtmp2 >> 5)) ;

//    else if((errtmp2 < 1000) && (errtmp2 >= 1))

//        v->Ui += (v->Ki * (v->Err>>2) * (errtmp2>>3)) ;


//    if(errtmp2 > 16383)
//		{
//			    v->Ui += (v->Ki * (v->Err /16) * (errtmp2 >>6)) ;
//		}
//    else if((errtmp2 < 16383) && (errtmp2 >= 2048))
//		{
//        v->Ui += (v->Ki * (v->Err /8) * (errtmp2 >>4)) ;
//		}

//    else if((errtmp2 < 2048) && (errtmp2 >= 256))
//    {
//        v->Ui += (v->Ki * (v->Err /4) * (errtmp2 >> 2)) ;
//		}

//    else if((errtmp2 < 256) && (errtmp2 >= 1))   //2*= 128
//		{
//        v->Ui += (v->Ki * (v->Err) * (errtmp2>>1)) ;
//		}


//	    if(errtmp2 > 16383)
//		{
//			    v->Ui += (v->Ki * (v->Err /8) * (errtmp2 >>3)) ;
//		}
//    else if((errtmp2 < 16383) && (errtmp2 >= 2048))
//		{
//        v->Ui += (v->Ki * (v->Err /4) * (errtmp2 >>2)) ;
//		}

//    else if((errtmp2 < 2048) && (errtmp2 >= 256))
//    {
//        v->Ui += (v->Ki * (v->Err /2) * (errtmp2 >> 1)) ;
//		}

//    else if((errtmp2 < 256) && (errtmp2 >= 1))   //2*= 128
//		{
//        v->Ui=0;// += (v->Ki * (v->Err) * (errtmp2)) ;
//		}



    v->Ui =  v->Ui + (v->Ki * ((v->Err) >> 4));

    if(v->Ui > v->UiMax)
    {
        v->Ui = v->UiMax;
    }

    if(v->Ui < v->UiMin)
    {
        v->Ui = v->UiMin;
    }

    if( v->Speedref > 0)
        tempspeeda = 750;

    else if( v->Speedref < 0)
        tempspeeda = -750;

    else tempspeeda = 0;

    //6.5+665
    //7.5 +750
    tempspeedb = ((v->Speedref * 15) >> 1) + tempspeeda;
    tempspeedb2 = ((MotorControler.SpeedFdbpFilter1 * 15) >> 1) + tempspeeda;

    v->OutPreSat = v->Up + ((v->Ui) >> 15) + tempspeedb;

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

    if((v->Ref > 0 ) && (v->Fdb >= 0)) //同向正 加减速
    {
        FindTable( MotorControler.SpeedFdbpFilter1, maxt2, &v1_ValueMax12, &v2_ValueMax12); //以速度  反馈的方式  进行查表
        FindTable( MotorControler.SpeedFdbpFilter1, maxtt2, &v1_ValueMax13, &v2_ValueMax13); //以速度  反馈的方式  进行查表

        if((v->Out > v2_ValueMax12) ) //加速
        {
            v->Out = v2_ValueMax12;
            v->Ui = (((int)(v2_ValueMax12 - tempspeedb2)) << 15);
            atest12345++;
        }

        if(v->Out < v2_ValueMax13)  //减速
        {
            v->Out = v2_ValueMax13;
            v->Ui = (((int)(v2_ValueMax12 - tempspeedb2)) << 15);
            atest12346++;
        }
    }
    else if((v->Ref < 0 ) && (v->Fdb <= 0)) //同向负 加减速
    {
        FindTable( MotorControler.SpeedFdbpFilter1, 0 - maxt2, &v1_ValueMax12, &v2_ValueMax12); //以速度  反馈的方式  进行查表
        FindTable( MotorControler.SpeedFdbpFilter1, 0 - maxtt2, &v1_ValueMax13, &v2_ValueMax13); //以速度  反馈的方式  进行查表

        if(v->Out <  v2_ValueMax12)
        {
            v->Out =  v2_ValueMax12;
            v->Ui = (((int)(v2_ValueMax12 - tempspeedb2)) << 15);
            atest12347++;
        }

        if(v->Out >  v2_ValueMax13)
        {
            v->Out =  v2_ValueMax13;
            v->Ui = (((int)(v2_ValueMax12 - tempspeedb2)) << 15);
            atest12348++;
        }
    }
    else if((v->Ref < 0 ) && (v->Fdb > 0)) //速度方向切换
    {
        FindTable( MotorControler.SpeedFdbpFilter1, maxt2, &v1_ValueMax12, &v2_ValueMax12); //以速度  反馈的方式  进行查表
        FindTable( MotorControler.SpeedFdbpFilter1, maxtt2, &v1_ValueMax13, &v2_ValueMax13); //以速度  反馈的方式  进行查表

        if(v->Out < v2_ValueMax13)  //减速
        {
            v->Out = v2_ValueMax13;
            v->Ui = (((int)(v2_ValueMax13 - tempspeedb2)) << 15);
            atest12346++;
        }
    }
    else if((v->Ref > 0 ) && (v->Fdb < 0)) //速度方向切换
    {
        FindTable( MotorControler.SpeedFdbpFilter1, 0 - maxt2, &v1_ValueMax12, &v2_ValueMax12); //以速度  反馈的方式  进行查表
        FindTable( MotorControler.SpeedFdbpFilter1, 0 - maxtt2, &v1_ValueMax13, &v2_ValueMax13); //以速度  反馈的方式  进行查表

        if(v->Out >  v2_ValueMax13)
        {
            v->Out =  v2_ValueMax13;
            v->Ui = (((int)(v2_ValueMax13 - tempspeedb2)) << 15);
            atest12348++;
        }
    }
    else if((v->Ref == 0 ) && (v->Fdb > 0)) //
    {
        FindTable( MotorControler.SpeedFdbpFilter1, maxt2, &v1_ValueMax12, &v2_ValueMax12); //以速度  反馈的方式  进行查表
        FindTable( MotorControler.SpeedFdbpFilter1, maxtt2, &v1_ValueMax13, &v2_ValueMax13); //以速度  反馈的方式  进行查表

        if((v->Out < v2_ValueMax13 ))
        {
            v->Out = v2_ValueMax13;
            v->Ui = (((int)(v2_ValueMax13 - tempspeedb2)) << 15);
            atest12349++;
        }
    }
    else if((v->Ref == 0 ) && (v->Fdb < 0)) //
    {
        FindTable( MotorControler.SpeedFdbpFilter1, 0 - maxt2, &v1_ValueMax12, &v2_ValueMax12); //以速度  反馈的方式  进行查表
        FindTable( MotorControler.SpeedFdbpFilter1, 0 - maxtt2, &v1_ValueMax13, &v2_ValueMax13); //以速度  反馈的方式  进行查表

        if(((v->Out > v2_ValueMax13)))
        {
            v->Out = v2_ValueMax13 ;
            v->Ui = (((int)(v2_ValueMax13 - tempspeedb2)) << 15);
            atest1234a++;
        }
    }

    if( v->Speedref != 0)
        v->SumIqs = v->SumIqs + ((long long)v->Iqs);
}
