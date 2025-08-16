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
//1倍额定负载下的正向速度-Uq表
short Uqs_Speed_1_0_Load_Table[31] =
{
    3500, 3500, 4320, 5020, 5750, 6540, 7310, 7910, 8710, 9320, 10260, 10980, 11740, 12420, 13410, 13740, //0-1500rpm
    14630, 15420, 16210, 16520, 17040, 18310, 19490, 19380, 20710, 21150, 22410, 22130, 23250, 23920, 25010	 //1600-3000rpm
};
//1倍额定负载下的正向速度-偏移表
short Offset_Speed_1_0_Load_Table[31] =
{
    150, 150, 150, 250, 300, 300, 350, 400, 400, 450, 450, 450, 450, 500, 500, 550,//0-1500rpm
    550, 550, 550, 600, 600, 600, 600, 600, 600, 650, 650, 700, 700, 700, 700    //1600-3000rpm
};
//1.5倍额定负载下的正向速度-Uq表
short Uqs_Speed_1_5_Load_Table[31] =
{
    4930, 4930, 5630, 6460, 7220, 8010, 8600, 9450, 10210, 10950, 11880, 12610, 13310, 14280, 15350, 15700, //0-1500rpm
    16720, 17610, 18420, 18670, 19650, 20630, 21050, 21330, 21910, 23210, 24110, 25330, 25610, 27340, 27350 //1600-3000rpm
};
//1.5倍额定负载下的正向速度-偏移表
short Offset_Speed_1_5_Load_Table[31] =
{
    150, 150, 200, 400, 400, 400, 450, 500, 500, 550, 550, 600, 600, 600, 600, 650, //0-1500rpm
    650, 650, 650, 700, 700, 700, 800, 800, 800, 800, 800, 800, 850, 850, 850     //1600-3000rpm
};
*/

//拟合得到的Uqs-速度公式，如第二项{626,1110}表示100rpm的Uqs=626x+1110，其中x的单位是1/4额定转矩
short Uqs_Speed_Equation_Table[31][2] =
{
    // 0 到 3000rpm，每一项间隔100rpm
    {626, 1110}, {626, 1110}, {617, 1876}, {659, 2460}, {667, 3147}, {666, 3934}, {687, 4540}, {715, 5109}, {725, 5846}, {746, 6496}, {803, 7029},
    {818, 7641}, {858, 8219}, {829, 9224}, {935, 9713}, {877, 10457}, {1000, 10780}, {972, 11557}, {970, 12440}, {921, 13057}, {930, 13811},
    {1001, 14607}, {965, 15253}, {831, 16344}, {1105, 16036}, {966, 17524}, {1017, 17897}, {1287, 17242}, {1175, 18616}, {1296, 19161}, {959, 21605}
};
//拟合得到的Offest-速度公式，如第三项{16,114}表示200rpm的Offest=16x+114，其中x的单位是1/4额定转矩
short Offest_Speed_Equation_Table[31][2] =
{
    {0, 150}, {0, 150}, {16, 114}, {48, 57}, {50, 100}, {50, 100}, {50, 150}, {59, 150}, {59, 150}, {66, 157}, {59, 200},
    {66, 200}, {66, 200}, {66, 207}, {64, 222}, {75, 207}, {66, 257}, {64, 271}, {66, 271}, {75, 257}, {75, 257},
    {75, 257}, {79, 293}, {79, 293}, {79, 293}, {80, 300}, {87, 303}, {81, 840}, {89, 323}, {89, 323}, {89, 323}
};

void FindTable(short speed, short torquecoeff, short* result1, short* result2)
{
    short index = speed / 100; // 计算速度所在的表格区间索引 (100 rpm 为一个区间)
    short frac = speed - index * 100; // 获取十位和个位数
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

//参数设置
//uint16_t KpGainCoeff = 30; //切换PID后Kp改变的系数
//uint16_t KiGainCoeff = 30; //切换PID后Ki改变的系数
//uint16_t ThresholdCoeff = 115; //门槛切换系数，除以最大值128可以得到百分比
//uint16_t TorqueCoeff = 6;       //用于改变速度的输出扭矩值（绝对值），单位为1/4额定
//uint16_t TorquePIDLimitCoeff = 4;  //用于PID阶段限幅的输出扭矩值，单位为1/4额定
//uint16_t TorqueDECLimitCoeff = 0;  //用于同向减速时限幅的输出扭矩值，单位为1/4额定

extern short SpeedFdbpFilter1;


void MixPID_calc(PIDC * v1, PID * v2, short vel_ref)
{
    static short LastSpeed = 0;     //上一次的目标速度
    static short SpeedThreshold = 0;//速度切换门槛
    static short SpeedDir = 0;      //目标速度方向
    static short SpeedChangeDir = 0;//速度改变方向
    static short Run_PID_Mode = 0;  //默认为0,为1则直接给定,为2则运行PID
    static short Run_Mode = 0;      //为0则复位参数，为1则加速，为2则减速
    static short v1_Value, v2_Value;//存放查表输出
    static short v1_ValueLimit, v2_ValueLimit;
    short vel_fb = SpeedFdbpFilter1;//MotorControler.SpeedFdbp;//SpeedFdbpFilter1;  //反馈速度

    if(LastSpeed != vel_ref) //目标速度切换
    {
        SpeedChangeDir = LastSpeed < vel_ref ? 1 : -1;  //速度变化方向
        SpeedDir = vel_ref < 0 ? (short) -1 : (short)1;  //目标速度方向

        //计算切换门槛，如果是正反向切换，只使用（目标速度*门槛系数）作为门槛，否则使用前后速度共同计算
        if(((vel_ref > 0) && (LastSpeed < 0)) || ((vel_ref < 0) && (LastSpeed > 0)))
        {
            SpeedThreshold = (short)(((int)SystemError.ThresholdCoeff * vel_ref) >> 7); //ThresholdCoeff为128时对应系数为1
        }
        else
        {
            SpeedThreshold = (short)((((int)SystemError.ThresholdCoeff * (vel_ref - LastSpeed)) >> 7) + LastSpeed);
        }

        Run_PID_Mode = 1;
        Run_Mode = 0;
        LastSpeed = vel_ref;

        //查表
        short v1_ValueMax, v2_ValueMax;
        FindTable(SpeedDir * vel_ref, SystemError.TorquePIDLimitCoeff, &v1_ValueMax, &v2_ValueMax);

        //对PID限幅，限幅值只与目标速度绝对值和限幅系数有关
        v1->UiMax = ((int)(v1_ValueMax) << 15);
        v1->UiMin = -v1->UiMax;
        v1->OutMax = v1_ValueMax;
        v1->OutMin = -v1_ValueMax;
        v2->UiMax = ((int)(v2_ValueMax) << 15);
        v2->UiMin = -v2->UiMax;
        v2->OutMax = v2_ValueMax;
        v2->OutMin = -v2_ValueMax;

        //每次速度改变都重新复位PI参数
        v1->Kp = pidpv_Kp / 10;
        v1->Ki = pidpv_Ki / 10;
        v2->Kp = 100;
        v2->Ki = 10;
    }

    // 确定运行模式
    if(Run_PID_Mode == 1)
    {
        if(vel_fb == 0) //反馈速度为0
        {
            if(vel_ref == 0)        //复位
            {
                Run_Mode = 0;
                Run_PID_Mode = 0;
            }
            else                    //阶跃加速
            {
                Run_Mode = 1;
            }
        }
        else if(Run_Mode == 4) //速度正反改变时，可能没有等于0的时刻，而是直接从正到负（或相反），此时需要改变模式为加速
        {
            if(((vel_fb > 0) && (vel_ref > 0)) || ((vel_fb < 0) && (vel_ref < 0)))
            {
                Run_Mode = 1;
            }
        }
        else           //反馈速度不为0
        {
            if(Run_Mode == 0)
            {
                if(((vel_fb > 0) && (vel_ref > vel_fb)) || ((vel_fb < 0) && (vel_ref < vel_fb))) //同向加速
                {
                    Run_Mode = 1;
                }
                else if(((vel_ref > 0) && (vel_ref < vel_fb)) || ((vel_ref < 0) && (vel_ref > vel_fb)))  //同向减速
                {
                    Run_Mode = 2;
                    FindTable(SpeedDir * vel_ref, SystemError.TorqueDECLimitCoeff, &v1_ValueLimit, &v2_ValueLimit); //查表得到同向减速的限幅值
                }
                else if(vel_ref == 0)  //减速为0
                {
                    Run_Mode = 3;
                }
                else if(((vel_ref > 0) && (vel_fb < 0)) || ((vel_ref < 0) && (vel_fb > 0))) //先减速再反向加速
                {
                    Run_Mode = 4;
                }
            }
        }
    }

    //确定反馈速度方向，如果反馈速度为0，则使用目标速度的方向
    short fb_dir = SpeedDir;
    if(vel_fb != 0)
    {
        fb_dir = vel_fb < 0 ? (short) -1 : (short)1;
    }

    // 根据运行模式，查表输出值
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
        if(Run_Mode == 1)        //加速
        {
            torqueCoeff = (short)(SystemError.TorqueCoeff);
        }
        else if(Run_Mode >= 2)  //减速
        {
            torqueCoeff = (short)(-SystemError.TorqueCoeff);
        }

        //反馈速度查表
        FindTable(fb_dir * vel_fb, torqueCoeff, &v1_Value, &v2_Value);
        v1_Value = fb_dir * v1_Value;
        v2_Value = fb_dir * v2_Value;

        if(Run_Mode == 2)   //同向减速需要限幅输出
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

    // 对速度进行判定，直接给定输出或者切换PID
    if(Run_PID_Mode == 1)      //为1则直接给定
    {
        //正向变化且速度小于阈值 或 反向变化且速度大于阈值
        if(((SpeedChangeDir == 1) && (vel_fb <= SpeedThreshold)) || ((SpeedChangeDir == -1) && (vel_fb >= SpeedThreshold)))
        {
            v1->Out = v1_Value;
            v2->Out = v2_Value;
        }
        else   //切换
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

    // 运行PID
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

        //输出扭矩限幅
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
        UqsPid.calc(&UqsPid);  //用于电流限幅
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
