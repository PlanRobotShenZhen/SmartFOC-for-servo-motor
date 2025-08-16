#ifndef __SpeedPlant_H
#define __SpeedPlant_H
#include "IQmathLib.h"
#ifdef __cplusplus
extern "C"
{
#endif

//当前数据结构体
typedef struct
{
    int position;
    int vel;
    int accel;
} Current_value;
//数据存放结构体
typedef struct
{
    int t1;
    int t2;
    int t3;
    int t4;
    int t5;
    int t6;
    int t7;
    int v1;
    int v2;
    int v3;
    int v4;
    int v5;
    int v6;
    int s1;
    int s2;
    int s3;
    int s4;
    int s5;
    int s6;
    int s7;
    int forward_flag;
    int vel_flag;
} SpeedPlant;
//参数设置结构体
typedef struct
{
    int vel_init;
    int vel_tar;
    int accel_max;
    int decel_max;
    int a_accel;
    int a_decel;
    int start_position;
    int end_position;
    SpeedPlant* sp;
} Set_SP_Para;
//多段曲线参数设置结构体
typedef struct
{
    float vel_tar;
    float vel_end;
    float end_position;
} Set_SP_Para_multi;
//错误类型枚举
typedef enum 
{
    none_err = 0,
    accel_max_neg_err = 1,
    decel_max_neg_err = 2,
    a_accel_neg_err = 3,
    a_decel_neg_err = 4,
    vel_tar_zero_err = 5,
    distance_too_small_err = 6
}SP_Error;
//错误编号，当设置多段曲线的参数出错时，会被赋值为出错的那一段编号
//根据错误类型和编号可以知道具体段数的具体错误
extern int multi_err_num;

//设置参数，规划S型曲线
SP_Error Set_SpeedPlant_Para(Set_SP_Para* ptr);
//获取t时刻对应的加速度、速度和位置
Current_value SpeedPlant_positionControl(Set_SP_Para* ptr, int t);
//获取单段曲线总时间
int get_total_time(Set_SP_Para* ptr);
/*   单段使用
    SpeedPlant SP;
    Set_SP_Para sp1={.vel_init = 5,...,.sp=&SP};
    SP_Error error=Set_SpeedPlant_Para(&sp1);
    if(error==none_err){
        for(float t=0;t<get_total_time(&sp1);t=t+0.01f){
            Current_value value=SpeedPlant_positionControl(&sp1,t);
            printf("%f\n",value.vel);
        }
    }
    else{printf("error\n");}
*/

//设置参数，规划多段S型曲线
//注；参数ptr指向的结构体的目标速度和目标位置无效，而是由tar_ptr指向的数据来设置
//SP_Error Set_SpeedPlant_Para_multi(Set_SP_Para *ptr, Set_SP_Para_multi *tar_ptr, int n);
////获取多段曲线t时刻对应的加速度、速度和位置
//Current_value SpeedPlant_positionControl_multi(float t);
////获取多段曲线总时间
//float get_total_time_multi(void);
/*   多段使用
    struct Set_SP_Para_multi sp_arr[num]={
        {   .end_position=100,
            .vel_tar=10,
            .vel_end=3
        },
        ...
    }
    struct Set_SP_Para sp1={.vel_init = 5,....};
    SP_Error error=Set_SpeedPlant_Para_multi(&sp1,sp_arr,num);
    if(error==none_err){
        for(float t=0;t<get_total_time_multi();t=t+0.01f){
            struct Current_value value=SpeedPlant_positionControl_multi(t);
            printf("%f\n",value.vel);
        }
    }
    else{printf("error\n");}
*/

#endif
#ifdef __cplusplus
}
#endif

