#include "DeviceConfig.h"     // Headerfile Include File
#include "ExternGlobals.h"
#include "math.h"
#include "IQmathLib.h"  // Include header for IQmath library 
//	float beta_1=0;
//	float beta_2=0;
//	float beta_3=0;

#define Liner_ADRC 1
#define None_Liner_ADRC 0//该控制器输出发散，请勿使用



#if None_Liner_ADRC

/**************TD**********/

float delta_TD = 600,//快速跟踪因子，决定跟踪快慢的参数
      h = 0.001;//滤波因子,系统调用步长,采样周期

/**************ESO**********/
float b       = 2,//系统系数
      delta   = 0.007,//delta为fal（e，alpha，delta）函数的线性区间宽度为h的整数倍
      beta_1 = 10,//扩张状态观测器反馈增益1
	  beta_2 = 10,//扩张状态观测器反馈增益2
	  beta_3 = 600;//扩张状态观测器反馈增益3
	  
/**************NLSEF*******/
float alpha_1 = 2,//
      alpha_2 = 2,//
      belta_1 = 1.5,//跟踪输入信号增益
      belta_2 = 1.5;//跟踪微分信号增益

float sgn(float x)
{
	if(x>0)
		return 1;
	if(x<0)
		return -1;
	if(x==0)
		return 0;
}

int FST(int x_1,int x_2,int delta,int h)
{	
	float d=0;
	float d_0=0;
	float y=0;
	float a_0=0;
	float a=0;
	float result=0;
	
	
	d=delta*h;
	d_0=h*d;
	y=x_1+h*x_2;
	a_0=sqrtf(d*d+8*delta*fabsf(y));
	

	
	if(fabsf(y)>d_0)
	{
		a=x_2+0.5*(a_0-d)*sgn(y);
	}
	if(fabsf(y)<=d_0)
	{
		a=x_2+y/h;
	}	
	
	
	
	if(fabsf(a)>d)
	{
		result=-delta*sgn(a);
	}
	
	if(fabsf(a)<=d)
	{
		result=-delta*a/d;
	}
	
	
	return result;
	
};


float fal(float e,float alpha,float delta)
{
	float result=0;
	float fabsf_e=fabsf(e);
	
	if(fabsf_e<=delta)
	{
		result=e*powf(delta,1.0-alpha);
	}
	if(fabsf_e>delta)
	{
		result=powf(fabsf_e,alpha)*sgn(e);
		
	}
	return result;
	
}
	float z_1=0;
	float z_2=0;
	float z_3=0;
	float x_1=0;
	float x_2=0;
	float e=0;
	float u=0;
	
	
	
float ADRC(float v,float y)
{
	float e_1=0;
	float e_2=0;
	float u_1=0;
	x_1=x_1+h*x_2;
	x_2=x_2+h*FST(x_1-v,x_2,delta_TD,h);
	
	e=z_1-y;
	
	z_1=z_1+h*(z_2-beta_1*e);
	z_2=z_2+h*(z_3-beta_2*fal(e,alpha_1,delta)+b*u);
	z_3=z_3-h*beta_3*fal(e,alpha_2,delta);
	
	if(z_1>=30000) z_1=30000;
	if(z_1<=-30000) z_1 = -30000;
	if(z_2>=30000) z_2=30000;
	if(z_2<=-30000) z_2 = -30000;
	if(z_3>=30000) z_3=30000;
	if(z_3<=-30000) z_3 = -30000;

	e_1=x_1-z_1;
	e_2=x_2-z_2;
	
	u_1=beta_1*fal(e_1,alpha_1,delta)+beta_2*fal(e_2,alpha_2,delta);
	
	u=u_1-z_3/b;
	(u<30000)?(u>-30000?(u=u):(u=-30000)):(u=30000);
	return u;
}
#endif
#if Liner_ADRC

#include <stdio.h>


double wc=30; // 控制器带宽
double wo=60; // 观测器带宽
double b0=1; // 系统参数
double ts=0.001; // 采样时间
double adrc_z[2]={0}; // 状态观测值
double dz[2]={0}; // 状态变化率
double out=0; // 控制输出
double out_up=20000; // 输出上限
double out_low=-20000; // 输出下限



// LADRC控制算法
float ADRC(float ref, float fb) 
{
// LESO部分
dz[0] = 2 * wo * (fb - adrc_z[0]) + adrc_z[1] + b0 * out;
dz[1] = wo * wo * (fb - adrc_z[0]);

adrc_z[0] += ts * dz[0];
adrc_z[1] += ts * dz[1];

// 控制器部分
out = (wc * (ref - adrc_z[0]) - adrc_z[1]) / b0;

if (out > out_up)
out = out_up;
if (out < out_low)
out = out_low;

return out;
}



#endif

