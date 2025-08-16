#include "DeviceConfig.h"     // Headerfile Include File
#include "ExternGlobals.h"
#include "math.h"
#include "IQmathLib.h"  // Include header for IQmath library 

float Rotor_paramater_B=0.001;
float Rotor_paramater_Magnet_flux=0.0119737;
float Rotor_paramater_Inductance=0.00046;
float Rotor_paramater_J=0.000058;
float Rotor_paramater_Resistance=0.19;
float Rotor_paramater_Kt=0.2;


float L1=0;
float L2=0;
float L3=0;	

float Wn=100;
//¦Ë1=¦Ë2=¦Ë3Ê±










int Theta_Estimate=0;
int Omega_Estimate=0;
int Torque_Load_Estimate=0;
int Theta_Estimate_Dot=0;
int Omega_Estimate_Dot=0;
int Torque_Load_Estimate_Dot=0;


int Luenberger_Observer(int Theta,int Iq)
{
	
	L1=2*Wn-Rotor_paramater_B/Rotor_paramater_J;
	L2=3*Wn*Wn-2*Wn*Rotor_paramater_B/Rotor_paramater_J+Rotor_paramater_B*Rotor_paramater_B/(Rotor_paramater_J*Rotor_paramater_J);
	L3=-powf(Wn,3)*Rotor_paramater_J;	
	Theta_Estimate_Dot = -L1*Theta_Estimate+Omega_Estimate+L1*Theta;
	Omega_Estimate_Dot = -L2*Theta_Estimate-Rotor_paramater_B*Omega_Estimate/Rotor_paramater_J-Torque_Load_Estimate/Rotor_paramater_J+Rotor_paramater_Kt*Iq/Rotor_paramater_J+L2*Theta;
	Torque_Load_Estimate_Dot = -L3*Theta_Estimate+L3*Theta;
	Theta_Estimate = Theta_Estimate + Theta_Estimate_Dot*0.001;
	Omega_Estimate = Omega_Estimate + Omega_Estimate_Dot*0.001;
	Torque_Load_Estimate = Torque_Load_Estimate + Torque_Load_Estimate_Dot*0.001;
	return Omega_Estimate;
}


//int Noneliner_Observer()
//{
//	
//	
//	
//	
//}


	