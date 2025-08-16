#ifndef __MAIN_H__
#define __MAIN_H__

#include "n32g45x.h"

short testa1 = 0;
short testa2 = 0;
short testb1 = 0;
short testb2 = 0;
short testc1 = 0;
short testc2 = 0;
short testd1 = 0;
short testd2 = 0;
short teste1 = 0;
short teste2 = 0;

short testf1 = 0;
short testf2 = 0;

//*******************º¯Êý***********************//
extern void System_Init(void);
//extern void ReadVar(void);
//extern void delay_Ms(uint16_t nms);
//extern void Set_DRV8323(void);
extern void CANopen_Init(void);
extern void CANopen_Task(void);
//extern void LostCoder(void); 

//extern void LostPhase(void); 


//extern void TorsionAnalyse(void); 
//extern void SpeedAnalyse(void);   
//extern void PowerManage(void);

////extern void UartMode_Runing(void);
//extern void SysErrManage(void); 
//extern void CiA402Mode_Runing(void);
extern void Led(void);
//extern void SaveAllRsetVar(void);
//extern void SaveAllVar(void);


#endif
