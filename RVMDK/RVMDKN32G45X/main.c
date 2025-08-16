#include "Globals.h"
#include "main.h"
#include "Function.h"

// 系统主函数声明
int main(void);
// ========================================================================
// 函数名称：main
// 输入参数：无
// 输出参数：无
// 扇    入：无
// 扇    出：无
// 函数描述：系统主函数
// ========================================================================



int main()
{
    System_Init();
    ReadVar();       //读取配置参数
	  if(SystemVar.SoftwareVersion!=SoftwareVersion)
		{
			SaveAllRsetVar(); //参数复位
		    ReadVar();        //读取配置参数
		}
	
    delay_Ms(500);
    Set_DRV8323();   //DRV使能
    //modbus通信
    eMBInit(MB_RTU, SystemVar.ModbusID, 3, (SystemVar.ModbusBaudrate)*100, MB_PAR_NONE);//0x01;//< 从机地址
    CANopen_Init();      //CanOpen外设初始化
    
	  SystemError.SysErr = 0;
    SystemError.RuningMode = 2;
    eMBEnable();
    while(1)
    {
        /**************************************1ms***************************************/
        if(timer.Flag1ms)
        {
            timer.Flag1ms = M_DISABLE0;
            testa1++;

            if(testa1 >= 1000)
            {
                testa1 = 0;
                testa2++;
            }
        }//end timer.Flag1ms

        /**************************************10ms**************************************/
        if(timer.Flag10ms)
        {
            timer.Flag10ms = M_DISABLE0;


            testb1++;

            if(testb1 >= 100)
            {
                testb1 = 0;
                testb2++;
            }
        }//end timer.Flag10ms

        if(timer.Flag25ms)
        {
            timer.Flag25ms = M_DISABLE0;
            testc1++;

            if(testc1 >= 40)
            {
                testc1 = 0;
                testc2++;
            }
        }

        if(timer.Flag50ms)
        {
            timer.Flag50ms = M_DISABLE0;
            testd1++;

            if(testd1 >= 20)
            {
                testd1 = 0;
                testd2++;
            }
						Modbus_Task(&modify);  //modbus通信
            CANopen_Task();        //CANopen通信
            LostCoder();           //编码器检查
            LostPhase();           //缺相检查
        }

        /**************************************100ms**************************************/
        if(timer.Flag100ms)
        {
            timer.Flag100ms = M_DISABLE0;
            TorsionAnalyse();      //过载分析
            SpeedAnalyse();        //超速分析
            PowerManage();
            if(SystemError.RuningMode == 2)
            {
                UartMode_Runing();  //modbus逻辑处理
							  //TestMode_Runing();
            }

            if(SystemError.RuningMode == 1)
            {
                CiA402Mode_Runing(); //CANopen逻辑处理
            }

            SysErrManage();         //系统错误处理

            teste1++;
            if(teste1 >= 10)
            {
                teste1 = 0;
                teste2++;
            }
        }//end timer.Flag100ms

        /**************************************500ms**************************************/
        if(timer.Flag500ms)
        {
            timer.Flag500ms = M_DISABLE0;
             Led();
					
            if(SystemError.SaveAllRsetFlag)
            {
                SystemError.SaveAllRsetFlag = 0;
                SaveAllRsetVar();    //参数复位
            }

            if(SystemError.SaveAllParaFlag)
            {
                SystemError.SaveAllParaFlag = 0;
                SaveAllVar();    //参数保存
            }

            testf1++;
            if(testf1 >= 2)
            {
                testf1 = 0;
                testf2++;
            }
						
					//	iqtest();
					 
        }//end timer.Flag500ms

        /**************************************1s****************************************/
        if(timer.Flag1s)
        {
            timer.Flag1s = M_DISABLE0;
        }//end timer.Flag1s

        if(timer.Flag1m)
        {
            timer.Flag1m = M_DISABLE0;
        } //end timer.Flag1m

        if(timer.Flag1h)
        {
            timer.Flag1h = M_DISABLE0;
        }
    }
}

