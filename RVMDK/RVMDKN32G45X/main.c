#include "Globals.h"
#include "main.h"
#include "Function.h"

// ϵͳ����������
int main(void);
// ========================================================================
// �������ƣ�main
// �����������
// �����������
// ��    �룺��
// ��    ������
// ����������ϵͳ������
// ========================================================================



int main()
{
    System_Init();
    ReadVar();       //��ȡ���ò���
	  if(SystemVar.SoftwareVersion!=SoftwareVersion)
		{
			SaveAllRsetVar(); //������λ
		    ReadVar();        //��ȡ���ò���
		}
	
    delay_Ms(500);
    Set_DRV8323();   //DRVʹ��
    //modbusͨ��
    eMBInit(MB_RTU, SystemVar.ModbusID, 3, (SystemVar.ModbusBaudrate)*100, MB_PAR_NONE);//0x01;//< �ӻ���ַ
    CANopen_Init();      //CanOpen�����ʼ��
    
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
						Modbus_Task(&modify);  //modbusͨ��
            CANopen_Task();        //CANopenͨ��
            LostCoder();           //���������
            LostPhase();           //ȱ����
        }

        /**************************************100ms**************************************/
        if(timer.Flag100ms)
        {
            timer.Flag100ms = M_DISABLE0;
            TorsionAnalyse();      //���ط���
            SpeedAnalyse();        //���ٷ���
            PowerManage();
            if(SystemError.RuningMode == 2)
            {
                UartMode_Runing();  //modbus�߼�����
							  //TestMode_Runing();
            }

            if(SystemError.RuningMode == 1)
            {
                CiA402Mode_Runing(); //CANopen�߼�����
            }

            SysErrManage();         //ϵͳ������

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
                SaveAllRsetVar();    //������λ
            }

            if(SystemError.SaveAllParaFlag)
            {
                SystemError.SaveAllParaFlag = 0;
                SaveAllVar();    //��������
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

