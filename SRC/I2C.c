#include "DSP2803x_Device.h"     // Headerfile Include File
#include "DSP2803x_Device.h"     // Headerfile Include File
#include "DSP2803x_Examples.h"
#include "DeviceConfig.h"
#include "ExternGlobals.h"
#define IDLE 0
#define M_WRITE_SART        0x01
#define M_WRITE_ADD_XRDY    0x02
#define M_WRITE_DATA_XRDY1  0x03
#define M_WRITE_DATA_XRDY2  0x04
#define M_WRITE_WRITING     0x05
#define M_WRITE_FINISH      0x06
#define M_READ_START        0x11
#define M_READ_RECEIVE      0x12
#define M_READ_READING      0x13
#define M_READ_FINISH       0x14
#define DELAY2              0x21
#define M_READ_READING2     0x15

Uint16 I2CA_WriteData(I2CMSG *msg);
Uint16 I2CA_ReadData(I2CMSG *msg);
void I2cWriteBit16(EepromWrite16 *p, u16 Addr, u16 Page);
void I2cReadBit16(EepromRead16 *p, u16 Addr, u16 Page);
void EEPromWrite(EEProm16 *v);
void EEPromRead(EEProm16 *v);
int EEPromErrCount = 0;
int I2CSTRTmp = 0;
int I2CState = 0;
int DelayCount = 0;
int data1 = 0;
int data2 = 0;
int WritingCount = 0;

void EEProm(EEProm16 *v);
void EEProm(EEProm16 *v)
{
    if(v->StateFlag == 1) //д
    {
        if(I2CState == IDLE)
        {
            I2CState = M_WRITE_SART;
            v->StateFlag = 2;
        }
    }
    else if(v->StateFlag == 3) //��
    {
        if(I2CState == IDLE)
        {
            I2CState = M_READ_START;
            v->StateFlag = 4;
        }
    }

    EEPromErrCount++;
    switch(I2CState)
    {
        case IDLE :
        {
            EEPromErrCount = 0;
        }
        break;

        case M_WRITE_SART :
        {
            if(I2caRegs.I2CSTR.bit.BB == 0) // ���߿���
            {
                I2caRegs.I2CSAR = 0x50 + v->Page;
                I2caRegs.I2CSTR.bit.BB = 1;
                I2caRegs.I2CCNT = 1 + 2;
                I2caRegs.I2CMDR.all = 0x6E20;  //I2C Mode Register
                I2CState =  M_WRITE_ADD_XRDY;
            }
        }
        break;

        case  M_WRITE_ADD_XRDY: //���ڷ��͵�ַ��
        {
            if(I2caRegs.I2CSTR.bit.XRDY == 1)
            {
                I2CState = M_WRITE_DATA_XRDY1;
                I2caRegs.I2CDXR = (v->Add) << 1;
            }
        }
        break;

        case M_WRITE_DATA_XRDY1 : //��������
        {
            if(I2caRegs.I2CSTR.bit.XRDY == 1)
            {
                I2caRegs.I2CDXR = (v->Dats) & 0xff; // I2C data transmit register             // I2C transmit shift register (not accessible to the CPU)
                I2CState = M_WRITE_DATA_XRDY2;
            }
        }
        break;

        case M_WRITE_DATA_XRDY2 : //��������
        {
            if(I2caRegs.I2CSTR.bit.XRDY == 1)
            {
                I2caRegs.I2CDXR = ((v->Dats) >> 8) & 0xff;
                WritingCount = 0;
                I2CState = M_WRITE_WRITING;
            }
        }
        break;

        case M_WRITE_WRITING :
        {
            WritingCount++;
            if(WritingCount > 10)
            {
                I2CState = M_WRITE_FINISH;
            }
        }
        break;
        case M_WRITE_FINISH :
        {
            if(I2caRegs.I2CSTR.bit.BB == 0) //æ
            {
                I2CState = IDLE;  //
                v->StateFlag = 5;
                EEPromErrCount = 0;
            }
        }
        break;


        /******************************READ******************************************/
        case M_READ_START :
        {
            if(I2caRegs.I2CSTR.bit.BB == 0) // ����æ �ȴ�
            {
                /****************************************************************************************
                15      14    13   12        11 10   9   8  7 6    5   4  3    2 1 0
                 NACKMOD FREE STT  Reserved STP MST TRX XA RM DLB IRS STB FDF BC
                ************************************************************/
                I2caRegs.I2CSTR.bit.BB = 1;
                I2caRegs.I2CSAR = 0x50 + v->Page;
                I2caRegs.I2CDXR = (v->Add) << 1;
                I2caRegs.I2CCNT = 1;
                I2caRegs.I2CMDR.all = 0x2620; //ֻ����ʼ������ֹͣ�ź� //I2C Mode Register    STT==>1   STP==>0; MST>=1 TRX==>1     XA==>0  7bit
                I2CState =  M_READ_RECEIVE;
            }
        }
        break;

        case M_READ_RECEIVE : //���͵�ַ
        {
            if(I2caRegs.I2CSTR.bit.ARDY == 1) //�ȴ��������
            {
                I2caRegs.I2CCNT = 2; //׼��������������
                I2caRegs.I2CMDR.all = 0x2C20; //������ʼ��ֹͣ�źţ��Ա��������  STT==>1   STP==>1; MST>=1 TRX==>0
                I2CState = M_READ_READING;
                DelayCount = 0;
            }
        }
        break;

        case M_READ_READING ://��������
        {
            if(I2caRegs.I2CSTR.bit.RRDY == 1) //SCD
            {
                data1 = I2caRegs.I2CDRR;
                I2CState = M_READ_READING2;
            }
        }
        break;
        case  M_READ_READING2 :
        {
            if(I2caRegs.I2CSTR.bit.RRDY == 1) //SCD
            {
                data2 = I2caRegs.I2CDRR;
                v->Dats =   data1 + (data2 << 8);
                I2CState = M_READ_FINISH;
            }
        }
        break;

        case M_READ_FINISH:
        {
            if(I2caRegs.I2CSTR.bit.BB == 0) //æ
            {
                I2CState = DELAY2;
                DelayCount = 0;
            }
        }
        break;

        case DELAY2 ://����������
        {
            DelayCount++;
            if(DelayCount > 1)
            {
                I2CState = IDLE;
                v->StateFlag = 6;
                EEPromErrCount = 0;
            }
        }
        break;
    }
    if(EEPromErrCount >= 50)
    {
        SysErr = M_SYSERR_EEPROM;
    }
}


















/********************

һ. ��������:
    ����������100K��400K���֣�
    ֧�ֶ��ͨѶ��
    ֧�ֶ�����ģ�飬��ͬһʱ��ֻ������һ�����أ�
    ��������SDA��ʱ��SCL���ɵĴ������ߣ�
    ÿ����·��ģ�鶼��Ψһ�ĵ�ַ��
    ÿ����������ʹ�ö�����Դ

��. ��������ԭ��:
    �������ź�START���ƹ����ߣ���ֹͣ�ź�STOP���ͷ����ߣ�
    ÿ��ͨѶ��START��ʼ����STOP����
    �����ź�START������ŷ���һ����ַ�ֽڣ�����7λΪ���������ĵ�ַ�룬һλΪ��/д����λR/W,R. /WλΪ0��ʾ�������򱻿�����д���ݣ�R/WΪ1��ʾ�������򱻿����������ݣ�
    ������������⵽�յ��ĵ�ַ���Լ��ĵ�ַ��ͬʱ���ڵ�9��ʱ���ڼ䷴��Ӧ���źţ�
    ÿ�������ֽ��ڴ���ʱ���Ǹ�λ(MSB)��ǰ��

дͨѶ����:
    1. �����ڼ�⵽���߿��е�״���£�������һ��START�ź��ƹ����ߣ�
    2. ���ͻ����ַ�ֽ?����7λ��ַ���һλR/W)��
    3. ������������⵽���ط��͵ĵ�ַ���Լ��ĵ�ַ��ͬʱ����һ��Ӧ���ź�(ACK)��
    4. �����յ�ACK��ʼ���͵�һ�������ֽڣ�
    5. �������յ������ֽں���һ��ACK��ʾ�����������ݣ�����NACK��ʾ�������ݽ�����
    6. ���ط�����ȫ�����ݺ󣬷���һ��ֹͣλSTOP����������ͨѶ�����ͷ����ߣ�

��ͨѶ����:
    1. �����ڼ�⵽���߿��е�״���£����ȷ���һ��START�ź��ƹ����ߣ�
    2. ����һ����ַ�ֽ�(����7λ��ַ���һλR/W)��
    3. ������������⵽���ط��͵ĵ�ַ���Լ��ĵ�ַ��ͬʱ����һ��Ӧ���ź�(ACK)��
    4. �����յ�ACK���ͷ��������ߣ���ʼ���յ�һ�������ֽڣ�
    5. �����յ����ݺ���ACK��ʾ�����������ݣ�����NACK��ʾ�������ݽ�����
    6. ���ط�����ȫ�����ݺ󣬷���һ��ֹͣλSTOP����������ͨѶ�����ͷ����ߣ�

��. �����ź�ʱ�����
    1. ���߿���״̬
    SDA��SCL�����ź��߶����ڸߵ�ƽ�������������е��������ͷ����ߣ������ź��߸��Ե���������ѵ�ƽ���ߣ�
    2. �����ź�START
    ʱ���ź�SCL���ָߵ�ƽ�������ź�SDA�ĵ�ƽ������(��������)�������źű����������źţ������ڽ������ź�ǰ���ޱ�֤���ߴ��ڿ���״̬��
    3. ֹͣ�ź�STOP
    ʱ���ź�SCL���ָߵ�ƽ�������߱��ͷţ�ʹ��SDA���ظߵ�ƽ(��������)��ֹͣ�ź�Ҳ�����������źš�
    4. ���ݴ���
    SCL�߳��ָߵ�ƽ�ڼ䣬SDA���ϵĵ�ƽ���뱣���ȶ����͵�ƽ��ʾ0(��ʱ���ߵ�ѹΪ�ص�ѹ)���ߵ�ƽ��ʾ1(��ʱ�ĵ�ѹ��Ԫ������VDD����)��ֻ����SCL��Ϊ�͵�ƽ�ڼ䣬SDA�ϵĵ�ƽ����仯��
    5. Ӧ���ź�ACK
    I2C���ߵ����ݶ������ֽ�(8λ)�ķ�ʽ���͵ģ���������ÿ����һ���ֽ�֮����ʱ�ӵĵ�9�������ڼ��ͷ��������ߣ��ɽ���������һ��ACK(���������ߵĵ�ƽ����)����ʾ���ݳɹ����ա�
    6. ��Ӧ���ź�NACK
    ��ʱ�ӵĵ�9�������ڼ䷢�����ͷ��������ߣ��������������������߱�ʾһ��NACK��NACK��������;:
    a. һ���ʾ������δ�ɹ����������ֽڣ�
    b. ����������������ʱ�����յ����һ���ֽں�Ӧ����һ��NACK�źţ���֪ͨ���ط������������ݷ��ͣ����ͷ����ߣ��Ա����ؽ���������һ��ֹͣ�ź�STOP��
    7. ���������յ�һ�������������ֽں��п�����Ҫ���һЩ�����������紦���ڲ��жϷ���ȣ������޷����̽�����һ���ֽڣ���ʱ�����������Խ�SCL�����ɵ͵�ƽ���Ӷ�ʹ�������ڵȴ�״̬��ֱ����������׼���ý�����һ���ֽ�ʱ�����ͷ�SCL��ʹ֮Ϊ�ߵ�ƽ���Ӷ�ʹ���ݴ��Ϳ��Լ������С�
    8. I2C���߽������ݴ���ʱ��ʱ���ź�Ϊ�ߵ�ƽ�ڼ䣬�������ϵ����ݱ��뱣���ȶ���ֻ����ʱ�����ϵ��ź�Ϊ�͵�ƽ�ڼ䣬�������ϵĸߵ�ƽ��͵�ƽ״̬������仯
��. ѰַԼ��
    ��ַ�ķ��䷽��������:
    1. ��CPU��������������ַ�������ʼ��ʱ���壬�������������������г�ͻ��
    2. ����CPU�ķ������������ɳ����������ڲ��̻������ɸı䡣

    ��7λΪ��ַ�룬���Ϊ������:
    1. ��4λ���ڹ̶���ַ���ɸı䣬�ɳ��ҹ̻���ͳһ��ַ��
    2. ����λΪ�����趨��ַ���������ⲿ�������趨(�������������������趨)��


********************/
