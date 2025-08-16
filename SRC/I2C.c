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
    if(v->StateFlag == 1) //写
    {
        if(I2CState == IDLE)
        {
            I2CState = M_WRITE_SART;
            v->StateFlag = 2;
        }
    }
    else if(v->StateFlag == 3) //读
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
            if(I2caRegs.I2CSTR.bit.BB == 0) // 总线空闲
            {
                I2caRegs.I2CSAR = 0x50 + v->Page;
                I2caRegs.I2CSTR.bit.BB = 1;
                I2caRegs.I2CCNT = 1 + 2;
                I2caRegs.I2CMDR.all = 0x6E20;  //I2C Mode Register
                I2CState =  M_WRITE_ADD_XRDY;
            }
        }
        break;

        case  M_WRITE_ADD_XRDY: //正在发送地址中
        {
            if(I2caRegs.I2CSTR.bit.XRDY == 1)
            {
                I2CState = M_WRITE_DATA_XRDY1;
                I2caRegs.I2CDXR = (v->Add) << 1;
            }
        }
        break;

        case M_WRITE_DATA_XRDY1 : //发送数据
        {
            if(I2caRegs.I2CSTR.bit.XRDY == 1)
            {
                I2caRegs.I2CDXR = (v->Dats) & 0xff; // I2C data transmit register             // I2C transmit shift register (not accessible to the CPU)
                I2CState = M_WRITE_DATA_XRDY2;
            }
        }
        break;

        case M_WRITE_DATA_XRDY2 : //发送数据
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
            if(I2caRegs.I2CSTR.bit.BB == 0) //忙
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
            if(I2caRegs.I2CSTR.bit.BB == 0) // 总线忙 等待
            {
                /****************************************************************************************
                15      14    13   12        11 10   9   8  7 6    5   4  3    2 1 0
                 NACKMOD FREE STT  Reserved STP MST TRX XA RM DLB IRS STB FDF BC
                ************************************************************/
                I2caRegs.I2CSTR.bit.BB = 1;
                I2caRegs.I2CSAR = 0x50 + v->Page;
                I2caRegs.I2CDXR = (v->Add) << 1;
                I2caRegs.I2CCNT = 1;
                I2caRegs.I2CMDR.all = 0x2620; //只发起始，不发停止信号 //I2C Mode Register    STT==>1   STP==>0; MST>=1 TRX==>1     XA==>0  7bit
                I2CState =  M_READ_RECEIVE;
            }
        }
        break;

        case M_READ_RECEIVE : //发送地址
        {
            if(I2caRegs.I2CSTR.bit.ARDY == 1) //等待发送完成
            {
                I2caRegs.I2CCNT = 2; //准备接收两个数据
                I2caRegs.I2CMDR.all = 0x2C20; //发送起始，停止信号，以便读起数据  STT==>1   STP==>1; MST>=1 TRX==>0
                I2CState = M_READ_READING;
                DelayCount = 0;
            }
        }
        break;

        case M_READ_READING ://读起数据
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
            if(I2caRegs.I2CSTR.bit.BB == 0) //忙
            {
                I2CState = DELAY2;
                DelayCount = 0;
            }
        }
        break;

        case DELAY2 ://启动读数据
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

一. 技术性能:
    工作速率有100K和400K两种；
    支持多机通讯；
    支持多主控模块，但同一时刻只允许有一个主控；
    由数据线SDA和时钟SCL构成的串行总线；
    每个电路和模块都有唯一的地址；
    每个器件可以使用独立电源

二. 基本工作原理:
    以启动信号START来掌管总线，以停止信号STOP来释放总线；
    每次通讯以START开始，以STOP束；
    启动信号START后紧接着发送一个地址字节，其中7位为被控器件的地址码，一位为读/写控制位R/W,R. /W位为0表示由主控向被控器件写数据，R/W为1表示由主控向被控器件读数据；
    当被控器件检测到收到的地址与自己的地址相同时，在第9个时钟期间反馈应答信号；
    每个数据字节在传送时都是高位(MSB)在前；

写通讯过程:
    1. 主控在检测到总线空闲的状况下，首先送一个START信号掌管总线；
    2. 发送桓龅刂纷纸?包括7位地址码和一位R/W)；
    3. 当被控器件检测到主控发送的地址与自己的地址相同时发送一个应答信号(ACK)；
    4. 主控收到ACK后开始发送第一个数据字节；
    5. 被控器收到数据字节后发送一个ACK表示继续传送数据，发送NACK表示传送数据结束；
    6. 主控发送完全部数据后，发送一个停止位STOP，结束整个通讯并且释放总线；

读通讯过程:
    1. 主控在检测到总线空闲的状况下，首先发送一个START信号掌管总线；
    2. 发送一个地址字节(包括7位地址码和一位R/W)；
    3. 当被控器件检测到主控发送的地址与自己的地址相同时发送一个应答信号(ACK)；
    4. 主控收到ACK后释放数据总线，开始接收第一个数据字节；
    5. 主控收到数据后发送ACK表示继续传送数据，发送NACK表示传送数据结束；
    6. 主控发送完全部数据后，发送一个停止位STOP，结束整个通讯并且释放总线；

四. 总线信号时序分析
    1. 总线空闲状态
    SDA和SCL两条信号线都处于高电平，即总线上所有的器件都释放总线，两条信号线各自的上拉电阻把电平拉高；
    2. 启动信号START
    时钟信号SCL保持高电平，数据信号SDA的电平被拉低(即负跳变)。启动信号必须是跳变信号，而且在建立该信号前必修保证总线处于空闲状态；
    3. 停止信号STOP
    时钟信号SCL保持高电平，数据线被释放，使得SDA返回高电平(即正跳变)，停止信号也必须是跳变信号。
    4. 数据传送
    SCL线呈现高电平期间，SDA线上的电平必须保持稳定，低电平表示0(此时的线电压为地电压)，高电平表示1(此时的电压由元器件的VDD决定)。只有在SCL线为低电平期间，SDA上的电平允许变化。
    5. 应答信号ACK
    I2C总线的数据都是以字节(8位)的方式传送的，发送器件每发送一个字节之后，在时钟的第9个脉冲期间释放数据总线，由接收器发送一个ACK(把数据总线的电平拉低)来表示数据成功接收。
    6. 无应答信号NACK
    在时钟的第9个脉冲期间发送器释放数据总线，接收器不拉低数据总线表示一个NACK，NACK有两种用途:
    a. 一般表示接收器未成功接收数据字节；
    b. 当接收器是主控器时，它收到最后一个字节后，应发送一个NACK信号，以通知被控发送器结束数据发送，并释放总线，以便主控接收器发送一个停止信号STOP。
    7. 接收器件收到一个完整的数据字节后，有可能需要完成一些其它工作，如处理内部中断服务等，可能无法立刻接收下一个字节，这时接收器件可以将SCL线拉成低电平，从而使主机处于等待状态。直到接收器件准备好接收下一个字节时，再释放SCL线使之为高电平，从而使数据传送可以继续进行。
    8. I2C总线进行数据传送时，时钟信号为高电平期间，数据线上的数据必须保持稳定，只有在时钟线上的信号为低电平期间，数据线上的高电平或低电平状态才允许变化
五. 寻址约定
    地址的分配方法有两种:
    1. 含CPU的智能器件，地址由软件初始化时定义，但不能与其它的器件有冲突；
    2. 不含CPU的非智能器件，由厂家在器件内部固化，不可改变。

    高7位为地址码，其分为两部分:
    1. 高4位属于固定地址不可改变，由厂家固化的统一地址；
    2. 低三位为引脚设定地址，可以由外部引脚来设定(并非所有器件都可以设定)；


********************/
