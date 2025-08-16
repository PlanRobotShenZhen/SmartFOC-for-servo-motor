//#include "SystemDefine.h"
#include "Spi.h"
#include "ExternGlobals.h"
extern short SpiReadState; //SPI读取状态
extern short SpiReadState2; //SPI读取状态
u16 SpiBuffer[2] = {0}; //存储读取数据

void SPI3_IRQHandler(void)
{
    uint32_t tempdat = 0, tmpv = 0;
    short 	NoMg = 0, LPV = 0, Over_Speed = 0, CRCV = 0;

    if(SPI_I2S_GetIntStatus(SPI3, SPI_I2S_INT_RNE) != RESET)
    {
        switch(SpiReadState)
        {
            case 0:
                SPI_I2S_ReceiveData(SPI3);
                SpiReadState = 1;
                MT6835_Read_Reg(0xFFFF);
                break;

            case 1:
                SpiBuffer[0] = SPI_I2S_ReceiveData(SPI3);
                SpiReadState = 2;
                MT6835_Read_Reg(0xFFFF);
                break;

            case 2:
                SpiBuffer[1] = SPI_I2S_ReceiveData(SPI3);
                SpiReadState = 0;
                CSn1_H;                //禁止SPI传输

                tmpv = SpiBuffer[0];
                tempdat = (tmpv << 16);
                tmpv = SpiBuffer[1];
                tempdat += tmpv;

                //处理数据
                if(tempdat & 0x00000400)
                    LPV = 1; //欠压报警，1报警0正常
                else
                    LPV = 0;

                if(tempdat & 0x00000200)
                    NoMg = 1; //外加磁场太弱1报警0正常
                else
                    NoMg = 0;

                if(tempdat & 0x00000100)
                    Over_Speed = 1; //过速提示1报警0正常
                else
                    Over_Speed = 0;

                CRCV = (u8)tempdat;
                tempdat = (tempdat >> 11); //21位
                NoMg = NoMg;
                LPV = LPV;
                Over_Speed = Over_Speed;
                CRCV = CRCV;

                if((LPV == 0) && (NoMg == 0) && (Over_Speed == 0))
                {
                    MotorControler.AngleFromMT6835 = 0x7fff - (tempdat >> 6);
                }

                SpiReadState = 0;
                SpiReadState2 = 0;
                break;
        }
    }
}


void Delay_Us(unsigned int i)  //j=5,i=1时大约600-700ns左右
{
    unsigned int j;
    for(j = 5; j > 0; j--)
        for(; i > 0; i--);
}
void MT6835_Init(void)
{
    NVIC_InitType NVIC_InitStructure;
    /* Configure and enable SPI_MASTER interrupt -------------------------------*/
    NVIC_InitStructure.NVIC_IRQChannel                   = SPI3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    SPI_InitType  SPI_InitStructure;

    SPI_InitStructure.DataDirection = SPI_DIR_DOUBLELINE_FULLDUPLEX;
    SPI_InitStructure.SpiMode = SPI_MODE_MASTER;
    SPI_InitStructure.DataLen = SPI_DATA_SIZE_16BITS;//gj点
    SPI_InitStructure.CLKPOL = SPI_CLKPOL_HIGH;
    SPI_InitStructure.CLKPHA = SPI_CLKPHA_SECOND_EDGE;
    SPI_InitStructure.NSS = SPI_NSS_SOFT;
    SPI_InitStructure.BaudRatePres = SPI_BR_PRESCALER_16;//SPI_BR_PRESCALER_256;
    SPI_InitStructure.FirstBit = SPI_FB_MSB;
    SPI_InitStructure.CRCPoly = 7;
    SPI_Init(SPI3, &SPI_InitStructure);

    SPI_I2S_EnableInt(SPI3, SPI_I2S_INT_RNE, ENABLE); //使能接收中断

    SPI_Enable(SPI3, ENABLE);
}

u16 SPI_ReadWriteByte(u16 TxData)                                        //SPI读写数据函数
{
    uint16_t retry = 0;

    /* Loop while DR register in not emplty */
    while(SPI_I2S_GetStatus(SPI3, SPI_I2S_TE_FLAG) == RESET)       //发送缓存标志位为空
    {
        retry++;

        if(retry > 200)
            return 0;
    }

    /* Send byte through the SPI1 peripheral */
    SPI_I2S_TransmitData(SPI3, TxData);     //通过外设SPI1发送一个数据
    retry = 0;
    return 0;   //不通过阻塞获取接收数据，而是通过中断
}
u16 MT6835_Read_Reg(u16 regaddr)
{
    u16 reg_val = 0;
    reg_val = SPI_ReadWriteByte(regaddr); //读取寄存器内容
    return(reg_val);       //返回状态值
}


void delay_Ms(uint16_t nms)
{
    u32 i = 0, j = 0;

    for(i = 0; i < nms; i++)
    {
        for(j = 0; j < 7200; j++);
    }
}
