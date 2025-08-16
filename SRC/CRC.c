#include "DSP2803x_Device.h"     // Headerfile Include File
#include "DSP2803x_Examples.h"   // Examples Include File
#include "ExternGlobals.h"

void CreatCRC(void)
{
    int i = 0;
    int GenPoly = 0x07;
    for(i = 1; i < 9; i++)
    {
        GenPoly = TxCommand[i] ^ GenPoly;
        GenPoly &= 0xff;
    }
    CRC = GenPoly;
    TxCommand[9] = CRC;
}



void ReverseCRC(void)
{

    int i = 0;
    int GenPoly = 0x07;
    for(i = 0; i < 9; i++)
    {
        GenPoly = RxCommand[i] ^ GenPoly;
        GenPoly &= 0xff;
    }
    CRC = GenPoly;

    if(CRC == 0)
    {


    }
}


