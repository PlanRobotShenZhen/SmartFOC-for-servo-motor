
//#include "ExternGlobals.h"
//#include "SpeedPlan2.h"
//#include "pidc.h"
//#include "IQmathLib.h"
//#include "ExternGlobals.h"
//#include "FLASH_WR.h"
//#include "UartMode.h"

//int TestSpeedTable[20] = {10, 300, 50, -10, 0, -3000, 50, 500, 800, -1500, 3000, -3000, 500, 0};
//int TestSpeedCount = 0;
//int TestSpeedStep = 0;
//void TestMode_Runing(void)
//{
//    if(UartMode.Mode == 3) //速度控制
//    {
//        switch(UartMode.State)
//        {
//            case 0 :
//            {
//                pidpv.OutMax = pidpv_OutMax;
//                pidpv.OutMin = pidpv_OutMin;
//                pidpv.UiMax = pidpv_UiMax;
//                pidpv.UiMin = pidpv_UiMin;
//                pidpv.Kp = pidpv_Kp;
//                pidpv.Ki = pidpv_Ki;

//                pidpv.Err = 0;
//                MotorControler.SpeedRef = 0;

//                MotorControler.State = 0;
//                pidpv.Ref = 0;
//                pidpv.Ui = 0;


//                if(UartMode.CMD == 15)
//                {
//                    UartMode.State = 1;
//                    UartMode.CMD = 0;
//                }
//            }
//            break;

//            case 1 :
//            {
//                if(UartMode.CMD == 31)
//                {
//                    UartMode.State = 2;
//                    UartMode.CMD = 0;
//                }

//                MotorControler.State = 6;
//                MotorControler.SpeedRef = 0;
//            }
//            break;

//            case 2 :  //电机正在运行过程中
//            {
//                if(UartMode.CMD == 128)
//                {
//                    UartMode.State = 0;
//                    UartMode.CMD = 0;
//                }

//                TestSpeedCount++;

//                if(TestSpeedCount >= 50)
//                {
//                    TestSpeedCount = 0;
//                    TestSpeedStep++;

//                    if(TestSpeedStep >= 10)
//                    {
//                        TestSpeedStep = 0;
//                    }
//                }

//                MotorControler.SpeedRef = TestSpeedTable[TestSpeedStep];
//                MotorControler.TorqueRef  = 600;
//                MotorControler.SpeedAcc = 3000;
//                MotorControler.SpeedDcc = 3000;
//            }
//            break;
//        }
//    }

//}







//uint8_t ui8Index, ui8Sign = 0;
//uint_fast32_t ui32Temp;
//uint_fast32_t uiq30Guess;
//uint_fast32_t uiqNInput1;
//uint_fast32_t uiqNInput2;
//uint_fast32_t uiqNResult;
//unsigned long long  uiiqNInput1;
//uint_fast16_t ui16IntState;
//uint_fast16_t ui16MPYState;


//char a;              //8
//unsigned char b;     //8
//short c;             //32
//unsigned short d;    //32
//int e;               //32
//unsigned int f;      //32
//long int g;          //32
//unsigned long int h; //32
//long long i ;        //64
//unsigned long long j;//64




//#define TYPE_DEFAULT    (0)
//#define TYPE_SQRT       (1)
//#define TYPE_MAG        (2)
//#define TYPE_IMAG       (3)


//int a__IQNdiv(int iqNInput1, int iqNInput2, const unsigned char type, const int8_t q_value);
//int_fast32_t a__IQNsqrt(int_fast32_t iqNInputX, int_fast32_t iqNInputY, const int8_t q_value, const int8_t type);
//int_fast64_t a__mpyx(int_fast32_t arg1, int_fast32_t arg2);
//unsigned int a__mpyf_ul(unsigned int arg1, unsigned int arg2);

//short	a_IQ15abs(short a);
//short a_IQ15mpy(short a, short b);
//int a_IQ30mpy(int a, int  b);
//int a_IQdiv2(int a);
//short a_IQ15(float a);

//int32_t a_IQ15div(int32_t a, int32_t b);
//int32_t a_IQ15sqrt(int32_t a);



//short inputa_IQ15abs=0;
//short intputa1_IQ15mpy=0,intputa2_IQ15mpy=0;
//int intputa1_IQ30mpy=0,intputa2_IQ30mpy=0;
//	
//int  intputa_IQdiv2;
//float  intputa_IQ15;
//int   intputa1_IQ15div,intputa2_IQ15div;
//int   intputa_IQ15sqrt;




//short outputa_IQ15abs=0;
//short outputa_IQ15mpy=0;
//int outputa_IQ30mpy=0;
//	
//int  outtputa_IQdiv2;
//short  outputa_IQ15;
//int   outputa_IQ15div;
//int   outputa_IQ15sqrt;



//short output_IQ15abs=0;
//short output_IQ15mpy=0;
//int output_IQ30mpy=0;
//	
//int  outtput_IQdiv2;
//short  output_IQ15;
//int   output_IQ15div;
//int   output_IQ15sqrt;


//void iqtest(void)
//{
//	
//	    outputa_IQ15abs=a_IQ15abs(inputa_IQ15abs);
//	 
//	    outputa_IQ15mpy=a_IQ15mpy(intputa1_IQ15mpy,intputa2_IQ15mpy);
//	
//	    outputa_IQ30mpy=a_IQ30mpy(intputa1_IQ30mpy,intputa2_IQ30mpy);
//	
//	    outtputa_IQdiv2=a_IQdiv2(intputa_IQdiv2);
//	
//	    outputa_IQ15=a_IQ15(intputa_IQ15);
//	
//	    outputa_IQ15div=a_IQ15div(intputa1_IQ15div,intputa2_IQ15div);
//	
//	    outputa_IQ15sqrt=a_IQ15sqrt(intputa_IQ15sqrt);
//	
//	
//		  output_IQ15abs=_IQ15abs(inputa_IQ15abs);
//	 
//	    output_IQ15mpy=_IQ15mpy(intputa1_IQ15mpy,intputa2_IQ15mpy);
//	
//	    output_IQ30mpy=_IQ30mpy(intputa1_IQ30mpy,intputa2_IQ30mpy);
//	
//	    outtput_IQdiv2=_IQdiv2(intputa_IQdiv2);
//	
//	    output_IQ15=_IQ15(intputa_IQ15);
//	
//	    output_IQ15div=_IQ15div(intputa1_IQ15div,intputa2_IQ15div);
//	
//	    output_IQ15sqrt=_IQ15sqrt(intputa_IQ15sqrt);
//	
//}















//short	a_IQ15abs(short a)
//{
//    if(a >= 0)
//    {
//        return a;
//    }
//    else
//    {
//        return 0 - a;
//    }
//}


//short a_IQ15mpy(short a, short b)
//{
//    int c = 0;
//    short d = 0;
//    c = ((int)a)  * ((int)b);
//    d = c >> 15;
//    return d;
//}



//int a_IQ30mpy(int a, int  b)
//{
//    long long c = 0;
//    int  d = 0;
//    c = ((long long)a)  * ((long long)b);
//    d = (int)(c >> 30);
//    return d;
//}



//int a_IQdiv2(int a)
//{
//    return a >> 1;
//}

//short a_IQ15(float a)
//{
//    int b = 0;
//    short c ;
//    b = a * 32768;
//    c = (short)b;
//    return c;
//}



//int32_t a_IQ15div(int32_t a, int32_t b)
//{
//    return a__IQNdiv(a, b, TYPE_DEFAULT, 15);
//}


//unsigned int a__mpyf_ul(unsigned int arg1, unsigned int arg2)
//{
//    return (unsigned int)(((unsigned long long )arg1 * (unsigned long long )arg2) >> 31);

//}

//int a__mpyf_ul_reuse_arg1(unsigned int arg1, unsigned int arg2)
//{
//    /* This is identical to __mpyf_ul */
//    return (unsigned int)(((unsigned long long )arg1 * (unsigned long long )arg2) >> 31);
//}

//int a__IQNdiv(int iqNInput1, int iqNInput2, const unsigned char type, const int8_t q_value)
//{
//    unsigned char ui8Index, ui8Sign = 0;
//    unsigned int ui32Temp;
//    unsigned int uiq30Guess;
//    unsigned int uiqNInput1;
//    unsigned int uiqNInput2;
//    unsigned int uiqNResult;
//    unsigned long long  uiiqNInput1;
////    unsigned short ui16IntState;
////    unsigned short ui16MPYState;


//    const unsigned char _IQ6div_lookup[65] =
//    {
//        0x7F, 0x7D, 0x7B, 0x79, 0x78, 0x76, 0x74, 0x73,
//        0x71, 0x6F, 0x6E, 0x6D, 0x6B, 0x6A, 0x68, 0x67,
//        0x66, 0x65, 0x63, 0x62, 0x61, 0x60, 0x5F, 0x5E,
//        0x5D, 0x5C, 0x5B, 0x5A, 0x59, 0x58, 0x57, 0x56,
//        0x55, 0x54, 0x53, 0x52, 0x52, 0x51, 0x50, 0x4F,
//        0x4E, 0x4E, 0x4D, 0x4C, 0x4C, 0x4B, 0x4A, 0x49,
//        0x49, 0x48, 0x48, 0x47, 0x46, 0x46, 0x45, 0x45,
//        0x44, 0x43, 0x43, 0x42, 0x42, 0x41, 0x41, 0x40, 0x40
//    };

//    if (type == TYPE_DEFAULT)
//    {
//        /* save sign of denominator */
//        if (iqNInput2 <= 0)
//        {
//            /* check for divide by zero */
//            if (iqNInput2 == 0)
//            {
//                return INT32_MAX;
//            }
//            else
//            {
//                ui8Sign = 1;
//                iqNInput2 = -iqNInput2;
//            }
//        }

//        /* save sign of numerator */
//        if (iqNInput1 < 0)
//        {
//            ui8Sign ^= 1;
//            iqNInput1 = -iqNInput1;
//        }
//    }
//    else
//    {
//        /* Check for divide by zero */
//        if (iqNInput2 == 0)
//        {
//            return INT32_MAX;
//        }
//    }

//    /* Save input1 and input2 to unsigned IQN and IIQN (64-bit). */
//    uiiqNInput1 = (unsigned long long )iqNInput1;
//    uiqNInput2 = (unsigned int)iqNInput2;

//    /* Scale inputs so that 0.5 <= uiqNInput2 < 1.0. */
//    while (uiqNInput2 < 0x40000000)
//    {
//        uiqNInput2 <<= 1;
//        uiiqNInput1 <<= 1;
//    }

//    /*
//     * Shift input1 back from iq31 to iqN but scale by 2 since we multiply
//     * by result in iq30 format.
//     */
//    if (q_value < 31)
//    {
//        uiiqNInput1 >>= (31 - q_value - 1);
//    }
//    else
//    {
//        uiiqNInput1 <<= 1;
//    }

//    /* Check for saturation. */
//    if (uiiqNInput1 >> 32)
//    {
//        if (ui8Sign)
//        {
//            return INT32_MIN;
//        }
//        else
//        {
//            return INT32_MAX;
//        }
//    }
//    else
//    {
//        uiqNInput1 = (unsigned int)uiiqNInput1;
//    }

//    /* use left most 7 bits as ui8Index into lookup table (range: 32-64) */
//    ui8Index = uiqNInput2 >> 24;
//    ui8Index -= 64;
//    uiq30Guess = (unsigned int)_IQ6div_lookup[ui8Index] << 24;

//    /*
//     * Mark the start of any multiplies. This will disable interrupts and set
//     * the multiplier to fractional mode. This is designed to reduce overhead
//     * of constantly switching states when using repeated multiplies (MSP430
//     * only).
//     */
//    //__mpyf_start(&ui16IntState, &ui16MPYState);
//    /* Do nothing. */

//    /* 1st iteration */
//    ui32Temp = a__mpyf_ul(uiq30Guess, uiqNInput2);
//    ui32Temp = -((unsigned int)ui32Temp - 0x80000000);
//    ui32Temp = ui32Temp << 1;
//    uiq30Guess = a__mpyf_ul_reuse_arg1(uiq30Guess, ui32Temp);

//    /* 2nd iteration */
//    ui32Temp = a__mpyf_ul(uiq30Guess, uiqNInput2);
//    ui32Temp = -((unsigned int)ui32Temp - 0x80000000);
//    ui32Temp = ui32Temp << 1;
//    uiq30Guess = a__mpyf_ul_reuse_arg1(uiq30Guess, ui32Temp);

//    /* 3rd iteration */
//    ui32Temp = a__mpyf_ul(uiq30Guess, uiqNInput2);
//    ui32Temp = -((unsigned int)ui32Temp - 0x80000000);
//    ui32Temp = ui32Temp << 1;
//    uiq30Guess = a__mpyf_ul_reuse_arg1(uiq30Guess, ui32Temp);

//    /* Multiply 1/uiqNInput2 and uiqNInput1. */
//    uiqNResult = a__mpyf_ul(uiq30Guess, uiqNInput1);

//    /*
//     * Mark the end of all multiplies. This restores MPY and interrupt states
//     * (MSP430 only).
//     */
//    //__mpy_stop(&ui16IntState, &ui16MPYState);
//    /* Do nothing. */

//    /* Saturate, add the sign and return. */
//    if (type == TYPE_DEFAULT)
//    {
//        if (uiqNResult > INT32_MAX)
//        {
//            if (ui8Sign)
//            {
//                return INT32_MIN;
//            }
//            else
//            {
//                return INT32_MAX;
//            }
//        }
//        else
//        {
//            if (ui8Sign)
//            {
//                return -(int)uiqNResult;
//            }
//            else
//            {
//                return (int)uiqNResult;
//            }
//        }
//    }
//    else
//    {
//        return uiqNResult;
//    }
//}



//int32_t a_IQ15sqrt(int32_t a)
//{
//    return a__IQNsqrt(a, 0, 15, TYPE_SQRT);
//}



// int_fast64_t a__mpyx(int_fast32_t arg1, int_fast32_t arg2)
//{
//    return ((int_fast64_t)arg1 * (int_fast64_t)arg2);
//}




// int_fast32_t a__IQNsqrt(int_fast32_t iqNInputX, int_fast32_t iqNInputY, const int8_t q_value, const int8_t type)
//{
//    uint8_t ui8Index;
//    uint8_t ui8Loops;
//    int_fast16_t i16Exponent;
////    uint_fast16_t ui16IntState;
////    uint_fast16_t ui16MPYState;
//    uint_fast32_t uiq30Guess;
//    uint_fast32_t uiq30Result;
//    uint_fast32_t uiq31Result;
//    uint_fast32_t uiq32Input;
//    
//	
//	/* sqrt */
//const uint_fast16_t _IQ14sqrt_lookup[96] = {
//    0x7f02, 0x7d19, 0x7b46, 0x7986, 0x77d9, 0x763d, 0x74b2, 0x7335, 
//    0x71c7, 0x7066, 0x6f11, 0x6dc8, 0x6c8b, 0x6b58, 0x6a2f, 0x690f, 
//    0x67f8, 0x66ea, 0x65e4, 0x64e5, 0x63ee, 0x62fe, 0x6214, 0x6131, 
//    0x6054, 0x5f7d, 0x5eab, 0x5dde, 0x5d17, 0x5c54, 0x5b96, 0x5add, 
//    0x5a28, 0x5977, 0x58ca, 0x5821, 0x577c, 0x56da, 0x563c, 0x55a1, 
//    0x5509, 0x5475, 0x53e3, 0x5354, 0x52c9, 0x523f, 0x51b9, 0x5135, 
//    0x50b3, 0x5034, 0x4fb7, 0x4f3d, 0x4ec4, 0x4e4e, 0x4dda, 0x4d68, 
//    0x4cf7, 0x4c89, 0x4c1d, 0x4bb2, 0x4b49, 0x4ae1, 0x4a7c, 0x4a18, 
//    0x49b5, 0x4954, 0x48f4, 0x4896, 0x483a, 0x47de, 0x4784, 0x472c, 
//    0x46d4, 0x467e, 0x4629, 0x45d6, 0x4583, 0x4532, 0x44e1, 0x4492, 
//    0x4444, 0x43f7, 0x43aa, 0x435f, 0x4315, 0x42cc, 0x4284, 0x423c, 
//    0x41f6, 0x41b0, 0x416b, 0x4127, 0x40e4, 0x40a2, 0x4060, 0x4020
//};
//	
//	
//	
//	
//    /* If the type is (inverse) magnitude we need to calculate x^2 + y^2 first. */
//    if (type == TYPE_MAG || type == TYPE_IMAG) {
//        uint_fast64_t ui64Sum;
//        
//        //__mpy_start(&ui16IntState, &ui16MPYState);
//			/* Do nothing. */
//        
//        /* Calculate x^2 */
//        ui64Sum = a__mpyx(iqNInputX, iqNInputX);
//        
//        /* Calculate y^2 and add to x^2 */
//        ui64Sum += a__mpyx(iqNInputY, iqNInputY);
//    
//        //__mpy_stop(&ui16IntState, &ui16MPYState);
//			 /* Do nothing. */
//        
//        /* Return if the magnitude is simply zero. */
//        if (ui64Sum == 0) {
//            return 0;
//        }
//        
//        /*
//         * Initialize the exponent to positive for magnitude, negative for
//         * inverse magnitude.
//         */
//        if (type == TYPE_MAG) {
//            i16Exponent = (32 - q_value);
//        }
//        else {
//            i16Exponent = -(32 - q_value);
//        }
//        
//        /* Shift to iq64 by keeping track of exponent. */
//        while ((uint_fast16_t)(ui64Sum >> 48) < 0x4000) {
//            ui64Sum <<= 2;
//            /* Decrement exponent for mag */
//            if (type == TYPE_MAG) {
//                i16Exponent--;
//            }
//            /* Increment exponent for imag */
//            else {
//                i16Exponent++;
//            }
//        }
//        
//        /* Shift ui64Sum to unsigned iq32 and set as uiq32Input */
//        uiq32Input = (uint_fast32_t)(ui64Sum >> 32);
//    }
//    else {
//        /* check sign of input */
//        if (iqNInputX <= 0) {
//            return 0;
//        }
//    
//        /* If the q_value gives an odd starting exponent make it even. */
//        if ((32 - q_value) % 2 == 1) {
//            iqNInputX <<= 1;
//            /* Start with positive exponent for sqrt */
//            if (type == TYPE_SQRT) {
//                i16Exponent = ((32 - q_value) - 1) >> 1;
//            }
//            /* start with negative exponent for isqrt */
//            else {
//                i16Exponent = -(((32 - q_value) - 1) >> 1);
//            }
//        }
//        else {
//            /* start with positive exponent for sqrt */
//            if (type == TYPE_SQRT) {
//                i16Exponent = (32 - q_value) >> 1;
//            }
//            /* start with negative exponent for isqrt */
//            else {
//                i16Exponent = -((32 - q_value) >> 1);
//            }
//        }
//        
//        /* Save input as unsigned iq32. */
//        uiq32Input = (uint_fast32_t)iqNInputX;
//        
//        /* Shift to iq32 by keeping track of exponent */
//        while ((uint_fast16_t)(uiq32Input >> 16) < 0x4000) {
//            uiq32Input <<= 2;
//            /* Decrement exponent for sqrt and mag */
//            if (type) {
//                i16Exponent--;
//            }
//            /* Increment exponent for isqrt */
//            else {
//                i16Exponent++;
//            }
//        }
//    }
//    
//    
//    /* Use left most byte as index into lookup table (range: 32-128) */
//    ui8Index = uiq32Input >> 25;
//    ui8Index -= 32;
//    uiq30Guess = (uint_fast32_t)_IQ14sqrt_lookup[ui8Index] << 16;
//    
//    /*
//     * Mark the start of any multiplies. This will disable interrupts and set
//     * the multiplier to fractional mode. This is designed to reduce overhead
//     * of constantly switching states when using repeated multiplies (MSP430
//     * only).
//     */
//    //__mpyf_start(&ui16IntState, &ui16MPYState);
//		/* Do nothing. */
//    
//    /*
//     * Set the loop counter:
//     *
//     *     iq1 <= q_value < 24 - 2 loops
//     *     iq22 <= q_value <= 31 - 3 loops
//     */
//    if (q_value < 24) {
//        ui8Loops = 2;
//    }
//    else {
//        ui8Loops = 3;
//    }

//    /* Iterate through Newton-Raphson algorithm. */
//    while (ui8Loops--) {
//        /* x*g */
//        uiq31Result = a__mpyf_ul(uiq32Input, uiq30Guess);
//        
//        /* x*g*g */
//        uiq30Result = a__mpyf_ul(uiq31Result, uiq30Guess);
//        
//        /* 3 - x*g*g */
//        uiq30Result = -(uiq30Result - 0xC0000000);
//        
//        /* 
//         * g/2*(3 - x*g*g) 
//         * uiq30Guess = uiq31Guess/2
//         */
//        uiq30Guess = a__mpyf_ul(uiq30Guess, uiq30Result);
//    }
//    
//    /* Calculate sqrt(x) for both sqrt and mag */
//    if (type == TYPE_SQRT || type == TYPE_MAG) {
//        /*
//         * uiq30Guess contains the inverse square root approximation, multiply
//         * by uiq32Input to get square root result.
//         */
//        uiq31Result = a__mpyf_ul(uiq30Guess, uiq32Input);
//    
//        //__mpy_stop(&ui16IntState, &ui16MPYState);
//			   /* Do nothing. */
//    
//        /*
//         * Shift the result right by 31 - q_value.
//         */
//        i16Exponent -= (31 - q_value);
//        
//        /* Saturate value for any shift larger than 1 (only need this for mag) */
//        if (type == TYPE_MAG) {
//            if (i16Exponent > 0) {
//                return 0x7fffffff;
//            }
//        }
//        
//        /* Shift left by 1 check only needed for iq30 and iq31 mag/sqrt */
//        if (q_value >= 30) {
//            if (i16Exponent > 0) {
//                uiq31Result <<= 1;
//                return uiq31Result;
//            }
//        }
//    }
//    /* Separate handling for isqrt and imag. */
//    else {
//        //__mpy_stop(&ui16IntState, &ui16MPYState);
//         /* Do nothing. */
//        /*
//         * Shift the result right by 31 - q_value, add one since we use the uiq30
//         * result without shifting.
//         */
//        i16Exponent = i16Exponent - (31 - q_value) + 1;
//        uiq31Result = uiq30Guess;
//        
//        /* Saturate any positive non-zero exponent for isqrt. */
//        if (i16Exponent > 0) {
//            return 0x7fffffff;
//        }
//    }
//        
//    /* Shift uiq31Result right by -exponent */
//    if (i16Exponent <= -32) {
//        return 0;
//    }
//    if (i16Exponent <= -16) {
//        uiq31Result >>= 16;
//        i16Exponent += 16;
//    }
//    if (i16Exponent <= -8) {
//        uiq31Result >>= 8;
//        i16Exponent += 8;
//    }
//    while (i16Exponent < -1) {
//        uiq31Result >>= 1;
//        i16Exponent++;
//    }
//    if (i16Exponent) {
//        uiq31Result++;
//        uiq31Result >>= 1;
//    }        
//    
//    return uiq31Result;
//}





