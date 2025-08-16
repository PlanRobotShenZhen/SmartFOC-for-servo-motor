#include "svpwm_dsp.h"
#include "IQmathLib.h"  // Include header for IQmath library 

void clarke(SVPVM *v)
{
    short Cosine, Sine;
    short tmpAngle;
    short tmp1 = 0;
    short tmp2 = 0;
    tmpAngle = v->Angle2;
    Sine = _IQ15sinPU(tmpAngle); //0--32767
    Cosine = _IQ15cosPU(tmpAngle);
    tmp1 = v->As;
    tmp2 = _IQ15mpy((v->As + _IQ15mpy(_IQ15(2), v->Bs)), _IQ15(0.57735026918963)); // 1/sqrt(3) = 0.57735026918963
    v->IDs = _IQ15mpy(tmp1, Cosine) + _IQ15mpy(tmp2, Sine);
    v->IQs = _IQ15mpy(tmp2, Cosine) - _IQ15mpy(tmp1, Sine);
}

short tmpDs, tmpQs;
void ipark(SVPVM *v)
{
    short Cosine, Sine;
    short tmpAngle;

    tmpAngle = v->Angle; //0--32767

    Sine = _IQ15sinPU(tmpAngle);
    Cosine = _IQ15cosPU(tmpAngle);

    tmpQs = _IQ12mpy(v->UQs, v->DcCoeff);
    tmpDs = _IQ12mpy(v->UDs, v->DcCoeff);

    if(tmpQs > 30500)// _IQ15(0.9))
    {
        tmpQs = 30500;//935
    }
    else if(tmpQs < -30500)// _IQ15(0.9))
    {
        tmpQs = -30500;//935
    }
    if(tmpDs > 6500)
    {
        tmpDs = 6500;
    }
    else if(tmpDs < -6500)
    {
        tmpDs = -6500;
    }

    v->Ualpha = _IQ15mpy(tmpDs, Cosine) - _IQ15mpy(tmpQs, Sine);
    v->Ubeta = _IQ15mpy(tmpQs, Cosine) + _IQ15mpy(tmpDs, Sine);
}

void svgendq(SVPVM *v)
{
    short Va, Vb, Vc, t1, t2;
    short Sector = 0;  // Sector is treated as Q0 - independently with global Q

    // Inverse clarke transformation
    Va = v->Ubeta;
    Vb = _IQ15mpy(_IQ15(-0.5), v->Ubeta) + _IQ15mpy(_IQ15(0.8660254), v->Ualpha); // 0.8660254 = sqrt(3)/2
    Vc = _IQ15mpy(_IQ15(-0.5), v->Ubeta) - _IQ15mpy(_IQ15(0.8660254), v->Ualpha); // 0.8660254 = sqrt(3)/2

    // 60 degree Sector determination
    if(Va > _IQ15(0))
        Sector = 1;
    if(Vb > _IQ15(0))
        Sector = Sector + 2;
    if(Vc > _IQ15(0))
        Sector = Sector + 4;

    // X,Y,Z (Va,Vb,Vc) calculations
    Va = v->Ubeta;                                                       // X = Va
    Vb = _IQ15mpy(_IQ15(0.5), v->Ubeta) + _IQ15mpy(_IQ15(0.8660254), v->Ualpha); // Y = Vb
    Vc = _IQ15mpy(_IQ15(0.5), v->Ubeta) - _IQ15mpy(_IQ15(0.8660254), v->Ualpha); // Z = Vc

    if(Sector == 0) // Sector 0: this is special case for (Ualpha,Ubeta) = (0,0)
    {
        v->Ta = _IQ15(0.5);
        v->Tb = _IQ15(0.5);
        v->Tc = _IQ15(0.5);
    }
    if(Sector == 1) // Sector 1: t1=Z and t2=Y (abc ---> Tb,Ta,Tc)
    {
        t1 = Vc;
        t2 = Vb;
        v->Tb = _IQ15mpy(_IQ15(0.5), (_IQ15(1) - t1 - t2)); // tbon = (1-t1-t2)/2
        v->Ta = v->Tb + t1;                           // taon = tbon+t1
        v->Tc = v->Ta + t2;                           // tcon = taon+t2
    }
    else if(Sector == 2) // Sector 2: t1=Y and t2=-X (abc ---> Ta,Tc,Tb)
    {
        t1 = Vb;
        t2 = -Va;
        v->Ta = _IQ15mpy(_IQ15(0.5), (_IQ15(1) - t1 - t2)); // taon = (1-t1-t2)/2
        v->Tc = v->Ta + t1;                           // tcon = taon+t1
        v->Tb = v->Tc + t2;                           // tbon = tcon+t2
    }
    else if(Sector == 3) // Sector 3: t1=-Z and t2=X (abc ---> Ta,Tb,Tc)
    {
        t1 = -Vc;
        t2 = Va;
        v->Ta = _IQ15mpy(_IQ15(0.5), (_IQ15(1) - t1 - t2)); // taon = (1-t1-t2)/2
        v->Tb = v->Ta + t1;                           // tbon = taon+t1
        v->Tc = v->Tb + t2;                           // tcon = tbon+t2
    }
    else if(Sector == 4) // Sector 4: t1=-X and t2=Z (abc ---> Tc,Tb,Ta)
    {
        t1 = -Va;
        t2 = Vc;
        v->Tc = _IQ15mpy(_IQ15(0.5), (_IQ15(1) - t1 - t2)); // tcon = (1-t1-t2)/2
        v->Tb = v->Tc + t1;                           // tbon = tcon+t1
        v->Ta = v->Tb + t2;                           // taon = tbon+t2
    }
    else if(Sector == 5) // Sector 5: t1=X and t2=-Y (abc ---> Tb,Tc,Ta)
    {
        t1 = Va;
        t2 = -Vb;
        v->Tb = _IQmpy(_IQ(0.5), (_IQ15(1) - t1 - t2)); // tbon = (1-t1-t2)/2
        v->Tc = v->Tb + t1;                           // tcon = tbon+t1
        v->Ta = v->Tc + t2;                           // taon = tcon+t2
    }
    else if(Sector == 6) // Sector 6: t1=-Y and t2=-Z (abc ---> Tc,Ta,Tb)
    {
        t1 = -Vb;
        t2 = -Vc;
        v->Tc = _IQ15mpy(_IQ15(0.5), (_IQ15(1) - t1 - t2)); // tcon = (1-t1-t2)/2
        v->Ta = v->Tc + t1;                           // taon = tcon+t1
        v->Tb = v->Ta + t2;                           // tbon = taon+t2
    }
    // Convert the unsigned GLOBAL_Q format (ranged (0,1)) -> signed GLOBAL_Q format (ranged (-1,1))
    v->Ta = _IQ15mpy(_IQ15(2.0), (v->Ta - _IQ15(0.5)));
    v->Tb = _IQ15mpy(_IQ15(2.0), (v->Tb - _IQ15(0.5)));
    v->Tc = _IQ15mpy(_IQ15(2.0), (v->Tc - _IQ15(0.5)));
}


void PWM(SVPVM *v)
{
    short MPeriod;
    int Tmp;

    v->MfuncC1 = v->Ta; // MfuncC1 is in Q15
    v->MfuncC2 = v->Tb; // MfuncC2 is in Q15
    v->MfuncC3 = v->Tc; // MfuncC3 is in Q15

    // Compute the timer period (Q0) from the period modulation input (Q15)
    Tmp = (int)v->PeriodMax * (int)v->MfuncPeriod;         // Q15 = Q0*Q15
    MPeriod = (short)(Tmp >> 16) + (short)(v->PeriodMax >> 1); // Q0 = (Q15->Q0)/2 + (Q0/2)

    //Compute the compare A (Q0) from the EPWM1AO & EPWM1BO duty cycle ratio (Q15)
    Tmp = (int)MPeriod * (int)v->MfuncC1;                  // Q15 = Q0*Q15
    //EPwm4Regs.CMPA.half.CMP_A= (int16)(Tmp>>16) + (int16)(MPeriod>>1);   // Q0 = (Q15->Q0)/2 + (Q0/2)
    v->Va = (short)(Tmp >> 16) + (short)(MPeriod >> 1); // Q0 = (Q15->Q0)/2 + (Q0/2)

    //Compute the compare B (Q0) from the EPWM2AO & EPWM2BO duty cycle ratio (Q15)
    Tmp = (int)MPeriod * (int)v->MfuncC2;                 // Q15 = Q0*Q15
    //EPwm5Regs.CMPA.half.CMP_A = (int16)(Tmp>>16) + (int16)(MPeriod>>1);  // Q0 = (Q15->Q0)/2 + (Q0/2)
    v->Vb = (short)(Tmp >> 16) + (short)(MPeriod >> 1); // Q0 = (Q15->Q0)/2 + (Q0/2)

    //Compute the compare C (Q0) from the EPWM3AO & EPWM3BO duty cycle ratio (Q15)
    Tmp = (int)MPeriod * (int)v->MfuncC3;                 // Q15 = Q0*Q15
    //EPwm6Regs.CMPA.half.CMP_A = (int16)(Tmp>>16) + (int16)(MPeriod>>1);  // Q0 = (Q15->Q0)/2 + (Q0/2)
    v->Vc = (short)(Tmp >> 16) + (short)(MPeriod >> 1); // Q0 = (Q15->Q0)/2 + (Q0/2)
}

