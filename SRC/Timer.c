#include "Timer1.h"
#define M_ENABLE0 1


void Timeing(TIMER *v);

#include "ExternGlobals.h"   //ygj


void Timeing(TIMER *v)
{
    v->Tmer100us ++;

    if(v->Tmer100us >= 10)
    {
        v->Tmer100us = 0;
        v->Flag1ms = M_ENABLE0;
        v->Tmer1ms ++ ;
			  v->Tmer1ms25 ++ ;
			  v->Tmer1ms50 ++ ;
    }

    if(v->Tmer1ms >= 10)
    {
        v->Flag10ms = M_ENABLE0;
        v->Tmer1ms = 0;
        v->Tmer10ms ++;
        v->Tmer25ms ++;
    }
    if(v->Tmer1ms25 >= 25)
    {
        v->Flag25ms = M_ENABLE0;
        v->Tmer1ms25 = 0;
    }
    if(v->Tmer1ms50 >= 50)
    {
        v->Flag50ms = M_ENABLE0;
        v->Tmer1ms50 = 0;
    }		
    if(v->Tmer10ms >= 10)
    {
        v->Flag100ms = M_ENABLE0;
        v->Tmer10ms = 0;
    }

    if(v->Tmer25ms >= 25)
    {
        v->Flag250ms = M_ENABLE0;
        v->Tmer25ms = 0;
        v->Tmer250ms ++;
    }

    if(v->Tmer250ms >= 2) //0.5Ãë
    {
        v->Flag500ms = M_ENABLE0;
        v->Tmer250ms = 0;
        v->Tmer500ms ++;
       
    }

    if(v->Tmer500ms >= 2) //1ÃëÖÓ
    {
        v->Flag1s = M_ENABLE0;
        v->Tmer500ms = 0;
        v->Tmer1s ++;
    }

    if(v->Tmer1s >= 60) //1·ÖÖÓ
    {
        v->Flag1m = M_ENABLE0;
        v->Tmer1s = 0;
        v->Tmer1m ++;
    }

    if(v->Tmer1m >= 60) //1Ğ¡Ê±
    {
        v->Flag1h = M_ENABLE0;
        v->Tmer1m = 0;
        v->Tmer1h++;
    }
}
