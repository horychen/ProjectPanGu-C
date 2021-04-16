/*
 * yxsvgen.c
 *
 *  Created on: 2021年1月11日
 *      Author: xinyuan
 */
#include <All_Definition.h>

void SVGEN_Drive(SVGENDQ* ptrV)
{
	float Va,Vb,Vc,t1,t2,temp_sv1,temp_sv2,t_voltage;
	int Sector = 0;  // Sector is treated as Q0 - independently with global Q
	SVGENDQ  v;
	v=(*ptrV);
	Sector = 0;
	t_voltage=Voltage_DC_BUS*0.577367;
	temp_sv1=(v.Ubeta*0.8660254)/Voltage_DC_BUS; 					/*divide by 2*/
	temp_sv2=(1.5*v.Ualpha)/Voltage_DC_BUS;	/* 0.8660254 = sqrt(3)/2*/

/* Inverse clarke transformation */
	Va = v.Ubeta/t_voltage;
	Vb = -temp_sv1 + temp_sv2;
	Vc = -temp_sv1 - temp_sv2;
/* 60 degree Sector determination */
	if (Va>0) Sector = 1;
	if (Vb>0) Sector = Sector+2;
	if (Vc>0) Sector = Sector+4;
/* X,Y,Z (Va,Vb,Vc) calculations X = Va, Y = Vb, Z = Vc */
	Va = v.Ubeta/t_voltage;
	Vb = temp_sv1 + temp_sv2;
	Vc = temp_sv1 - temp_sv2;
/* Sector 0: this is special case for (Ualpha,Ubeta) = (0,0)*/

switch(Sector)
{
	case 0:
	v.Ta = 0.5;
	v.Tb = 0.5;
	v.Tc = 0.5;
	break;
	case 1:   /*Sector 1: t1=Z and t2=Y (abc ---> Tb,Ta,Tc)*/
	t1 = Vc;
	t2 = Vb;
	v.Tb=(1-t1-t2)/2;
	v.Ta = v.Tb+t1;			 	/* taon = tbon+t1		*/
	v.Tc = v.Ta+t2;			  	/* tcon = taon+t2		*/
	break;
	case 2:	  /* Sector 2: t1=Y and t2=-X (abc ---> Ta,Tc,Tb)*/
	t1 = Vb;
	t2 = -Va;
	v.Ta=(1-t1-t2)/2;
	v.Tc = v.Ta+t1;				/*  tcon = taon+t1		*/
	v.Tb = v.Tc+t2;				/*  tbon = tcon+t2		*/
	break;
	case 3:	  /* Sector 3: t1=-Z and t2=X (abc ---> Ta,Tb,Tc)*/
	t1 = -Vc;
	t2 = Va;
	v.Ta=(1-t1-t2)/2;
	v.Tb = v.Ta+t1;				/*	tbon = taon+t1		*/
	v.Tc = v.Tb+t2;				/*	tcon = tbon+t2		*/
	break;
	case 4:	  /* Sector 4: t1=-X and t2=Z (abc ---> Tc,Tb,Ta)*/
	t1 = -Va;
	t2 = Vc;
	v.Tc=(1-t1-t2)/2;
	v.Tb = v.Tc+t1;				/*	tbon = tcon+t1		*/
	v.Ta = v.Tb+t2;				/*	taon = tbon+t2		*/
	break;
	case 5:	  /* Sector 5: t1=X and t2=-Y (abc ---> Tb,Tc,Ta)*/
	t1 = Va;
	t2 = -Vb;					/*	tbon = (1-t1-t2)/2	*/
	v.Tb=(1-t1-t2)/2;
	v.Tc = v.Tb+t1;				/*	taon = tcon+t2		*/
	v.Ta = v.Tc+t2;
	break;
	case 6:	  /* Sector 6: t1=-Y and t2=-Z (abc ---> Tc,Ta,Tb)*/
	t1 = -Vb;
	t2 = -Vc;
	v.Tc=(1-t1-t2)/2;
	v.Ta = v.Tc+t1;				/*	taon = tcon+t1		*/
	v.Tb = v.Ta+t2;				/*	tbon = taon+t2		*/
	break;
}
    v.Ta=1-v.Ta;
    v.Tb=1-v.Tb;
    v.Tc=1-v.Tc;
    // ----------------------------SVGEN防止饱和--- Convert the unsigned GLOBAL_Q format (ranged (0,1))
        if (v.Ta>MAX_PWM_LIMATATION )
            v.Ta=MAX_PWM_LIMATATION ;
        if (v.Tb>MAX_PWM_LIMATATION )
            v.Tb=MAX_PWM_LIMATATION ;
        if (v.Tc>MAX_PWM_LIMATATION )
            v.Tc=MAX_PWM_LIMATATION ;
        if (v.Ta<MIN_PWM_LIMATATION)
            v.Ta=MIN_PWM_LIMATATION;
        if (v.Tb<MIN_PWM_LIMATATION)
            v.Tb=MIN_PWM_LIMATATION;
        if (v.Tc<MIN_PWM_LIMATATION)
            v.Tc=MIN_PWM_LIMATATION;
    (*ptrV)=v;
		}



