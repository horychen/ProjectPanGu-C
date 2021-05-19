/*
 * yxsvgen.c
 *
 *  Created on: 2021年1月11日
 *      Author: xinyuan
 */
#include <All_Definition.h>

void SVGEN_Drive(SVGENDQ* ptrV){

    float Va,Vb,Vc,t1,t2,temp_sv1,temp_sv2,t_voltage;
    int Sector = 0;  // Sector is treated as Q0 - independently with global Q
    SVGENDQ  v;
    v=(*ptrV);
    Sector = 0;
    t_voltage=G.Voltage_DC_BUS*0.577367;
    temp_sv1=(v.Ubeta*0.8660254)/G.Voltage_DC_BUS; 					/*divide by 2*/
    temp_sv2=(1.5*v.Ualpha)/G.Voltage_DC_BUS;	/* 0.8660254 = sqrt(3)/2*/

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








//void SPWM_Drive(SVGENDQ* ptrV){
//
//    //    // outputs are inv.ual_comp, inv.u_comp[3]
//    //    Modified_ParkSul_Compensation(); // inverter nonlinearity compensation
//    //    // apply conpensation to the comanded voltage signals in al-be frame
//    //    CTRL.ual = CTRL.ual + inv.ual_comp*0;
//    //    CTRL.ube = CTRL.ube + inv.ube_comp*0;
//
//    /* 调制比 这里以后检查一下，调制比应该用乘以这个根号三分之二吗？显然要的，这里的CTRL.VCurPerUnit == m_a，见徐德鸿《电力电子技术》@p164 */
//    CTRL.VCurPerUnit = SQRT_2_SLASH_3 * sqrt(CTRL.ual*CTRL.ual+CTRL.ube*CTRL.ube) *2.0 *CTRL.lUdc;
//    //防止过调制
//    if(CTRL.VCurPerUnit > 1){
//        float temp = 1.0/CTRL.VCurPerUnit;
//        CTRL.pi_iMs.i_state *= temp;
//        CTRL.pi_iTs.i_state *= temp;
//        CTRL.uMs_cmd *= temp;
//        CTRL.uTs_cmd *= temp;
//        CTRL.ual *= temp;
//        CTRL.ube *= temp;
//        /*
//        \[\begin{array}{l}
//        {\mathop{\rm suppose}\nolimits} \;ma = \sqrt {\frac{2}{3}} \sqrt {u_{\alpha s}^2 + u_{\beta s}^2}  > 1\\
//        let\;\sqrt {\frac{2}{3}} \sqrt {{{\left( {c{u_{\alpha s}}} \right)}^2} + {{\left( {c{u_{\beta s}}} \right)}^2}}  = 1\\
//         \Rightarrow \frac{1}{c} = \sqrt {\frac{2}{3}} \sqrt {{{\left( {{u_{\alpha s}}} \right)}^2} + {{\left( {{u_{\beta s}}} \right)}^2}}  = ma
//        \end{array}\]
//         */
//        CTRL.VCurPerUnit = 1;
//        CTRL.OverModulation = TRUE;
//    }
//    else{
//        CTRL.OverModulation = FALSE;
//    }
//
//    /*
//     * 恒幅值变换变成ABC三相时的系数是1，此时变换阵的逆是变换阵的转置乘以3/2。
//     * 但是我们用的是恒功率变换，变换阵的逆就是变换阵的转置。
//     * */
//    // Inverse Clarke Trans. 2to3 into phase axes
//    CTRL.VaPU = SQRT_2_SLASH_3 * (       CTRL.ual                                   )* 2.0 * CTRL.lUdc;
//    CTRL.VbPU = SQRT_2_SLASH_3 * (-0.5 * CTRL.ual - SIN_DASH_2PI_SLASH_3 * CTRL.ube )* 2.0 * CTRL.lUdc;
//    CTRL.VcPU = SQRT_2_SLASH_3 * (-0.5 * CTRL.ual - SIN_2PI_SLASH_3      * CTRL.ube )* 2.0 * CTRL.lUdc;
//}

//    // 考虑母线电压可能的限幅，需要重新计算电压。注意，需要用死区补偿作用之前的Ta，Tb和Tc，因为使用Ta，Tb，Tc计算电压是不考虑死区的，所以也不能考虑死区补偿。
//    {
//        volt_calc1.MfuncV1=CTRL.svgen1.Ta;
//        volt_calc1.MfuncV2=CTRL.svgen1.Tb;
//        volt_calc1.MfuncV3=CTRL.svgen1.Tc;
//        volt_calc1.DcBusVolt=Voltage_DC_BUS;
//        PHASEVOLT_MACRO(volt_calc1);
//        volt_calc1.Valpha;
//        volt_calc1.Vbeta;
//    }
