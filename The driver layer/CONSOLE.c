/*
 * SCI_232.c
 *
 *  Created on: 2021年5月15日
 *      Author: yuanx
 */
#include <All_Definition.h>
int runstop;
int Sci_direction;
int Sci_iq;
int Sci_id;
float pi_kp;
float pi_ki;
int Motor_AD;
int Motor_AD_2;
int ppi_kp;
int ppi_ki;
int nnn;
Uint16 sciRX_dataA[8];
Uint16 sciTX_dataA[8];
int Temp_MOSFET;
interrupt void scicRxFifoIsr1(void);
void sciTxFifoIsr(void);
void sciRxFifoIsr(void);
void sci_init(void);
int SCI_number=0;
interrupt void scicRxFifoIsr1(void){
    int i=0;
    for(i=0;i<8;i++){
         sciRX_dataA[i]=ScicRegs.SCIRXBUF.all;
    }
    ScicRegs.SCIFFRX.bit.RXFFINTCLR=1;  // Clear SCI Interrupt flag
    ScicRegs.SCIFFRX.bit.RXFFOVRCLR=1;
    PieCtrlRegs.PIEACK.all|=0x100;      // Issue PIE ACK
}
void sciTxFifoIsr(void){
    Uint16 i;
    if (nnn==1){
        sciTX_dataA[0]=0x5A;
        sciTX_dataA[1]=0xA5;
        sciTX_dataA[2]=0x05;
        sciTX_dataA[3]=runstop;
        sciTX_dataA[4]=Sci_direction;
        sciTX_dataA[5]=0x01;
        sciTX_dataA[6]=Sci_iq%256;
        sciTX_dataA[7]=Sci_iq/256;
    }else if (nnn==2){
        sciTX_dataA[0]=0x5A;
        sciTX_dataA[1]=0xA5;
        sciTX_dataA[2]=0x05;
        sciTX_dataA[3]=Sci_id%256;
        sciTX_dataA[4]=Sci_id/256;
        sciTX_dataA[5]=0x02;
        sciTX_dataA[6]=ppi_kp%256 ;
        sciTX_dataA[7]=ppi_kp/256  ;
    }else if (nnn==3){
        sciTX_dataA[0]=0x5A;
        sciTX_dataA[1]=0xA5;
        sciTX_dataA[2]=0x05;
        sciTX_dataA[3]=ppi_ki%256;
        sciTX_dataA[4]=ppi_ki/256;
        sciTX_dataA[5]=0x03;
        sciTX_dataA[6]=Motor_AD%256;
        sciTX_dataA[7]=Motor_AD/256;
    }else if (nnn==4){
        sciTX_dataA[0]=0x5A;
        sciTX_dataA[1]=0xA5;
        sciTX_dataA[2]=0x05;
        sciTX_dataA[3]=Motor_AD_2%256;
        sciTX_dataA[4]=Motor_AD_2/256;
        sciTX_dataA[5]=0x04;
        sciTX_dataA[6]=Temp_MOSFET%256;
        sciTX_dataA[7]=Temp_MOSFET/256;
    }
    for(i=0; i<8; i++){
        ScicRegs.SCITXBUF.all=sciTX_dataA[i];     // Send data
    }
    DELAY_US(10000);
}

void sciRxFifoIsr(void){
    if (sciRX_dataA[0]==0xFF  &&sciRX_dataA[1]==0xFF &&sciRX_dataA[6]==0xEE  &&sciRX_dataA[7]==0xEE ){
        switch ( sciRX_dataA[2] ) {
            case 0x01   : // 系统启动 0是系统停止工作
            runstop=sciRX_dataA[3];
            break;

            case 0x02   :  // 方向                            iq，id 为正                       FF FF 01 00 00 00  EE EE            iq，id 为负    FF FF 01 01 00 00  EE EE
            Sci_direction =sciRX_dataA[3];
            break;

            case 0x03   :  //Sci_iq         iq值大小 iq=100   FF FF 02 00 00 64  EE EE
            Sci_iq=sciRX_dataA[4]*256;
            Sci_iq=sciRX_dataA[3]+Sci_iq;
            break;

            case 0x04   :  //Sci_id          id值大小 id=100   FF FF 03 00 00 64  EE EE
            Sci_id=sciRX_dataA[4]*256;
            Sci_id=sciRX_dataA[3]+Sci_id;
            break;

            case 0x05   :  // pi_kp           id值大小 id=100   FF FF 03 00 00 64  EE EE
            ppi_kp=sciRX_dataA[4]*256;
            ppi_kp=sciRX_dataA[3]+ppi_kp;
            break;

            case 0x06   :  // pi_ki         FF FF FF 00 01 02  EE EE
            ppi_ki=sciRX_dataA[4]*256;
            ppi_ki=sciRX_dataA[3]+ppi_ki;
            break;
        }
    }
}
