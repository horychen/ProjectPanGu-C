//###########################################################################
//
// FILE:   F2837xD_Spi.c
//
// TITLE:  F2837xD SPI Initialization & Support Functions.
//
//###########################################################################
// $TI Release: F2837xD Support Library v210 $
// $Release Date: Tue Nov  1 14:46:15 CDT 2016 $
// $Copyright: Copyright (C) 2013-2016 Texas Instruments Incorporated -
//             http://www.ti.com/ ALL RIGHTS RESERVED $
//###########################################################################

//
// Included Files
//
#include "All_Definition.h"

//
// Calculate BRR: 7-bit baud rate register value
// SPI CLK freq = 500 kHz
// LSPCLK freq  = CPU freq / 4  (by default)
// BRR          = (LSPCLK freq / SPI CLK freq) - 1
//
#if CPU_FRQ_200MHZ
//#define SPI_BRR        ((200E6 / 4) / 500E3) - 1 // 用这个BRR也能跑，就是SPI输出的频率就是降低了大概十倍吧……
#define SPI_BRR        ((200E6 / 4) / 5E6) - 1 // 12.5 MHz for high speed SPI
#endif

#if CPU_FRQ_150MHZ
#define SPI_BRR        ((150E6 / 4) / 500E3) - 1
#endif

#if CPU_FRQ_120MHZ
#define SPI_BRR        ((120E6 / 4) / 500E3) - 1
#endif

//
// InitSPI - This function initializes the SPI to a known state
//
void InitSpi(void)
{
    // GPIO配置
    //InitSpiaGpio();

    // 上电复位后，SPI工作在标准模式下，禁止SPI FIFO功能
    // Initialize SPI FIFO registers
    //SpiaRegs.SPIFFTX.all=0xE040;
    //SpiaRegs.SPIFFRX.all=0x204f;
    //SpiaRegs.SPIFFTX.all=0xe021;      // Enable FIFO's, set TX FIFO level to 1
    //SpiaRegs.SPIFFRX.all=0x0;//0x0021;      // Set RX FIFO level to 8
    //SpiaRegs.SPIFFCT.all=0x0;

    // SPI 寄存器配置

    //SpiaRegs.SPICCR.all =0x000F;    // Reset on, output at rising edge, 16-bit char bits
    // Set reset low before configuration changes
    // Clock polarity (0 == rising, 1 == falling)
    // 16-bit character
    // Disnable loop-back
    SpiaRegs.SPICCR.bit.SPISWRESET = 0;
    SpiaRegs.SPICCR.bit.CLKPOLARITY = 0;
    SpiaRegs.SPICCR.bit.SPICHAR = (16-1);
    SpiaRegs.SPICCR.bit.SPILBK = 1; // loop back


    //SpicRegs.SPICTL.all =0x0006;    // CLOCK PHASE=0, Master Mode, enable talk, and SPI int disabled.
    // Enable master (0 == slave, 1 == master)
    // Enable transmission (Talk)
    // Clock phase (0 == normal, 1 == delayed)
    // SPI interrupts are disabled
    SpiaRegs.SPICTL.bit.MASTER_SLAVE = 1;
    SpiaRegs.SPICTL.bit.TALK = 1;
    SpiaRegs.SPICTL.bit.CLK_PHASE = 0;
    SpiaRegs.SPICTL.bit.SPIINTENA = 0;

    // Set the baud rate
    SpiaRegs.SPIBRR.bit.SPI_BIT_RATE = SPI_BRR;
    //SpiaRegs.SPIBRR = 0x1;           // SPI Baud Rate = LSPCLK/(SPIBRR+1), 根据书上公式，LSPCLK=37.5MHz so=37.5/4=9.375MHz

    SpiaRegs.SPICCR.all = 0x008F;    // 在改变设置前将RESET清零，并在设置结束后将其置位

    // Set FREE bit
    // Halting on a breakpoint will not halt the SPI
    SpiaRegs.SPIPRI.bit.FREE = 1;   // breakpoints don't disturb xmission

    //GpioCtrlRegs.GPBMUX2.bit.GPIO57 = 0; // Configure GPIO57 as C\S\ signal for MAX5307
    // 唤醒MAX5307
    GpioDataRegs.GPBSET.bit.GPIO61 = 1;             //cs=1
    NOP;
    NOP;
    GpioDataRegs.GPBCLEAR.bit.GPIO61 = 1;           //cs=0

    SpiaRegs.SPITXBUF=0xfffc;                       //MAX5307唤醒字符
    while(SpiaRegs.SPISTS.bit.INT_FLAG!=1){NOP;}    // 数据传完后INT_FLAG会置位

    GpioDataRegs.GPBSET.bit.GPIO61 = 1;             //cs=1为下一次做准备

    SpiaRegs.SPICCR.bit.SPISWRESET=0;               //通过reset 清楚SPI中断标志INT_FLAG
    NOP;
    NOP;
    // Release the SPI from reset
    SpiaRegs.SPICCR.bit.SPISWRESET=1;               // Relinquish SPI from Reset

}

//
// InitSpiGpio - This function initializes GPIO pins to function as SPI pins.
//               Each GPIO pin can be configured as a GPIO pin or up to 3
//               different peripheral functional pins. By default all pins come
//               up as GPIO inputs after reset.
//
//               Caution:
//               For each SPI peripheral
//               Only one GPIO pin should be enabled for SPISOMO operation.
//               Only one GPIO pin should be enabled for SPISOMI operation.
//               Only one GPIO pin should be enabled for SPICLK  operation.
//               Only one GPIO pin should be enabled for SPISTE  operation.
//               Comment out other unwanted lines.
//
//void InitSpiGpio()
//{
//   InitSpicGpio();
//}

void InitSpicGpio(){

    EALLOW;

    GpioCtrlRegs.GPDPUD.bit.GPIO100 = 0;  // Enable pull-up on GPIO54 (SPISIMOA)
    GpioCtrlRegs.GPDQSEL1.bit.GPIO100 = 3;  // Asynch input GPIO54 (SPISIMOA)
    GpioCtrlRegs.GPDMUX1.bit.GPIO100 = 1; // Configure GPIO54 as SPISIMOA

    //    GPIO_SetupPinOptions(56, GPIO_PULLUP, GPIO_ASYNC);
    //    GPIO_SetupPinMux(56,GPIO_MUX_CPU2,0);
    GpioCtrlRegs.GPDPUD.bit.GPIO102 = 0;  // Enable pull-up on GPIO56 (SPICLKA)
    GpioCtrlRegs.GPDQSEL1.bit.GPIO102 = 3; // Asynch input GPIO55 (SPICLKA)
    GpioCtrlRegs.GPDMUX1.bit.GPIO102 = 1; // Configure GPIO56 as SPICLKA

    // SPISTEA for SPI with D/A chip MAX5307
    GPIO_SetupPinOptions(103, GPIO_OUTPUT, GPIO_SYNC);
    #if NUMBER_OF_DSP_CORES == 1
        GPIO_SetupPinMux(103,GPIO_MUX_CPU1,0);
    #else
        GPIO_SetupPinMux(103,GPIO_MUX_CPU2,0);
    #endif

    EDIS;
}

//
// InitSpiaGpio - Initialize SPIA GPIOs
//
void InitSpiaGpio()
{
   EALLOW;

    //
    // Enable internal pull-up for the selected pins
    //
    // Pull-ups can be enabled or disabled by the user.
    // This will enable the pullups for the specified pins.
    // Comment out other unwanted lines.
    //
    //GpioCtrlRegs.GPAPUD.bit.GPIO16 = 0;  // Enable pull-up on GPIO16 (SPISIMOA)
    //GpioCtrlRegs.GPAPUD.bit.GPIO17 = 0;  // Enable pull-up on GPIO17 (SPISOMIA)
    //GpioCtrlRegs.GPAPUD.bit.GPIO18 = 0;  // Enable pull-up on GPIO18 (SPICLKA)
    //GpioCtrlRegs.GPAPUD.bit.GPIO19 = 0;  // Enable pull-up on GPIO19 (SPISTEA)




//    GPIO_SetupPinOptions(54, GPIO_PULLUP, GPIO_ASYNC);
//    GPIO_SetupPinMux(54,GPIO_MUX_CPU2,0);
    GpioCtrlRegs.GPBPUD.bit.GPIO58 = 0;  // Enable pull-up on GPIO54 (SPISIMOA)
    GpioCtrlRegs.GPBQSEL2.bit.GPIO58 = 3;  // Asynch input GPIO54 (SPISIMOA)
    GpioCtrlRegs.GPBMUX2.bit.GPIO58 = 1; // Configure GPIO54 as SPISIMOA

    //GpioCtrlRegs.GPBPUD.bit.GPIO59 = 0;  // Enable pull-up on GPIO55 (SPISOMIA)
    //GpioCtrlRegs.GPBQSEL2.bit.GPIO59 = 3;  // Asynch input GPIO55 (SPISOMIA)
    //GpioCtrlRegs.GPBMUX2.bit.GPIO59 = 1; // Configure GPIO55 as SPISOMIA

//    GPIO_SetupPinOptions(56, GPIO_PULLUP, GPIO_ASYNC);
//    GPIO_SetupPinMux(56,GPIO_MUX_CPU2,0);
    GpioCtrlRegs.GPBPUD.bit.GPIO60 = 0;  // Enable pull-up on GPIO56 (SPICLKA)
    GpioCtrlRegs.GPBQSEL2.bit.GPIO60 = 3; // Asynch input GPIO55 (SPICLKA)
    GpioCtrlRegs.GPBMUX2.bit.GPIO60 = 1; // Configure GPIO56 as SPICLKA

    // SPISTEA for SPI with D/A chip MAX5307
    GPIO_SetupPinOptions(61, GPIO_OUTPUT, GPIO_SYNC);
    #if NUMBER_OF_DSP_CORES == 1
        GPIO_SetupPinMux(61,GPIO_MUX_CPU1,0);
    #else
        GPIO_SetupPinMux(61,GPIO_MUX_CPU2,0);
    #endif
    //GpioCtrlRegs.GPBPUD.bit.GPIO57 = 0;  // Enable pull-up on GPIO57 (SPISTEA)
    //GpioCtrlRegs.GPBQSEL2.bit.GPIO57 = 3; // Asynch input GPIO56 (SPISTEA)
    //GpioCtrlRegs.GPBMUX2.bit.GPIO57 = 1; // Configure GPIO57 as SPISTEA




    //
    // Set qualification for selected pins to asynch only
    //
    // This will select asynch (no qualification) for the selected pins.
    // Comment out other unwanted lines.
    //


    //
    //Configure SPI-A pins using GPIO regs
    //
    // This specifies which of the possible GPIO pins will be SPI functional
    // pins.
    // Comment out other unwanted lines.
    //
    //GpioCtrlRegs.GPAMUX2.bit.GPIO16 = 1; // Configure GPIO16 as SPISIMOA
    //GpioCtrlRegs.GPAMUX2.bit.GPIO17 = 1; // Configure GPIO17 as SPISOMIA
    //GpioCtrlRegs.GPAMUX2.bit.GPIO18 = 1; // Configure GPIO18 as SPICLKA
    //GpioCtrlRegs.GPAMUX2.bit.GPIO19 = 1; // Configure GPIO19 as SPISTEA


    EDIS;
}

void InitHighSpeedSpiGpio()
{

   EALLOW;

/* Enable internal pull-up for the selected pins */
// Pull-ups can be enabled or disabled by the user.
// This will enable the pullups for the specified pins.
// Comment out other unwanted lines.

    GpioCtrlRegs.GPBPUD.bit.GPIO58 = 0;   // Enable pull-up on GPIO16 (SPISIMOA)
    GpioCtrlRegs.GPBPUD.bit.GPIO59 = 0;   // Enable pull-up on GPIO17 (SPISOMIA)
    GpioCtrlRegs.GPBPUD.bit.GPIO60 = 0;   // Enable pull-up on GPIO18 (SPICLKA)
    //GpioCtrlRegs.GPBPUD.bit.GPIO61 = 0;   // Enable pull-up on GPIO19 (SPISTEA) // 与MAX5307通讯，不需要真正意义上的 SPISTEA 功能，GPIO61作为普通的GPIO使用即可，需要把GPIO61的使用权限授权给CPU02，具体见函数：void Gpio_initialize(void)

    GpioCtrlRegs.GPBPUD.bit.GPIO63 = 0;   // Enable pull-up on GPIO63 (SPISIMOB)
    GpioCtrlRegs.GPCPUD.bit.GPIO64 = 0;   // Enable pull-up on GPIO64 (SPISOMIB)
    GpioCtrlRegs.GPCPUD.bit.GPIO65 = 0;   // Enable pull-up on GPIO65 (SPICLKB)
    GpioCtrlRegs.GPCPUD.bit.GPIO66 = 0;   // Enable pull-up on GPIO66 (SPISTEB)

    GpioCtrlRegs.GPCPUD.bit.GPIO69 = 0;   // Enable pull-up on GPIO69 (SPISIMOC)
    GpioCtrlRegs.GPCPUD.bit.GPIO70 = 0;   // Enable pull-up on GPIO70 (SPISOMIC)
    GpioCtrlRegs.GPCPUD.bit.GPIO71 = 0;   // Enable pull-up on GPIO71 (SPICLKC)
    GpioCtrlRegs.GPCPUD.bit.GPIO72 = 0;   // Enable pull-up on GPIO72 (SPISTEC)

/* Set qualification for selected pins to asynch only */
// This will select asynch (no qualification) for the selected pins.
// Comment out other unwanted lines.

    GpioCtrlRegs.GPBQSEL2.bit.GPIO58 = 3; // Asynch input GPIO16 (SPISIMOA)
    GpioCtrlRegs.GPBQSEL2.bit.GPIO59 = 3; // Asynch input GPIO17 (SPISOMIA)
    GpioCtrlRegs.GPBQSEL2.bit.GPIO60 = 3; // Asynch input GPIO18 (SPICLKA)
    //GpioCtrlRegs.GPBQSEL2.bit.GPIO61 = 3; // Asynch input GPIO19 (SPISTEA) // 与MAX5307通讯，不需要真正意义上的 SPISTEA 功能，GPIO61作为普通的GPIO使用即可，需要把GPIO61的使用权限授权给CPU02，具体见函数：void Gpio_initialize(void)

    GpioCtrlRegs.GPBQSEL2.bit.GPIO63 = 3; // Asynch input GPIO63 (SPISIMOB)
    GpioCtrlRegs.GPCQSEL1.bit.GPIO64 = 3; // Asynch input GPIO64 (SPISOMIB)
    GpioCtrlRegs.GPCQSEL1.bit.GPIO65 = 3; // Asynch input GPIO65 (SPICLKB)
    GpioCtrlRegs.GPCQSEL1.bit.GPIO66 = 3; // Asynch input GPIO66 (SPISTEB)

    GpioCtrlRegs.GPCQSEL1.bit.GPIO69 = 3; // Asynch input GPIO69 (SPISIMOCC)
    GpioCtrlRegs.GPCQSEL1.bit.GPIO70 = 3; // Asynch input GPIO70 (SPISOMIC)
    GpioCtrlRegs.GPCQSEL1.bit.GPIO71 = 3; // Asynch input GPIO71 (SPICLKC)
    GpioCtrlRegs.GPCQSEL1.bit.GPIO72 = 3; // Asynch input GPIO72 (SPISTEC)

/* Configure SPI-A pins using GPIO regs*/
// This specifies which of the possible GPIO pins will be SPI functional pins.
// Comment out other unwanted lines.


    GpioCtrlRegs.GPBGMUX2.bit.GPIO58 = 3;
    GpioCtrlRegs.GPBGMUX2.bit.GPIO59 = 3;
    GpioCtrlRegs.GPBGMUX2.bit.GPIO60 = 3;
    //GpioCtrlRegs.GPBGMUX2.bit.GPIO61 = 3;
    GpioCtrlRegs.GPBMUX2.bit.GPIO58 = 3; // Configure GPIO16 as SPISIMOA
    GpioCtrlRegs.GPBMUX2.bit.GPIO59 = 3; // Configure GPIO17 as SPISOMIA
    GpioCtrlRegs.GPBMUX2.bit.GPIO60 = 3; // Configure GPIO18 as SPICLKA
    //GpioCtrlRegs.GPBMUX2.bit.GPIO61 = 3; // Configure GPIO19 as SPISTEA // 与MAX5307通讯，不需要真正意义上的 SPISTEA 功能，GPIO61作为普通的GPIO使用即可，需要把GPIO61的使用权限授权给CPU02，具体见函数：void Gpio_initialize(void)

    GpioCtrlRegs.GPBGMUX2.bit.GPIO63 = 3;
    GpioCtrlRegs.GPCGMUX1.bit.GPIO64 = 3;
    GpioCtrlRegs.GPCGMUX1.bit.GPIO65 = 3;
    GpioCtrlRegs.GPCGMUX1.bit.GPIO66 = 3;
    GpioCtrlRegs.GPBMUX2.bit.GPIO63 = 3; // Configure GPIO63 as SPISIMOB
    GpioCtrlRegs.GPCMUX1.bit.GPIO64 = 3; // Configure GPIO64 as SPISOMIB
    GpioCtrlRegs.GPCMUX1.bit.GPIO65 = 3; // Configure GPIO65 as SPICLKB
    GpioCtrlRegs.GPCMUX1.bit.GPIO66 = 3; // Configure GPIO66 as SPISTEB

    GpioCtrlRegs.GPCGMUX1.bit.GPIO69 = 3;
    GpioCtrlRegs.GPCGMUX1.bit.GPIO70 = 3;
    GpioCtrlRegs.GPCGMUX1.bit.GPIO71 = 3;
    GpioCtrlRegs.GPCGMUX1.bit.GPIO72 = 3;
    GpioCtrlRegs.GPCMUX1.bit.GPIO69 = 3; // Configure GPIO69 as SPISIMOC
    GpioCtrlRegs.GPCMUX1.bit.GPIO70 = 3; // Configure GPIO70 as SPISOMIC
    GpioCtrlRegs.GPCMUX1.bit.GPIO71 = 3; // Configure GPIO71 as SPICLKC
    GpioCtrlRegs.GPCMUX1.bit.GPIO72 = 3; // Configure GPIO72 as SPISTEC


    EDIS;
}

//
// End of file
//
