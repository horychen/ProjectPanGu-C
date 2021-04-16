//###########################################################################
//
// FILE:    F2837xD_spi.h
//
// TITLE:   F2837xD Device SPI Register Definitions.
//
//###########################################################################
// $TI Release: F2837xD Support Library v210 $
// $Release Date: Tue Nov  1 14:46:15 CDT 2016 $
// $Copyright: Copyright (C) 2013-2016 Texas Instruments Incorporated -
//             http://www.ti.com/ ALL RIGHTS RESERVED $
//###########################################################################

#ifndef __F2837xD_SPI_H__
#define __F2837xD_SPI_H__

#ifdef __cplusplus
extern "C" {
#endif


//---------------------------------------------------------------------------
// SPI Individual Register Bit Definitions:

struct SPICCR_BITS {                    // bits description
    Uint16 SPICHAR:4;                   // 3:0 Character Length Control
    Uint16 SPILBK:1;                    // 4 SPI Loopback
    Uint16 HS_MODE:1;                   // 5 High Speed mode control
    Uint16 CLKPOLARITY:1;               // 6 Shift Clock Polarity
    Uint16 SPISWRESET:1;                // 7 SPI Software Reset
    Uint16 rsvd1:8;                     // 15:8 Reserved
};

union SPICCR_REG {
    Uint16  all;
    struct  SPICCR_BITS  bit;
};

struct SPICTL_BITS {                    // bits description
    Uint16 SPIINTENA:1;                 // 0 SPI Interupt Enable
    Uint16 TALK:1;                      // 1 Master/Slave Transmit Enable
    Uint16 MASTER_SLAVE:1;              // 2 SPI Network Mode Control
    Uint16 CLK_PHASE:1;                 // 3 SPI Clock Phase
    Uint16 OVERRUNINTENA:1;             // 4 Overrun Interrupt Enable
    Uint16 rsvd1:11;                    // 15:5 Reserved
};

union SPICTL_REG {
    Uint16  all;
    struct  SPICTL_BITS  bit;
};

struct SPISTS_BITS {                    // bits description
    Uint16 rsvd1:5;                     // 4:0 Reserved
    Uint16 BUFFULL_FLAG:1;              // 5 SPI Transmit Buffer Full Flag
    Uint16 INT_FLAG:1;                  // 6 SPI Interrupt Flag
    Uint16 OVERRUN_FLAG:1;              // 7 SPI Receiver Overrun Flag
    Uint16 rsvd2:8;                     // 15:8 Reserved
};

union SPISTS_REG {
    Uint16  all;
    struct  SPISTS_BITS  bit;
};

struct SPIBRR_BITS {                    // bits description
    Uint16 SPI_BIT_RATE:7;              // 6:0 SPI Bit Rate Control
    Uint16 rsvd1:9;                     // 15:7 Reserved
};

union SPIBRR_REG {
    Uint16  all;
    struct  SPIBRR_BITS  bit;
};

struct SPIFFTX_BITS {                   // bits description
    Uint16 TXFFIL:5;                    // 4:0 TXFIFO Interrupt Level
    Uint16 TXFFIENA:1;                  // 5 TXFIFO Interrupt Enable
    Uint16 TXFFINTCLR:1;                // 6 TXFIFO Interrupt Clear
    Uint16 TXFFINT:1;                   // 7 TXFIFO Interrupt Flag
    Uint16 TXFFST:5;                    // 12:8 Transmit FIFO Status
    Uint16 TXFIFO:1;                    // 13 TXFIFO Reset
    Uint16 SPIFFENA:1;                  // 14 FIFO Enhancements Enable
    Uint16 SPIRST:1;                    // 15 SPI Reset
};

union SPIFFTX_REG {
    Uint16  all;
    struct  SPIFFTX_BITS  bit;
};

struct SPIFFRX_BITS {                   // bits description
    Uint16 RXFFIL:5;                    // 4:0 RXFIFO Interrupt Level
    Uint16 RXFFIENA:1;                  // 5 RXFIFO Interrupt Enable
    Uint16 RXFFINTCLR:1;                // 6 RXFIFO Interupt Clear
    Uint16 RXFFINT:1;                   // 7 RXFIFO Interrupt Flag
    Uint16 RXFFST:5;                    // 12:8 Receive FIFO Status
    Uint16 RXFIFORESET:1;               // 13 RXFIFO Reset
    Uint16 RXFFOVFCLR:1;                // 14 Receive FIFO Overflow Clear
    Uint16 RXFFOVF:1;                   // 15 Receive FIFO Overflow Flag
};

union SPIFFRX_REG {
    Uint16  all;
    struct  SPIFFRX_BITS  bit;
};

struct SPIFFCT_BITS {                   // bits description
    Uint16 TXDLY:8;                     // 7:0 FIFO Transmit Delay Bits
    Uint16 rsvd1:8;                     // 15:8 Reserved
};

union SPIFFCT_REG {
    Uint16  all;
    struct  SPIFFCT_BITS  bit;
};

struct SPIPRI_BITS {                    // bits description
    Uint16 TRIWIRE:1;                   // 0 3-wire mode select bit
    Uint16 STEINV:1;                    // 1 SPISTE inversion bit
    Uint16 rsvd1:2;                     // 3:2 Reserved
    Uint16 FREE:1;                      // 4 Free emulation mode
    Uint16 SOFT:1;                      // 5 Soft emulation mode
    Uint16 rsvd2:1;                     // 6 Reserved
    Uint16 rsvd3:9;                     // 15:7 Reserved
};

union SPIPRI_REG {
    Uint16  all;
    struct  SPIPRI_BITS  bit;
};

struct SPI_REGS {
    union   SPICCR_REG                       SPICCR;                       // SPI Configuration Control Register
    union   SPICTL_REG                       SPICTL;                       // SPI Operation Control Register
    union   SPISTS_REG                       SPISTS;                       // SPI Status Register
    Uint16                                   rsvd1;                        // Reserved
    union   SPIBRR_REG                       SPIBRR;                       // SPI Baud Rate Register
    Uint16                                   rsvd2;                        // Reserved
    Uint16                                   SPIRXEMU;                     // SPI Emulation Buffer Register
    Uint16                                   SPIRXBUF;                     // SPI Serial Input Buffer Register
    Uint16                                   SPITXBUF;                     // SPI Serial Output Buffer Register
    Uint16                                   SPIDAT;                       // SPI Serial Data Register
    union   SPIFFTX_REG                      SPIFFTX;                      // SPI FIFO Transmit Register
    union   SPIFFRX_REG                      SPIFFRX;                      // SPI FIFO Receive Register
    union   SPIFFCT_REG                      SPIFFCT;                      // SPI FIFO Control Register
    Uint16                                   rsvd3[2];                     // Reserved
    union   SPIPRI_REG                       SPIPRI;                       // SPI Priority Control Register
};

//---------------------------------------------------------------------------
// SPI External References & Function Declarations:
//
#ifdef CPU1
extern volatile struct SPI_REGS SpiaRegs;
extern volatile struct SPI_REGS SpibRegs;
extern volatile struct SPI_REGS SpicRegs;
#endif
#ifdef CPU2
extern volatile struct SPI_REGS SpiaRegs;
extern volatile struct SPI_REGS SpibRegs;
extern volatile struct SPI_REGS SpicRegs;
#endif
#ifdef __cplusplus
}
#endif                                  /* extern "C" */

#endif

//===========================================================================
// End of file.
//===========================================================================
