//----------------------------------------------------------------------------------
//	FILE:			F2837x_QEP_module.h
//
//	Description:	Contains QEP macros
//
//	Version: 		1.0
//
//  Target:  		TMS320F28377D,
//
//----------------------------------------------------------------------------------
//  Copyright Texas Instruments 2004-2015
//----------------------------------------------------------------------------------
//  Revision History:
//----------------------------------------------------------------------------------
//  Date	   | Description / Status
//----------------------------------------------------------------------------------
//  4 Nov 2015 - QEP macros
//----------------------------------------------------------------------------------

#ifndef __F2837X_QEP_H__
#define __F2837X_QEP_H__

#include "f2837xbmsk.h"
#define POLES   1  //旋转变压器的极对数与电机极对数都是4，故采用1.
/*-----------------------------------------------------------------------------
    Initialization states for EQEP Decode Control Register
------------------------------------------------------------------------------*/
#define QDECCTL_INIT_STATE     ( XCR_X2 + QSRC_QUAD_MODE + SWAP_ENABLE )

/*-----------------------------------------------------------------------------
    Initialization states for EQEP Control Register
------------------------------------------------------------------------------*/
#define QEPCTL_INIT_STATE      ( QEP_EMULATION_FREE + \
                                 PCRM_INDEX + \
                                 IEI_RISING + \
                                 IEL_RISING + \
                                 QPEN_ENABLE + \
                                 QCLM_TIME_OUT + \
                                 UTE_ENABLE )  

/*-----------------------------------------------------------------------------
    Initialization states for EQEP Position-Compare Control Register
------------------------------------------------------------------------------*/
#define QPOSCTL_INIT_STATE      PCE_DISABLE 

/*-----------------------------------------------------------------------------
    Initialization states for EQEP Capture Control Register
------------------------------------------------------------------------------*/
#define QCAPCTL_INIT_STATE     ( UPPS_X1 + \
                                 CCPS_X128 + \
                                 CEN_ENABLE )

#endif // __F2837X_QEP_H__




