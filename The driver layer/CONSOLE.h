/*
 * CONSOLE.h
 *
 *  Created on: Jun 10, 2021
 *      Author: horyc
 */

#ifndef THE_DRIVER_LAYER_CONSOLE_H_
#define THE_DRIVER_LAYER_CONSOLE_H_

extern Uint16 LoopCount;
extern Uint16 ErrorCount;
extern Uint16 SendChar;
extern Uint16 ReceivedChar;


void sci_poll(int16 ch);

#endif /* THE_DRIVER_LAYER_CONSOLE_H_ */
