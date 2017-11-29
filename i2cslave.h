/*
--------------------------------------------------------------------------------
 Description  : TWI/I2c slave, interrupt driven
 Target       : Atmel ATTiny2313/4313
--------------------------------------------------------------------------------
*/

#ifndef _I2CSLAVE_H_
#define _I2CSLAVE_H_

#include <avr/io.h>

/* buffer sizes */
#define TWI_RX_BUFFER_SIZE   (12)
#define TWI_TX_BUFFER_SIZE   (20)

//********** USI_TWI Prototypes **********//
 /* initialization, should called once */
void  USI_TWI_Slave_Initialize(void);

//********** Global Variables **********//
 /* I/O buffers */
extern volatile unsigned char TWI_RxBuf[TWI_RX_BUFFER_SIZE];
extern volatile unsigned char TWI_TxBuf[TWI_TX_BUFFER_SIZE];
 /* current index on buffers */
extern volatile unsigned char TWI_rxBufIdx;
extern volatile unsigned char TWI_txBufIdx;

/* set to true when first byte received, should be externally reset */
extern volatile unsigned char bNewIncomingFrame;
/* set to true when new byte received, should be externally reset */
extern volatile unsigned char bNewIncomingByte;

/* managed I2c address */
extern unsigned char TWI_slaveAddress;

#endif
