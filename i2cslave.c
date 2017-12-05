/*
--------------------------------------------------------------------------------
  Description  : TWI/I2c slave, fully interrupt driven
  Target       : Atmel ATtiny2313/4313 (principle should work as well on ATmega169, ATtiny26...)
  Compiler     : avr-gcc
  Credit       : inspired by note "AVR312 - Using the USI module as a TWI slave"
--------------------------------------------------------------------------------
*/

#include "i2cslave.h"

#include <avr/interrupt.h>


/* MCU dependent stuff */
#define DDR_USI             DDRB
#define PORT_USI            PORTB
#define PIN_USI             PINB
#define PORT_TWI_SDA        PORTB5
#define PORT_TWI_SCL        PORTB7
#define PIN_TWI_SDA         PINB5
#define PIN_TWI_SCL         PINB7

/* some explicit names for interrupt flags in USISR (Status Register) */
#define START_CONDITION_INT_FLAG         USISIF
#define COUNTER_OVERFLOW_INT_FLAG        USIOIF
#define STOP_CONDITION_INT_FLAG          USIPF
#define DATA_OUTPUT_COLLISION_INT_FLAG   USIDC

/* states of I2C overflow interrupt */
#define TWI_SLAVE_CHECK_ADDRESS                (0x01)
#define TWI_SLAVE_SEND_DATA                    (0x02)
#define TWI_SLAVE_REQUEST_REPLY_FROM_SEND_DATA (0x03)
#define TWI_SLAVE_CHECK_REPLY_FROM_SEND_DATA   (0x04)
#define TWI_SLAVE_REQUEST_DATA                 (0x05)
#define TWI_SLAVE_GET_DATA_AND_SEND_ACK        (0x06)

/* digests to improve readability in overflow interrupt code */
#define SET_TWI_TO_SEND_ACK                                                          \
{                                                                                    \
    USIDR = 0;                    /* Prepare ACK                         */          \
    DDR_USI |= (1<<PORT_TWI_SDA); /* Set SDA as output                   */          \
    USISR = (0<<START_CONDITION_INT_FLAG) | (1<<COUNTER_OVERFLOW_INT_FLAG) |         \
            (1<<STOP_CONDITION_INT_FLAG) | (1<<DATA_OUTPUT_COLLISION_INT_FLAG)       \
            |                /* Clear all flags, except Start Cond  */               \
            (0xe<<USICNT0); /* set USI counter to shift 1 bit */                     \
}

#define SET_TWI_TO_READ_ACK                                                          \
{                                                                                    \
    DDR_USI &= ~(1<<PORT_TWI_SDA); /* Set SDA as input */                            \
    USIDR = 0;                     /* Prepare ACK        */                          \
    USISR = (0<<START_CONDITION_INT_FLAG) | (1<<COUNTER_OVERFLOW_INT_FLAG) |         \
            (1<<STOP_CONDITION_INT_FLAG) | (1<<DATA_OUTPUT_COLLISION_INT_FLAG)       \
            |               /* Clear all flags, except Start Cond  */                \
            (0xe<<USICNT0); /* set USI counter to shift 1 bit */                     \
}

#define SET_TWI_TO_START_CONDITION_MODE                                                                    \
{                                                                                                          \
    DDR_USI &= ~(1<<PORT_TWI_SDA); /* Set SDA as input */                                                  \
    USICR = (1<<USISIE) | (0<<USIOIE) | /* Enable Start Condition Interrupt. Disable Overflow Interrupt */ \
      (1<<USIWM1) | (0<<USIWM0) | /* Set USI in Two-wire mode. No USI Counter overflow hold       */       \
      (1<<USICS1) | (0<<USICS0) | (0<<USICLK)                                                              \
      | /* Shift Register Clock Source = External, positive edge        */                                 \
      (0<<USITC);                                                                                          \
    USISR = (0<<START_CONDITION_INT_FLAG) | (1<<COUNTER_OVERFLOW_INT_FLAG) |                               \
            (1<<STOP_CONDITION_INT_FLAG) | (1<<DATA_OUTPUT_COLLISION_INT_FLAG)                             \
            |                  /* Clear all flags, except Start Cond  */                                   \
            (0x0<<USICNT0);                                                                                \
}

#define SET_TWI_TO_SEND_DATA                                                         \
{                                                                                    \
    DDR_USI |= (1<<PORT_TWI_SDA); /* Set SDA as output                  */           \
    USISR = (0<<START_CONDITION_INT_FLAG) | (1<<COUNTER_OVERFLOW_INT_FLAG) |         \
            (1<<STOP_CONDITION_INT_FLAG) | (1<<DATA_OUTPUT_COLLISION_INT_FLAG)       \
            |               /* Clear all flags, except Start Cond  */                \
            (0x0<<USICNT0); /* set USI to shift out 8 bits        */                 \
}

#define SET_TWI_TO_READ_DATA                                                         \
{                                                                                    \
    DDR_USI &= ~(1<<PORT_TWI_SDA); /* Set SDA as input                   */          \
    USISR = (0<<START_CONDITION_INT_FLAG) | (1<<COUNTER_OVERFLOW_INT_FLAG) |         \
            (1<<STOP_CONDITION_INT_FLAG) | (1<<DATA_OUTPUT_COLLISION_INT_FLAG)       \
            |               /* Clear all flags, except Start Cond  */                \
            (0x0<<USICNT0); /* set USI to shift out 8 bits        */                 \
}

unsigned char TWI_slaveAddress; /* I2C address, always even, should be set externally */

volatile unsigned char TWI_RxBuf[TWI_RX_BUFFER_SIZE];
volatile unsigned char TWI_TxBuf[TWI_TX_BUFFER_SIZE];

volatile unsigned char TWI_rxBufIdx;
volatile unsigned char TWI_txBufIdx;

volatile unsigned char bNewIncomingFrame;
volatile unsigned char bNewIncomingByte;

static volatile unsigned char USI_TWI_Overflow_State;

/*----------------------------------------------------------
  Initialize USI for TWI Slave mode
----------------------------------------------------------*/
void USI_TWI_Slave_Initialize(void)
{
  PORT_USI |=  (1<<PORT_TWI_SCL);                       // Set SCL high
  PORT_USI |=  (1<<PORT_TWI_SDA);                       // Set SDA high
  DDR_USI  |=  (1<<PORT_TWI_SCL);                       // Set SCL as output
  DDR_USI  &= ~(1<<PORT_TWI_SDA);                       // Set SDA as input
  USICR    =   (1<<USISIE)|(0<<USIOIE)|                 // Enable Start Condition Interrupt. Disable Overflow Interrupt
               (1<<USIWM1)|(0<<USIWM0)|                 // Set USI in Two-wire mode. No USI Counter overflow prior
                                                        // to first Start Condition (potential failure)
               (1<<USICS1)|(0<<USICS0)|(0<<USICLK)|     // Shift Register Clock Source = External, positive edge
               (0<<USITC);
  // Clear all flags and reset overflow counter
  USISR    = (1<<START_CONDITION_INT_FLAG)|(1<<COUNTER_OVERFLOW_INT_FLAG)|
             (1<<STOP_CONDITION_INT_FLAG)|(1<<DATA_OUTPUT_COLLISION_INT_FLAG)|
             (0x0<<USICNT0);
}


// ********** Interrupt Handlers ********** //

/*----------------------------------------------------------
 Detects the USI_TWI Start Condition and initializes the USI
 for reception of the "TWI Address" packet
----------------------------------------------------------*/
ISR(USI_START_vect)
{
/* Set default starting conditions for new TWI package */
    DDR_USI  &= ~(1<<PORT_TWI_SDA);                                 // Set SDA as input

/* Enable TWI counter overflow and interrupts*/
    USICR   =   (1<<USISIE)|(1<<USIOIE)|                            // Enable Overflow and Start Condition Interrupt. (Keep StartCondInt to detect RESTART)
                (1<<USIWM1)|(1<<USIWM0)|                            // Set USI in Two-wire mode
                (1<<USICS1)|(0<<USICS0)|(0<<USICLK)|                // Shift Register Clock Source = External, positive edge
                (0<<USITC);
    // this active loop is guaranteed to be very short because this interrupt draws clock to low
    while(PIN_USI&(1<<PORT_TWI_SCL)) {
     if(PIN_USI&(1<<PORT_TWI_SDA))
      break;
    }
    // clear all flags and set counter for 16 clock edges, i.e. 8 bits of data
    USISR   =   (1<<USISIF)|(1<<COUNTER_OVERFLOW_INT_FLAG)|(1<<STOP_CONDITION_INT_FLAG)|(1<<DATA_OUTPUT_COLLISION_INT_FLAG)|
                (0x0<<USICNT0);
    USI_TWI_Overflow_State = TWI_SLAVE_CHECK_ADDRESS;
}

/*----------------------------------------------------------
 Handles all the communication
 Is disabled when waiting for new Start Condition
----------------------------------------------------------*/
ISR(USI_OVERFLOW_vect)
{
    switch (USI_TWI_Overflow_State) {
        // ---------- Address mode ----------
        // Check address and send ACK (and next USI_SLAVE_SEND_DATA) if OK, else reset USI
      case TWI_SLAVE_CHECK_ADDRESS:
        if ((USIDR & 0xfe) == TWI_slaveAddress) {
            if (USIDR & 0x01) {
             USI_TWI_Overflow_State = TWI_SLAVE_SEND_DATA;
             TWI_txBufIdx=0;
            } else {
             USI_TWI_Overflow_State = TWI_SLAVE_REQUEST_DATA;
             TWI_rxBufIdx=0;
            }
            SET_TWI_TO_SEND_ACK
        } else {
            goto label__SET_USI_TO_TWI_START_CONDITION_MODE;
        }
        break;

        // ----- Master write data mode ------
        // Check reply and goto USI_SLAVE_SEND_DATA if OK, else reset USI
      case TWI_SLAVE_CHECK_REPLY_FROM_SEND_DATA:
        if (USIDR) // If NACK, the master does not want more data
        {
            goto label__SET_USI_TO_TWI_START_CONDITION_MODE;
        }
        // No 'break;' on purpose
        // From here we just drop straight into USI_SLAVE_SEND_DATA if the master sent an ACK

        // Copy data from buffer to USIDR and set USI to shift byte. Next USI_SLAVE_REQUEST_REPLY_FROM_SEND_DATA
      case TWI_SLAVE_SEND_DATA:
        // Get data from Buffer
       if (TWI_txBufIdx < TWI_TX_BUFFER_SIZE) {
            USIDR = TWI_TxBuf[TWI_txBufIdx++];
        } else {// If the buffer is empty then:
label__SET_USI_TO_TWI_START_CONDITION_MODE:
            SET_TWI_TO_START_CONDITION_MODE
            return;
        }
        USI_TWI_Overflow_State = TWI_SLAVE_REQUEST_REPLY_FROM_SEND_DATA;
        SET_TWI_TO_SEND_DATA
        break;

        // Set USI to sample reply from master. Next USI_SLAVE_CHECK_REPLY_FROM_SEND_DATA
      case TWI_SLAVE_REQUEST_REPLY_FROM_SEND_DATA:
        USI_TWI_Overflow_State = TWI_SLAVE_CHECK_REPLY_FROM_SEND_DATA;
        SET_TWI_TO_READ_ACK
        break;

        // ----- Master read data mode ------
        // Set USI to sample data from master. Next USI_SLAVE_GET_DATA_AND_SEND_ACK.
      case TWI_SLAVE_REQUEST_DATA:
        USI_TWI_Overflow_State = TWI_SLAVE_GET_DATA_AND_SEND_ACK;
        SET_TWI_TO_READ_DATA
        break;

        // Copy data from USIDR and send ACK. Next USI_SLAVE_REQUEST_DATA
      case TWI_SLAVE_GET_DATA_AND_SEND_ACK:
        // Put data into Buffer
        if(TWI_rxBufIdx<TWI_RX_BUFFER_SIZE) {
         TWI_RxBuf[TWI_rxBufIdx++] = USIDR;
         if(TWI_rxBufIdx==1) {
          bNewIncomingFrame=!0;
         }
         bNewIncomingByte=!0;
        }
        USI_TWI_Overflow_State = TWI_SLAVE_REQUEST_DATA;
        SET_TWI_TO_SEND_ACK
        break;
    }
}

