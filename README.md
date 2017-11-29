# ATtiny-Slave-I2c
**Simple I2c (TWI) Slave Drive on Atmel ATtiny 2313A/4313**

Using the USI (Universal Serial Interface) in TWI mode (two-wire mode, compatible with I2C)

This module is inspired from Atmel Technical Note AVR312 "Using the USI module as a TWI slave"

It is simplified in that it fills two I/O buffers. Those buffers are non-circular: this simplifies implementation, is more flexible in terms of buffer size and is easier to use in constrained environments

We also found source code that is attached to this technical note to have a flaw in the start condition interrupt routine. The while loop in this routine is wrong, which most of the time makes the last clock pulse of the address/read/write byte to be truncated by an early overflow interrupt which forcibly and in premature way drives the clock to a low state

The attempt also consists of completely removing any active while loop in the TWI interrupts

As stated in this application note, this driver should work as well with the ATtiny26 and Atmega169 MCU's
