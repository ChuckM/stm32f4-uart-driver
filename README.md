README
------

This is my somewhat "exotic" UART driver for the STM32F4 series of
microprocessors. Basically I got tired of trying to get all the various
calls correct and wrote some code that takes a transmit pin, a receive pin,
and assuming it can connect those to the same serial port it creates the
necessary calls to the chip to do so. Note, it could even split transmit
and receive across USART/UARTs but that is going a bit far. If you need 
that then do two USART inits and two APB inits one each for the RX and TX 
lines.

I borrowed the Makefiles and linkerscript from the [libopencm3][a] project
since they do everything I needed and I'm using libopencm3 for the other
infrastructure.

[a]:https://github.com/libopencm3/libopencm3

Comments or questions to me.

--Chuck
