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

In addition there is some number formatting code (ntoa) which, yes, is 
basically a re-implementation of some of the capabilities of `printf` 
except that I find its harder to find just the 'formatting' code from
printf these days. And since I have been having issues getting newlib
to do what I want on a bare metal platform, I once again wrote some number
formatting code. Basically it prints numbers as either decimal, binary,
octal, or hex. And it can add some decoration to indicated their base
(decimals end in .0, octal starts with a leading 0, binary with 0b, and
hexadecimal with 0x) The goal being a companion 'getnum' which can reverse
that and process any base number. The combination being useful in the
development of a small embedded monitor for the ARM chip.

Comments or questions to me.

--Chuck
