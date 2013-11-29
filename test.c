/*
 * Code to test my UART driver
 *
 * It is set up to talk on USART6 (because PC6, PC7) and this
 * is the USART used by the Embest expansion board for the
 * STM32F4-DISCOVERY board. And talking on USART2 (PD5, PD6)
 * as the 'debug' console, which I have connected to the serial
 * port that comes off the BlackMagic Debug Probe. When run
 *
 * Console 1 prints:            Console 2 (debug) prints
 * ------------------------     ---------------------------------------
 *                              Debugging Initialized
 * UART Driver Test Program     [Press Space] (you need to press space)
 * Enter some text : <enter text>
 * 'what you entered'
 * Enter some text :
 */

#include <stdint.h>
#include "uart.h"
#include "clock.h"
#include "debug.h"

static const char *greet = "UART Driver Test Program.\n";

int 
main(void) {
    int u;
    char buf[128];

    clock_init(1000);
    debug_init();
    debug_puts("Debugging initialized.\n");
    u = uart_init(PC6, PC7, 115200);
    uart_puts(u, greet);
    debug_wait();
    while (1) {
        uart_puts(u, "Enter some text: ");
        uart_gets(u, buf, 128);
        uart_puts(u, "'");
        uart_puts(u, buf);
        uart_puts(u, "'");
        uart_puts(u, "\n\r");
    }
}
