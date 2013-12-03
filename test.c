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
 * Some tests of the number
 * formatting code. Then the
 * prompt :
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
    int n;

    clock_init(1000);
    debug_init();
    debug_puts("\n---------- new session --------------\nDebugging initialized.\n");
    u = uart_init(PC6, PC7, 115200);
    uart_puts(u, greet);
    // debug_wait();
    uart_puts(u, "Testing number formatting.\n");
    uart_puts(u, "Address of buf is : ");
    uart_putnum(u, FMT_HEX_CONSTANT | FMT_NEWLINE, (uint32_t) &buf[0]);
    uart_puts(u, "Number test sequence :\n");
    for (n = -10; n < 10; n++) {
        uart_putnum(u, FMT_BASE_10 | FMT_ALTERNATE_FORM | FMT_SIGNED, n);
        uart_puts(u, " -- ");
        uart_putnum(u, FMT_BASE_8 | FMT_ALTERNATE_FORM, n);
        uart_puts(u, " -- ");
        uart_putnum(u, FMT_BASE_2 | FMT_LEADING_ZERO | FMT_ALTERNATE_FORM | 10, n & 0xff);
        uart_puts(u, " -- ");
        uart_putnum(u, FMT_BASE_16 | FMT_ALTERNATE_FORM, n);
        uart_puts(u, " -- ");
        uart_putnum(u, FMT_BASE_16 | FMT_LEADING_ZERO | 2, n & 0xff);
        uart_puts(u, " -- ");
        uart_putnum(u, FMT_BASE_16 | FMT_LEADING_ZERO | 8, n & 0xffff);
        uart_puts(u, "\n");
    }
    while (1) {
        uart_puts(u, "Enter some text: ");
        uart_gets(u, buf, 128);
        uart_puts(u, "'");
        uart_puts(u, buf);
        uart_puts(u, "'");
        uart_puts(u, "\n");
        uart_puts(u, "Enter a number : ");
        uart_gets(u, buf, 128);
        n = aton(buf);
        uart_putnum(u, FMT_BASE_10 | FMT_ALTERNATE_FORM, n);
        uart_puts(u, " -- ");
        uart_putnum(u, FMT_BASE_2 | FMT_ALTERNATE_FORM, n);
        uart_puts(u, " -- ");
        uart_putnum(u, FMT_BASE_8 | FMT_ALTERNATE_FORM, n);
        uart_puts(u, " -- ");
        uart_putnum(u, FMT_BASE_16 | FMT_ALTERNATE_FORM, n);
        uart_puts(u, "\n");
    }
}
