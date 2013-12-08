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
#include "term.h"

char * stime(uint32_t t);

static const char *greet = "UART Driver Test Program.\n";

int 
main(void) {
    int u;
    char buf[128];
    uint32_t delay;
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
        uart_putnum(u, FMT_BASE_2 | FMT_LEADING_ZERO | FMT_ALTERNATE_FORM | 1, n & 0xff);
        uart_puts(u, " -- ");
        uart_putnum(u, FMT_BASE_16 | FMT_ALTERNATE_FORM, n);
        uart_puts(u, " -- ");
        uart_putnum(u, FMT_BASE_16 | FMT_LEADING_ZERO | 1, n);
        uart_puts(u, " -- ");
        uart_putnum(u, FMT_BASE_16 | FMT_LEADING_ZERO | 2, n);
        uart_puts(u, "\n");
    }
    while (1) {
        delay = mtime() + 5000;
        move_cursor(u, 1, 40);
        clear_screen(u);
        while (delay > mtime()) {
            move_cursor(u, 1, 40);
            text_color(u, YELLOW);
            uart_puts(u, stime(mtime()));
            msleep(100);
        }
        move_cursor(u, 10, 1);
        text_color(u, DEFAULT);
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
        uart_putnum(u, FMT_BASE_10 | FMT_ALTERNATE_FORM | FMT_SIGNED, n);
        uart_puts(u, " -- ");
        uart_putnum(u, FMT_BASE_2 | FMT_ALTERNATE_FORM, n);
        uart_puts(u, " -- ");
        uart_putnum(u, FMT_BASE_8 | FMT_ALTERNATE_FORM, n);
        uart_puts(u, " -- ");
        uart_putnum(u, FMT_BASE_16 | FMT_ALTERNATE_FORM, n);
        uart_puts(u, "\n");
    }
}

/*
 * stime(uint32_t)
 *
 * Convert a number representing milliseconds into a 'time' string
 * of HHH:MM:SS.mmm where HHH is hours, MM is minutes, SS is seconds
 * and .mmm is fractions of a second.
 */
char *
stime(uint32_t t) {
    static char time_string[14];
    uint16_t msecs = t % 1000;
    uint8_t secs = (t / 1000) % 60;
    uint8_t mins = (t / 60000) % 60;
    uint16_t hrs = (t /3600000);

    // HH:MM:SS.mmm\0
    // 0123456789abc
    time_string[0] = (hrs / 100) % 10 + '0';
    time_string[1] = (hrs / 10) % 10 + '0';
    time_string[2] = hrs % 10 + '0';
    time_string[3] = ':';
    time_string[4] = (mins / 10)  % 10 + '0';
    time_string[5] = mins % 10 + '0';
    time_string[6] = ':';
    time_string[7] = (secs / 10)  % 10 + '0';
    time_string[8] = secs % 10 + '0';
    time_string[9] = '.';
    time_string[10] = (msecs / 100) % 10 + '0';
    time_string[11] = (msecs / 10) % 10 + '0';
    time_string[12] = msecs % 10 + '0';
    time_string[13] = 0;
    return &time_string[0];
}

