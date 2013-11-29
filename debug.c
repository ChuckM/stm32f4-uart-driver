/*
 * Copyright (c) 2013 - Chuck McManis, all rights reserved.
 *
 * These are some dead simple, polled, debug APIs for using the USART2 port
 * as a diagnostic console on PD5, nad PD6. I use the Black Magic ride along
 * UART to monitor this port. These are well tested and self contained so 
 * generally plugging them in is 'easy' and you can then use these (and a 
 * bunch of calls to debug_puts() :-)) to figure out what is going on in your
 * code if you can't run it under GDB.
 */
#include <stdint.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include "debug.h"

/*
 * Hard Defined, USART2, PD5 & PD6, 38400 baud. Compatible
 * with the Blackmagic Debug Probe ACM1 port.
 */
void
debug_init(void) {
	rcc_peripheral_enable_clock(&RCC_APB1ENR, RCC_APB1ENR_USART2EN);
	rcc_peripheral_enable_clock(&RCC_AHB1ENR, RCC_AHB1ENR_IOPDEN);
	/* Setup GPIO pins for USART2 transmit. PD5-PD6 to AF8*/
	gpio_mode_setup(GPIOD, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO5 | GPIO6);
	gpio_set_output_options(GPIOD, GPIO_OTYPE_OD, GPIO_OSPEED_25MHZ, GPIO5);
	gpio_set_af(GPIOD, GPIO_AF7, GPIO5 | GPIO6);

	usart_set_baudrate(USART2, 38400);
	usart_set_databits(USART2, 8);
	usart_set_stopbits(USART2, USART_STOPBITS_1);
	usart_set_mode(USART2, USART_MODE_TX_RX);
	usart_set_parity(USART2, USART_PARITY_NONE);
	usart_set_flow_control(USART2, USART_FLOWCONTROL_NONE);
	usart_enable(USART2);
}

/*
 * Get a character, optionally wait for it. (no function key support)
 */
char
debug_getc(int wait) {
    if (wait) {
        usart_wait_recv_ready(USART2);
    }
    return usart_get_flag(USART2, USART_SR_RXNE) ? (char) usart_recv(USART2) : '\000';
}

/*
 * Put a character
 */
void
debug_putc(char c) {
    usart_wait_send_ready(USART2);
    usart_send(USART2, c);
    if (c == '\n') {
        usart_wait_send_ready(USART2);
        usart_send(USART2, '\r');
    }
}

/*
 * Write out a string.
 */
void
debug_puts(const char *s) {
    char *t = (char *)s;
    while (*t) {
        debug_putc(*t++);
    }
}

static const char *w_space = "[Press Space]";
/*
 * Write out the string '[Press Space]' and wait
 * for the space key to be pressed.
 */
void
debug_wait() {
    debug_puts(w_space);
    while (debug_getc(0) != ' ');
    return;
}
