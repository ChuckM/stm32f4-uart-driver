#ifndef __UART_H
#define __UART_H

/*
 * UART Pin Map 
 */

/* All pins that can be a TX or RX pin for a USART or a UART */
enum UART_PORT_PIN {
        PA0,    // UART4_TX
        PA1,    // UART4_RX
        PA2,    // USART2_TX
        PA3,    // USART2_RX
        PA9,    // USART1_TX
        PA10,   // USART1_RX
        PB6,    // USART1_TX
        PB7,    // USART1_RX
        PB10,   // USART3_TX
        PB11,   // USART3_RX
        PC6,    // USART6_TX
        PC7,    // USART6_RX
        PC10,   // USART3_TX, UART4_TX (these default to USART3)
        PC11,   // USART3_RX, UART4_RX,
        PC12,   // USART5_TX,
        PD2,    // USART5_RX,
        PD5,    // USART2_TX
        PD6,    // USART2_RX
        PD8,    // USART3_TX
        PD9,    // USART3_RX
        PE0,    // UART8_RX
        PE1,    // UART8_TX
        PE7,    // UART7_RX
        PE8,    // UART7_TX
        PF6,    // UART7_RX
        PF7,    // UART7_TX
        PG9,    // USART6_RX
        PG14,   // USART6_TX
};

/* Defines for the putnum function */
#define FMT_WIDTH_MASK      0x1f << 0
#define FMT_BASE_10         0 << 5
#define FMT_BASE_2          1 << 5
#define FMT_BASE_8          2 << 5
#define FMT_BASE_16         3 << 5
#define FMT_BASE_MASK       3 << 5
#define FMT_SIGNED          1 << 6
#define FMT_LEADING_ZERO    1 << 7
#define FMT_ALTERNATE_FORM  1 << 8
#define FMT_LEFT_ADJUST     1 << 9
#define FMT_NEWLINE         1 << 10
#define FMT_DECIMAL         FMT_BASE_10 | FMT_ALTERNATE_FORM | FMT_SIGNED
#define FMT_HEX_BYTE        FMT_BASE_16 | FMT_LEADING_ZERO | 1
#define FMT_HEX_WORD        FMT_BASE_16 | FMT_LEADING_ZERO | 3
#define FMT_HEX_LONG        FMT_BASE_16 | FMT_LEADING_ZERO | 7
#define FMT_BINARY_BYTE     FMT_BASE_2 | FMT_LEADING_ZERO | 7
#define FMT_BINARY_WORD     FMT_BASE_2 | FMT_LEADING_ZERO | 15
#define FMT_BINARY_LONG     FMT_BASE_2 | FMT_LEADING_ZERO | 31
#define FMT_HEX_CONSTANT    FMT_BASE_16 | FMT_ALTERNATE_FORM

/* Prototypes */
int uart_init(enum UART_PORT_PIN tx, enum UART_PORT_PIN rx, int baudrate);
void uart_setbaudrate(int ud, int baudrate);
char uart_getc(int channel, int wait);
void uart_putc(int channel, char c);
void uart_puts(int channel, const char *s);
char *uart_gets(int channel, char *s, int len);
void uart_putnum(int channel, uint16_t fmt, uint32_t num);
int uart_getnum(int channel, uint16_t fmt, uint32_t *num);
void ntoa(uint32_t val, char *buf, int base);
uint32_t aton(char *buf, int base);
#endif // __UART_H
