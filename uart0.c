// UART0 Library
// Jason Losh

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL
// Target uC:       TM4C123GH6PM
// System Clock:    -

// Hardware configuration:
// UART Interface:
//   U0TX (PA1) and U0RX (PA0) are connected to the 2nd controller
//   The USB on the 2nd controller enumerates to an ICDI interface and a virtual COM port

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <uart0.h>
#include "tm4c123gh6pm.h"

// PortA masks
#define UART_TX_MASK 2
#define UART_RX_MASK 1

//-----------------------------------------------------------------------------
// Global variables
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

// Initialize UART0
void initUart0()
{
    // Enable clocks
    SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R0;
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R0;
    _delay_cycles(3);

    // Configure UART0 pins
    GPIO_PORTA_DR2R_R |= UART_TX_MASK;                  // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTA_DEN_R |= UART_TX_MASK | UART_RX_MASK;    // enable digital on UART0 pins
    GPIO_PORTA_AFSEL_R |= UART_TX_MASK | UART_RX_MASK;  // use peripheral to drive PA0, PA1 // AF_SEL is 6 bits for each port (1,2...)
    GPIO_PORTA_PCTL_R &= ~(GPIO_PCTL_PA1_M | GPIO_PCTL_PA0_M); // clear bits 0-7
    GPIO_PORTA_PCTL_R |= GPIO_PCTL_PA1_U0TX | GPIO_PCTL_PA0_U0RX;
                                                        // select UART0 to drive pins PA0 and PA1: default, added for clarity

    // Configure UART0 to 115200 baud, 8N1 format
    UART0_CTL_R = 0;                                    // turn-off UART0 to allow safe programming
    UART0_CC_R = UART_CC_CS_SYSCLK;                     // use system clock (40 MHz)
    UART0_IBRD_R = 21;                                  // r = 40 MHz / (Nx115.2kHz), set floor(r)=21, where N=16
    UART0_FBRD_R = 45;                                  // round(fract(r)*64)=45
    UART0_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN;    // configure for 8N1 w/ 16-level FIFO
    UART0_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN;
                                                        // enable TX, RX, and module
}

// Set baud rate as function of instruction cycle frequency
void setUart0BaudRate(uint32_t baudRate, uint32_t fcyc)
{
    uint32_t divisorTimes128 = (fcyc * 8) / baudRate;   // calculate divisor (r) in units of 1/128,
                                                        // where r = fcyc / 16 * baudRate
    divisorTimes128 += 1;                               // add 1/128 to allow rounding
    UART0_CTL_R = 0;                                    // turn-off UART0 to allow safe programming
    UART0_IBRD_R = divisorTimes128 >> 7;                // set integer value to floor(r)
    UART0_FBRD_R = ((divisorTimes128) >> 1) & 63;       // set fractional value to round(fract(r)*64)
    UART0_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN;    // configure for 8N1 w/ 16-level FIFO
    UART0_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN;
                                                        // turn-on UART0
}

// Blocking function that writes a serial character when the UART buffer is not full
void putcUart0(char c)
{
    while (UART0_FR_R & UART_FR_TXFF);               // wait if uart0 tx fifo full
    UART0_DR_R = c;                                  // write character to fifo
}

// Blocking function that writes a string when the UART buffer is not full
void putsUart0(char* str)
{
    uint8_t i = 0;
    while (str[i] != '\0')
        putcUart0(str[i++]);
}

// Blocking function that returns with serial data once the buffer is not empty
char getcUart0()
{
    while (UART0_FR_R & UART_FR_RXFE);               // wait if uart0 rx FIFO empty
    return UART0_DR_R & 0xFF;                        // get character from fifo
}



// Returns the status of the receive buffer
bool kbhitUart0()
{
    return !(UART0_FR_R & UART_FR_RXFE);
}

void getsUart0(USER_DATA *save_to)
{
    int i = 0;

    char *buffer = save_to->buffer;

    while (i < MAX_CHARS)
    {
        char c = getcUart0();

        if (c == '\n' || c == '\r')
            break;

        if (c == 8 || c == 127)
        {
            if (i > 0)
                --i;
        }
        else if (c >= 32)
        {
            buffer[i++] = c;
        }
        // special characters. For now, we just ignore them
        else {}
    }

    buffer[i] = '\0';
}

// TODO: stronger type system. Just add more types (some important chars)
char _getFieldType(char c)
{
    // Integer types
    if (c >= '0' && c <= '9')
        return 'n';
    if (c == '-' || c == '.')
        return 'n';

    // Alphabets
    if (c >= 'a' && c <= 'z')
        return 'a';
    if (c >= 'A' && c <= 'Z')
        return 'a';

    // Delimiters AND special characters.
    // This is OK since we always ignore them anyways
    return '*';
}

void parseFields(USER_DATA *save_to)
{
    // start with delim. basically ignore leading
    // delim. At least for now
    char lastType = '*';

    uint8_t num_fields = 0;
    char *buffer = save_to->buffer;

    int i;
    for (i = 0; buffer[i] && num_fields < MAX_FIELDS; ++i)
    {
        char curType = _getFieldType(buffer[i]);

        if (lastType != curType)
        {
            // delim is not a field. neither are special characters yet
            // Only numbers or alphabetics
            if (curType == 'n' || curType == 'a')
            {
                // Count a string like '123ab' as a num not 'num (123ab) and char (ab)'
                if (lastType == '*')
                {
                    save_to->fieldPosition[num_fields] = i;
                    save_to->fieldType[num_fields] = curType;
                    ++num_fields;
                }
            }
            // replace first delim we run into after non-delim streak to with NULL
            else
                save_to->buffer[i] = '\0';
        }

        lastType = curType;
    }

    save_to->fieldCount = num_fields;
}

char* getFieldString(USER_DATA* data, uint8_t fieldNumber)
{
    return fieldNumber < data->fieldCount
            ? &data->buffer[data->fieldPosition[fieldNumber]]
            : NULL;
}

int32_t getFieldInteger(USER_DATA* data, uint8_t fieldNumber)
{
    return fieldNumber < data->fieldCount && data->fieldType[fieldNumber] == 'n'
            ? atoi(&data->buffer[data->fieldPosition[fieldNumber]]) : 0;
}

bool _isstreq(const char str1[], const char str2[])
{
    while (*str1 != '\0' && *str1 == *str2)
    {
        ++str1; ++str2;
    }

    return *str2 == '\0';
}

bool isCommand(USER_DATA* data, const char strCommand[], uint8_t minArguments)
{
    char *str = data->buffer + data->fieldPosition[0];
    return _isstreq(strCommand, str) && data->fieldCount > minArguments;
}

