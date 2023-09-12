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

#ifndef UART0_H_
#define UART0_H_

//----------------------------------------------------------------------------
// Type definitions
//----------------------------------------------------------------------------

#define MAX_CHARS 80
#define MAX_FIELDS 6

typedef struct _USER_DATA{
    char buffer[MAX_CHARS+1];
    uint8_t fieldCount;
    uint8_t fieldPosition[MAX_FIELDS];
    char fieldType[MAX_FIELDS];
} USER_DATA;


//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

void initUart0();
void setUart0BaudRate(uint32_t baudRate, uint32_t fcyc);
void putcUart0(char c);
void putsUart0(char* str);
char getcUart0();
bool kbhitUart0();

void getsUart0(USER_DATA *save_to);
void parseFields(USER_DATA *save_to);
char* getFieldString(USER_DATA* data, uint8_t fieldNumber);
int32_t getFieldInteger(USER_DATA* data, uint8_t fieldNumber);
bool _isstreq(const char str1[], const char str2[]);
bool isCommand(USER_DATA* data, const char strCommand[], uint8_t minArguments);

#endif
