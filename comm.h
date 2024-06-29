#ifndef _COMM_H_
#define _COMM_H_

#include <stdbool.h>

// currently active UART source
typedef enum {
    SOURCE_USB,
    SOURCE_BLUETOOTH,
    SOURCE_WAITING
} UART_Source;

extern volatile UART_Source currentUARTSource;

typedef enum {
    CMD_PART_TYPE,  // 1st character of command
    CMD_PART_SPEC,  // 2nd character of command
    CMD_PART_VALUE  // the value after the command
} CommandPart;

extern char outBuf[120];      // UART output string buffer
extern char outBufAngles[40]; // angle part of UART output string buffer
extern char outBufSpeed[40];  // speed part of output string buffer
extern char outBufPID[40];    // PID part of  output string buffer

extern volatile int outP;
extern volatile int outI;
extern volatile int outD;
extern volatile int outPID;
extern volatile int lastOutP;
extern volatile int lastOutI;
extern volatile int lastOutD;
extern volatile int lastOutPID;


extern volatile bool sendData;

#define BT_SPACING 1
extern volatile int BTspacer; // to make Bluetooth output less frequent

extern volatile bool sendGyro;  // enable gyro out stream
extern volatile bool sendMotor; // enable motor speed out stream
extern volatile bool sendPID;   // enable PID out stream


/// Read char from the currently active UART source
char UART_getChar();

/// Checks if currently active UART source has an empty buffer
bool UART_isBufferEmpty();

/// Clears the RX buffer of the currently active UART source
void UART_clearRxBuffer();

/// Sends string to the currently active UART source
void UART_active_PutString();

/// Send string to all UART sources
/// @param forceDual disable BT spacing for this output
void UART_dual_PutString(bool forceDual);

/// Read command from currently active UART source
void UART_enum();


#endif  //_COMM_H_