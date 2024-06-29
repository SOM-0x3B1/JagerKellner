#include "comm.h"
#include "project.h"
#include "stdio.h"
#include "control.h"


//=====[ Serial communication ]==================

volatile UART_Source currentUARTSource = SOURCE_WAITING;

char outBuf[120];      // UART output string buffer
char outBufAngles[40]; // angle part of UART output string buffer
char outBufSpeed[40];  // speed part of output string buffer
char outBufPID[40];    // PID part of  output string buffer

volatile int outP = 0;
volatile int outI = 0;
volatile int outD = 0;
volatile int outPID = 0;
volatile int lastOutP = 0;
volatile int lastOutI = 0;
volatile int lastOutD = 0;
volatile int lastOutPID = 0;


volatile bool sendData = false;

volatile int BTspacer = 0; // to make Bluetooth output less frequent

volatile bool sendGyro = true;  // enable gyro out stream
volatile bool sendMotor = true; // enable motor speed out stream
volatile bool sendPID = true;   // enable PID out stream


/// Read char from the currently active UART source
char UART_getChar(){
    switch (currentUARTSource){
        case SOURCE_USB:
            return  UART_USB_GetChar();
        case SOURCE_BLUETOOTH:
            return  UART_Bluetooth_GetChar();
        default:
            return 0;
    }
}

/// Checks if currently active UART source has an empty buffer
bool UART_isBufferEmpty(){
    if(currentUARTSource == SOURCE_USB)
        return !UART_USB_GetRxBufferSize();
    else
        return !UART_Bluetooth_GetRxBufferSize();
}

/// Clears the RX buffer of the currently active UART source
void UART_clearRxBuffer(){
    if(currentUARTSource == SOURCE_USB)
        UART_USB_ClearRxBuffer();
    else
        UART_Bluetooth_ClearRxBuffer();
}

/// Sends string to the currently active UART source
void UART_active_PutString(){
    if(currentUARTSource == SOURCE_USB)
        UART_USB_PutString(outBuf);
    else
        UART_Bluetooth_PutString(outBuf);    
}

/// Send string to all UART sources
/// @param forceDual disable BT spacing for this output
void UART_dual_PutString(bool forceDual){    
    if(forceDual){
        UART_Bluetooth_PutString(outBuf);
        UART_USB_PutString(outBuf);
    }
    else {
        UART_USB_PutString(outBuf);
        if(BTspacer >= BT_SPACING){
            UART_Bluetooth_PutString(outBuf);
            BTspacer = 0;
        }
        else
            BTspacer++;
    }
}

/// Read command from currently active UART source
void UART_enum() {
    if(currentUARTSource != SOURCE_WAITING){
        bool fail = false;
        volatile CommandPart currPart = CMD_PART_TYPE; // which part of the command string to expect
        
        /* S: set parameter
         * E: set enabled
         */
        volatile char cmdType; 
        
        /* P: set proportional
         * I: set integral
         * D: set derivative
         * A: set target angle
         * G: enable gyroscope out stream
         * M: enable motor out stream
         * P: enable PID out stream
         */
        volatile char cmdSpec;
        
        volatile int value = 0;
        bool value_negative = false;        
        
        volatile char ch;
        do {
            ch = UART_getChar();
        } while (ch == '\0');
        
        // state machine
        while(!fail && ch != '\n' && ch != '\r'){
            switch(currPart) {
                // get type
                case CMD_PART_TYPE: {
                    if(ch == 'S' || ch == 'E') {
                        cmdType = ch;
                        currPart = CMD_PART_SPEC;
                    }
                    else { 
                        UART_clearRxBuffer();
                        fail = true;
                    }
                    break;
                }
                
                // get specification
                case CMD_PART_SPEC: {
                    if (cmdType == 'S'){
                        if(ch == 'P' || ch == 'I' || ch == 'D' || ch == 'A') {
                            cmdSpec = ch;
                            currPart = CMD_PART_VALUE;
                        }
                        else { 
                            UART_clearRxBuffer();
                            fail = true;
                        }
                    }
                    else if (cmdType == 'E') {
                        if(ch == 'G' || ch == 'M' || ch == 'P'){
                            cmdSpec = ch;
                            currPart = CMD_PART_VALUE;
                        }
                        else{
                            UART_clearRxBuffer();
                            fail = true;
                        }
                    }
                    break;
                }
                
                // get value
                case CMD_PART_VALUE: {
                    if(ch == '-')
                        value_negative = true;
                    else if(isdigit(ch)){
                        value *= 10;
                        value += ch - '0';
                    } else
                        fail = true;
                    break;
                }
            }
            if(!fail){
                do {
                    ch = UART_getChar();
                } while (ch == 0);
            }
        }        
      
        if(!fail){
            // set
            if(cmdType == 'S'){
                if(value_negative) value = -value;
                switch(cmdSpec){
                    case 'P':
                        Kp = (float)value;
                        break;
                    case 'I':
                        Ki = (float)value;
                        break;
                    case 'D':
                        Kd = (float)value;
                        break;
                    case 'A':
                        targetPitchAngle = (float)value / 10;
                        break;
                }
            } 
            // enable
            else if(cmdType == 'E') {
                if(cmdSpec == 'G')
                    sendGyro = (value != 0);
                else if(cmdSpec == 'M')
                    sendMotor = (value != 0);
                else if(cmdSpec == 'P')
                    sendPID = (value != 0);
            }
        }        
        //UART_clearRxBuffer();
        currentUARTSource = SOURCE_WAITING;
    }   
}