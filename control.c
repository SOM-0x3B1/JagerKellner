#include "control.h"


volatile float targetPitchAngle = 0; // keep this angle to achieve balance

volatile float Kp = 500;          // (P)roportional Tuning Parameter
volatile float Ki = 800;          // (I)ntegral Tuning Parameter        
volatile float Kd = 160;          // (D)erivative Tuning Parameter       
volatile float iTerm = 0;         // used to accumulate error (integral)
volatile float maxPID = 1024;     // the maximum value that can be output

volatile float lastPitch = 0;     // the last sensor value
volatile float lastError = 0;     // the last error value