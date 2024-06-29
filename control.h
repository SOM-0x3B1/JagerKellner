#ifndef _CONFIG_H_
#define _CONFIG_H_
    

//=====[ Main parameters ]==================
    
    
#define TIPPING_TRESHOLD 30 // give up control and suspend motors over this tilt angle
    

extern volatile float targetPitchAngle; // keep this angle to achieve balance

extern volatile float Kp;          // (P)roportional Tuning Parameter
extern volatile float Ki;          // (I)ntegral Tuning Parameter        
extern volatile float Kd;          // (D)erivative Tuning Parameter       
extern volatile float iTerm;       // used to accumulate error (integral)
extern volatile float maxPID;      // the maximum value that can be output

extern volatile float lastPitch;   // the last sensor value
extern volatile float lastError;   // the last error value
    
    
    
#endif //_CONFIG_H_