#include "project.h"
#include "mpu6050.h"
#include "stdio.h"
#include "math.h"
#include "kalman.h"


#define GYRO_EVAL_INTERVAL 0.016
#define PWM_MAX 11999
volatile const float radToDeg = 180/M_PI;


typedef enum {
    MOTOR_GO,
    MOTOR_BREAKING,
    MOTOR_SLEEPING
} MotorState;

typedef enum {
    GYRO_SAMPLE,
    GYRO_EVAL
} GyroState;


char usbOutBuf[100];

volatile float targetPitchAngle = 0;
volatile float targetRollAngle = 0;

volatile GyroState gyroState = GYRO_SAMPLE;
int16_t CAX, CAY, CAZ; //current acceleration values
int16_t CGX, CGY, CGZ; //current gyroscope values
volatile float AXoff = 0, AYoff = 0, AZoff = 0; //accelerometer offset values
volatile float GXoff = 0, GYoff = 0, GZoff = 0; //gyroscope offset values
volatile double AX = 0, AY = 0, AZ = 0; //acceleration doubles
volatile double GX = 0, GY = 0, GZ = 0; //gyroscope doubles
volatile int sampleCount = 0;
volatile double accPitch, accRoll;
volatile double compRoll, compPitch;;

MotorState motorSate = MOTOR_SLEEPING;
float motorRPercentage = 0;
float motorLPercentage = 0; 



CY_ISR(GyroSampleIT){
    Timer_GY87_Sample_STATUS;
    if(gyroState == GYRO_SAMPLE)
        MPU6050_getMotion6(&CAX, &CAY, &CAZ, &CGX, &CGY, &CGZ);
}

CY_ISR(GyroEvalIT){
    Timer_GY87_Eval_STATUS;
    gyroState = GYRO_EVAL; 
}



void calibrate(int numberOfTests){
    for(int i=0; i<numberOfTests; i++) {   
        MPU6050_getMotion6(&CAX, &CAY, &CAZ, &CGX, &CGY, &CGZ);
        AXoff += CAX;
        AYoff += CAY;
        AZoff += CAZ;
        GXoff += CGX;
        GYoff += CGY;
        GZoff += CGZ;
    
        CyDelay(1);
        if((i & 3) == 0) { // Write only when i % 4 == 0
            sprintf(usbOutBuf, "\r%d / %d ", i + 1, numberOfTests);
            USBUART_PutString(usbOutBuf);
        }
    }
    
    sprintf(usbOutBuf, "\r%d / %d \n\r", numberOfTests, numberOfTests);
    USBUART_PutString(usbOutBuf);
    
    
    AXoff = AXoff/numberOfTests;
    AYoff = AYoff/numberOfTests;
    AZoff = AZoff/numberOfTests;
    GXoff = GXoff/numberOfTests;
    GYoff = GYoff/numberOfTests;
    GZoff = GZoff/numberOfTests;
}


void setPWM(float mL, float mR){
    int lPWM = (int)PWM_MAX * mL;
    int rPWM = 1 - ((int)PWM_MAX * mR);   
    PWM_Motor_WriteCompare1(lPWM);
    PWM_Motor_WriteCompare1(rPWM);
}


void init(void)
{    
    Clock_Motor_PWM_Start();
    USBUART_Start();    
    USBUART_PutString("\n\rCOM Port Open\n\r");
    
    I2C_GY87_Start();    
    MPU6050_init();
	MPU6050_initialize();
    USBUART_PutString(MPU6050_testConnection() ? "MPU6050 connection successful\n\r" : "MPU6050 connection failed\n\n\r");   
    
    USBUART_PutString("Calbirating...\n\r");
    calibrate(500);
    USBUART_PutString("Calbiration done\n\n\r");
    
    Clock_Timer_G87_Sample_Start();
    Clock_Timer_G87_Eval_Start();
    Timer_GY87_Sample_Start();
    Timer_GY87_Eval_Start();
    
    PWM_LED_Start();
    PWM_LED_BRIGHTNESS_Start();
}


int main(void)
{
    isr_Sample_GY87_StartEx(GyroSampleIT);
    isr_Eval_GY87_StartEx(GyroEvalIT);    
    
    init();   
    
    CyGlobalIntEnable; /* Enable global interrupts. */
 
    
    for(;;)
    {        
        switch(gyroState){
            case GYRO_SAMPLE: {
                AX += ((float)CAX-AXoff);
                AY += ((float)CAY-AYoff);
                AZ += ((float)CAZ-AZoff); 
                
                GX += ((float)CGX-GXoff);
                GY += ((float)CGY-GYoff);
                GZ += ((float)CGZ-GZoff);
                sampleCount++;
            
                break;
            }
            case GYRO_EVAL: {
                if(sampleCount > 0) {
                    AX /= sampleCount;
                    AY /= sampleCount;
                    AZ /= sampleCount;
                    GX /= sampleCount;
                    GY /= sampleCount;
                    GZ /= sampleCount;
                    
                    AX = ((float)CAX-AXoff)/16384.00;
                    AY = ((float)CAY-AYoff)/16384.00; //16384 is just 32768/2 to get our 1G value
                    AZ = ((float)CAZ-(AZoff-16384))/16384.00; //remove 1G before dividing
                    
                    GX = ((float)CGX-GXoff)/131.07; //131.07 is just 32768/250 to get us our 1deg/sec value
                    GY = ((float)CGY-GYoff)/131.07;
                    GZ = ((float)CGZ-GZoff)/131.07;                                         
                  
                    
                    accRoll  = atan2(AY, AZ) * radToDeg;
                    accPitch = atan(-AX / sqrt(AY * AY + AZ * AZ)) * radToDeg;                   
                  
                    
                    compRoll = 0.98 * (compRoll + GX * GYRO_EVAL_INTERVAL) + 0.02 * accRoll; // Calculate the angle using a Complimentary filter
                    compPitch = 0.98 * (compPitch + GY * GYRO_EVAL_INTERVAL) + 0.02 * accPitch;
                    
                    
                    //sprintf(usbOutBuf, "\rPitch:%4d, Roll:%4d  ",  (int)angY, (int)angX); 
                    //sprintf(usbOutBuf, "\rPitch:%4d, Roll:%4d  ",  (int)compPitch, (int)compRoll); 
                    sprintf(usbOutBuf, "%d %d\r",  (int)(compPitch*100), (int)(compRoll*100)); 
                    USBUART_PutString(usbOutBuf);
                    
                    sampleCount = 0;                
                }                
                AX = 0; AY = 0; AZ = 0;
                GX = 0; GY = 0; GZ = 0;
                
                gyroState = GYRO_SAMPLE;
                break;
            }
            default:
                break;
        }       
        
        switch(motorSate){
            case MOTOR_GO:
                setPWM(motorLPercentage, motorRPercentage);
                break;
            default:
                break;
        }
    }
}

/* [] END OF FILE */
