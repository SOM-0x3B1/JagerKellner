#include "project.h"
#include "mpu6050.h"
#include "stdio.h"


#define PWM_MAX 11999

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


volatile float targetAngle = 0;

volatile GyroState gyroState = GYRO_SAMPLE;
int16_t CAX, CAY, CAZ; //current acceleration values
int16_t CGX, CGY, CGZ; //current gyroscope values
volatile float AXoff = 0, AYoff = 0, AZoff = 0; //accelerometer offset values
volatile float GXoff = 0, GYoff = 0, GZoff = 0; //gyroscope offset values
volatile int AX = 0, AY = 0, AZ = 0; //acceleration floats
volatile int GX = 0, GY = 0, GZ = 0; //gyroscope floats
volatile int sampleCount = 0;


void calibrate(int numberOfTests){
    for(int i=0; i<numberOfTests; i++) {   
        MPU6050_getMotion6(&CAX, &CAY, &CAZ, &CGX, &CGY, &CGZ);
        AXoff += CAX;
        AYoff += CAY;
        AZoff += CAZ;
        GXoff += CGX;
        GYoff += CGY;
        GZoff += CGZ;
    
        CyDelay(25);
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


void init(void)
{    
    Clock_Motor_PWM_Start();
    /*USBUART_Start(0, USBUART_5V_OPERATION);
    USBUART_CDC_Init();
    while(USBUART_GetConfiguration() == 0) {}*/
    USBUART_Start();    
    USBUART_PutString("\n\rCOM Port Open\n\r");
    
    I2C_GY87_Start();    
    MPU6050_init();
	MPU6050_initialize();
    USBUART_PutString(MPU6050_testConnection() ? "MPU6050 connection successful\n\r" : "MPU6050 connection failed\n\n\r");   
    
    USBUART_PutString("Calbirating...\n\r");
    calibrate(100);
    USBUART_PutString("Calbiration done\n\r");
    
    Clock_Timer_G87_Sample_Start();
    Clock_Timer_G87_Eval_Start();
    Timer_GY87_Sample_Start();
    Timer_GY87_Eval_Start();
    
    PWM_LED_Start();
    PWM_LED_BRIGHTNESS_Start();
}


void setPWM(float mL, float mR){
    int lPWM = (int)PWM_MAX * mL;
    int rPWM = 1 - ((int)PWM_MAX * mR);   
    PWM_Motor_WriteCompare1(lPWM);
    PWM_Motor_WriteCompare1(rPWM);
}


CY_ISR(GyroSampleIT){
    Timer_GY87_Sample_STATUS;
    if(gyroState == GYRO_SAMPLE) {
        MPU6050_getMotion6(&CAX, &CAY, &CAZ, &CGX, &CGY, &CGZ);
        AX += (CAX-AXoff);
        AY += (CAY-AYoff);
        AZ += (CAZ-AZoff); 
        
        GX += (CGX-GXoff);
        GY += (CGY-GYoff);
        GZ += (CGZ-GZoff);
        sampleCount++;
    }
}

CY_ISR(GyroEvalIT){
    Timer_GY87_Eval_STATUS;
    if(gyroState == GYRO_SAMPLE)
        gyroState = GYRO_EVAL; 
}



int main(void)
{
    isr_Sample_GY87_StartEx(GyroSampleIT);
    isr_Eval_GY87_StartEx(GyroEvalIT);    
    
    init();   
    
    CyGlobalIntEnable; /* Enable global interrupts. */
    
    
    MotorState motorSate = MOTOR_SLEEPING;
    float motorRPercentage = 0;
    float motorLPercentage = 0;   
    
    
    for(;;)
    {        
        switch(gyroState){
            case GYRO_EVAL:
                if(sampleCount > 0) {
                    AX /= sampleCount;
                    AY /= sampleCount;
                    AZ /= sampleCount;
                    GX /= sampleCount;
                    GY /= sampleCount;
                    GZ /= sampleCount;
                    
                    sprintf(usbOutBuf, "\rAX:%6d, AY:%6d, AZ:%6d || GX:%6d, GY:%6d, GZ:%6d     ", AX, AY, AZ, GX, GY, GZ);
                    USBUART_PutString(usbOutBuf);  
                    
                    sampleCount = 0;
                }                
                AX = 0; AY = 0; AZ = 0;
                GX = 0; GY = 0; GZ = 0;
                
                gyroState = GYRO_SAMPLE;
                break;
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
