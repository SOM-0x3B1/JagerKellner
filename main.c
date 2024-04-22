#include "project.h"
#include "mpu6050.h"
#include "stdio.h"
#include "math.h"
#include "MadgwickAHRS.h"


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
volatile float AX = 0, AY = 0, AZ = 0; //acceleration floats
volatile float GX = 0, GY = 0, GZ = 0; //gyroscope floats
volatile int sampleCount = 0;

volatile float test;
volatile float yaw;
volatile float pitch;
volatile float roll;


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
        if((i & 1) == 0) { // Write only when i % 2 == 0
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
        AX += ((float)CAX-AXoff);
        AY += ((float)CAY-AYoff);
        AZ += ((float)CAZ-AZoff); 
        
        GX += ((float)CGX-GXoff);
        GY += ((float)CGY-GYoff);
        GZ += ((float)CGZ-GZoff);
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
                    
                    AX = ((float)CAX-AXoff)/16384.00;
                    AY = ((float)CAY-AYoff)/16384.00; //16384 is just 32768/2 to get our 1G value
                    AZ = ((float)CAZ-(AZoff-16384))/16384.00; //remove 1G before dividing
                    
                    GX = ((float)CGX-GXoff)/131.07; //131.07 is just 32768/250 to get us our 1deg/sec value
                    GY = ((float)CGY-GYoff)/131.07;
                    GZ = ((float)CGZ-GZoff)/131.07; 
    
                    
                    /* MadgwickAHRSupdateIMU(AX, AY, AZ, GX, GY, GZ);
                    
                    test = q1*q2 +q0*q3;
                    //heading (about z), attitude (about y), bank (about x) = yaw,pitch,roll
                	if (test > 0.499) { // singularity at north pole
                		yaw = 2 * atan2(q1,q0)*(180/M_PI);
                		pitch = 90;
                		roll= 0;
                	}
                    else{
                        if (test < -0.499) { // singularity at south pole
                            yaw = (-2 * atan2(q1,q0))*(180/M_PI);
                            pitch = -90;
                            roll = 0;
                        } 
                        else {
                            roll = atan2(2*(q0*q1+q2*q3),1-2*(q1*q1 +q2*q2))*(180/M_PI);
                            pitch = asin(2*(q0*q2-q3*q1))*(180/M_PI);
                            yaw = atan2(2*(q0*q3+q1*q2),1-2*(q2*q2+q3*q3))*(180/M_PI);
                        }
                    }*/
                            
                    /*yaw   = atan2(2.0f * (q1 * q2 + q0 * q3), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3);
                    pitch = -asin(2.0f * (q1 * q3 - q0 * q2));
                    roll  = atan2(2.0f * (q0 * q1 + q2 * q3), q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3);
                    pitch *= 180.0f / M_PI;
                    yaw   *= 180.0f / M_PI;
                    roll  *= 180.0f / M_PI;*/
                    
                    /*fXg = Xg * alpha + (fXg * (1.0 - alpha));
                    fYg = Yg * alpha + (fYg * (1.0 - alpha));
                    fZg = Zg * alpha + (fZg * (1.0 - alpha));
                    
                    roll  = (int)(atan2(-AY, AZ)*180.0)/M_PI;
                    pitch = (int)(atan2(AX, sqrt(AY*AY + AZ*AZ))*180.0)/M_PI;*/
                    
                    //sprintf(usbOutBuf, "\rAY:%6d, AZ:%6d, Roll:%6d, Pitch:%6d     ", AY, AZ, roll, pitch);
                    sprintf(usbOutBuf, "\rAX:%6d, AY:%6d, AZ:%6d  ||  GX:%6d, GY:%6d, GZ:%6d   ", (int)AX, (int)AY, (int)AZ, (int)GX, (int)GY, (int)GZ);
                    //sprintf(usbOutBuf, "\r  Pitch:%6d, Roll:%6d, Yaw:%6d   ",  (int)pitch, (int)roll, (int)yaw);
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
