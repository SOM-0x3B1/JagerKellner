#include "project.h"
#include "mpu6050.h"
#include "stdio.h"
#include "math.h"
#include "kalman.h"


#define RAD_TO_DEG 180/M_PI
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

Kalman_t KalmanX = {
        .Q_angle = 0.001f,
        .Q_bias = 0.003f,
        .R_measure = 0.03f
};

Kalman_t KalmanY = {
        .Q_angle = 0.001f,
        .Q_bias = 0.003f,
        .R_measure = 0.03f,
};

//volatile float test;
volatile double yaw, pitch, roll;
volatile double angX, angY;
volatile double compAngleX, compAngleY;;



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
                                        
                    
                    /*
                    //roll  = atan2f(AY, AZ) * 180/M_PI;
                    //pitch = atan2f(AX, sqrt(AY*AY + AZ*AZ)) * 180/M_PI;    
                    
                    
                    double roll_sqrt = sqrt(AX * AX + AZ * AZ);
                    if (roll_sqrt != 0.0)
                        roll = atan2f(AY, AZ) * 180/M_PI;
                    else
                        roll = 0.0;
                        
                    pitch = atan2(-AX, AZ) * 180/M_PI;
                    if ((pitch < -90 && angY > 90) || (pitch > 90 && angY < -90)) {
                        KalmanY.angle = pitch;
                        angY = pitch;
                    } else {
                        angY = Kalman_getAngle(&KalmanY, pitch, GY, 100000);
                    }
                    if (fabs(angY) > 90)
                        GX = -GX;
                    angX = Kalman_getAngle(&KalmanX, roll, GX, 100000); 
                    */
                    
                    
                    roll  = atan2(AY, AZ) * RAD_TO_DEG;
                    pitch = atan(-AX / sqrt(AY * AY + AZ * AZ)) * RAD_TO_DEG;                   
                  
                    
                    compAngleX = 0.93 * (compAngleX + GX * 0.02) + 0.07 * roll; // Calculate the angle using a Complimentary filter
                    compAngleY = 0.93 * (compAngleY + GY * 0.02) + 0.07 * pitch;
                    
                    
                    //sprintf(usbOutBuf, "\rPitch:%4d, Roll:%4d  ",  (int)angY, (int)angX); 
                    sprintf(usbOutBuf, "\rPitch:%4d, Roll:%4d  ",  (int)compAngleY, (int)compAngleX); 
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
