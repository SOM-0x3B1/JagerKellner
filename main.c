#include "project.h"
#include "mpu6050.h"
#include "stdio.h"
#include "math.h"
#include "kalman.h"


volatile const float radToDeg = 180/M_PI;


typedef enum {
    MOTOR_GO,
    MOTOR_BRAKING,
    MOTOR_SLEEPING
} MotorState;

typedef enum {
    FORWARD,
    BACWARDS
} Direction;

typedef enum {
    GYRO_SAMPLE,
    GYRO_EVAL
} GyroState;


char usbOutBuf[100];

volatile float targetPitchAngle = 0;
volatile float targetRollAngle = 0;

#define GYRO_EVAL_INTERVAL 0.016
volatile GyroState gyroState = GYRO_SAMPLE;
volatile bool gyroStarted = false;
int16_t CAX = 0, CAY = 0, CAZ = 0; //current acceleration values
int16_t CGX = 0, CGY = 0, CGZ = 0; //current gyroscope values
volatile float AXoff = 0, AYoff = 0, AZoff = 0; //accelerometer offset values
volatile float GXoff = 0, GYoff = 0, GZoff = 0; //gyroscope offset values
volatile double AX = 0, AY = 0, AZ = 0; //acceleration doubles
volatile double GX = 0, GY = 0, GZ = 0; //gyroscope doubles
volatile int sampleCount = 0;
volatile double accPitch = 0, accRoll = 0;
volatile double compRoll = 0, compPitch = 0;





#define PWM_MAX 11999

typedef struct MotorIn{
    uint8 first;
    uint8 second;  
} MotorIn;
const MotorIn motor_in_sleep = (MotorIn) {0, 0};
const MotorIn motor_in_forward = (MotorIn) {1, 0};
const MotorIn motor_in_backwards = (MotorIn) {0, 1};
const MotorIn motor_in_brake = (MotorIn) {1, 1};

MotorState motor_sate = MOTOR_SLEEPING;
bool motor_firstStart = true;
Direction motor_newDirection;
Direction motor_lastDirection;
MotorIn motor_curr_in;
float motor_LPercentage = 0; 
float motor_RPercentage = 0;



CY_ISR(GyroSampleIT){
    Timer_GY87_Sample_STATUS;
    if(gyroState == GYRO_SAMPLE)
        MPU6050_getMotion6(&CAX, &CAY, &CAZ, &CGX, &CGY, &CGZ);
}

CY_ISR(GyroEvalIT){
    Timer_GY87_Eval_STATUS;
    gyroState = GYRO_EVAL; 
}



void gyro_calibrate(int numberOfTests){
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


void motor_evalDirection(){    
    if(compPitch >= 0){
        motor_newDirection = FORWARD;
        motor_curr_in = motor_in_forward;
    }
    else {
        motor_newDirection = BACWARDS;
        motor_curr_in = motor_in_backwards;
    }
    
    if(motor_firstStart)
        motor_lastDirection = motor_newDirection;
    
    if( motor_newDirection != motor_lastDirection){
        motor_sate = MOTOR_BRAKING;
        motor_curr_in = motor_in_brake;
    }
    
    motor_lastDirection = motor_newDirection;
}

void motor_setDirection(){
    Pin_Motor_In_1_Write(motor_curr_in.first);
    Pin_Motor_In_2_Write(motor_curr_in.second);
    Pin_Motor_In_3_Write(motor_curr_in.first);
    Pin_Motor_In_4_Write(motor_curr_in.second);
}

void motor_setPWM(){
    int lPWM = (int)(PWM_MAX * motor_LPercentage);
    int rPWM = (int)(PWM_MAX * (1 - motor_RPercentage)); // inverted PWM
    PWM_Motor_WriteCompare1(lPWM);
    PWM_Motor_WriteCompare2(rPWM);
}


void init(void)
{        
    USBUART_Start();    
    USBUART_PutString("\n\rCOM Port Open\n\r");
    
    I2C_GY87_Start();    
    MPU6050_init();
	MPU6050_initialize();
    USBUART_PutString(MPU6050_testConnection() ? "MPU6050 connection successful\n\r" : "MPU6050 connection failed\n\n\r");   
    
    USBUART_PutString("Calbirating...\n\r");
    gyro_calibrate(500);
    USBUART_PutString("Calbiration done\n\n\r");
    
    Clock_Timer_G87_Sample_Start();
    Clock_Timer_G87_Eval_Start();
    Timer_GY87_Sample_Start();
    Timer_GY87_Eval_Start();
    
    Clock_Motor_PWM_Start();
    motor_curr_in = motor_in_sleep;
    
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
                gyroStarted = true;
                break;
            }
        }       
        
        
        if(motor_sate != MOTOR_SLEEPING && gyroStarted)
            motor_evalDirection();
        
        
        switch(motor_sate){
            case MOTOR_GO:
                motor_setDirection();
                motor_setPWM();
                break;
            case MOTOR_BRAKING:
                
                break;
            default:
                break;
        }
    }
}

/* [] END OF FILE */
