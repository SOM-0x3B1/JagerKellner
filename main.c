#include "project.h"
#include "mpu6050.h"
#include "stdio.h"
#include "math.h"



char outBuf[100];

volatile bool sleep = false;

volatile float targetPitchAngle = 0;
volatile float targetRollAngle = 0;



typedef enum {
    GYRO_SAMPLE,
    GYRO_EVAL
} GyroState;

#define TIPPING_TRESHOLD 60
#define GYRO_EVAL_INTERVAL 0.016
volatile const float radToDeg = 180/M_PI;

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



#define MOTOR_PWM_MAX 11999

typedef enum {
    MOTOR_GO,
    MOTOR_BRAKE,
    MOTOR_SLEEP
} MotorState;

typedef enum {
    FORWARD,
    BACWARDS
} Direction;

typedef struct MotorIn{
    uint8 first;
    uint8 second;  
} MotorIn;

const MotorIn motor_input_sleep = (MotorIn) {0, 0};
const MotorIn motor_input_forward = (MotorIn) {0, 1};
const MotorIn motor_input_backwards = (MotorIn) {1, 0};
const MotorIn motor_input_brake = (MotorIn) {1, 1};

volatile MotorIn motor_curr_input;

volatile MotorState motor_sate = MOTOR_SLEEP;
volatile bool motor_firstStart = true;
volatile Direction motor_newDirection;
volatile Direction motor_lastDirection;
volatile float motor_LPercentage = 0; 
volatile float motor_RPercentage = 0;



typedef struct Encoder{
    uint currCount;
    uint evalCount;
} Encoder;

Encoder encoderL = (Encoder) {0, 0};
Encoder encoderR = (Encoder) {0, 0};
volatile bool encoderEvalReady = false;



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
            sprintf(outBuf, "\r%d / %d ", i + 1, numberOfTests);
            UART_USB_PutString(outBuf);
        }
    }
    
    sprintf(outBuf, "\r%d / %d \n\r", numberOfTests, numberOfTests);
    UART_USB_PutString(outBuf);
    
    
    AXoff = AXoff/numberOfTests;
    AYoff = AYoff/numberOfTests;
    AZoff = AZoff/numberOfTests;
    GXoff = GXoff/numberOfTests;
    GYoff = GYoff/numberOfTests;
    GZoff = GZoff/numberOfTests;
}


void gyro_getAngles(){
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


    compRoll = 0.93 * (compRoll + GX * GYRO_EVAL_INTERVAL) + 0.07 * accRoll; // Calculate the angle using a Complimentary filter
    compPitch = 0.93 * (compPitch + GY * GYRO_EVAL_INTERVAL) + 0.07 * accPitch;

    sampleCount = 0;   
}



void motor_evalDirection(){    
    if(compPitch >= 0){
        motor_newDirection = FORWARD;
        motor_curr_input = motor_input_forward;
    }
    else {
        motor_newDirection = BACWARDS;
        motor_curr_input = motor_input_backwards;
    }
    
    if(motor_firstStart)
        motor_lastDirection = motor_newDirection;
    
    if( motor_newDirection != motor_lastDirection)
        motor_sate = MOTOR_BRAKE;
    
    motor_lastDirection = motor_newDirection;
}

void motor_setDirection(){
    Pin_Motor_In_1_Write(motor_curr_input.first);
    Pin_Motor_In_2_Write(motor_curr_input.second);
    Pin_Motor_In_3_Write(motor_curr_input.first);
    Pin_Motor_In_4_Write(motor_curr_input.second);
}

void motor_evalPWM(){
    float balanceSpeed = fabsf((float)compPitch / 45);
    motor_LPercentage = balanceSpeed;
    motor_RPercentage = balanceSpeed;
}

void motor_resetPWM(){
    motor_LPercentage = 0;
    motor_RPercentage = 0;
}

void motor_setPWM(){
    int lPWM = (int)(MOTOR_PWM_MAX * motor_LPercentage);
    int rPWM = (int)(MOTOR_PWM_MAX * (1 - motor_RPercentage)); // inverted PWM
    PWM_Motor_WriteCompare1(lPWM);
    PWM_Motor_WriteCompare2(rPWM);
}



void updateLED(){
    if(sleep)
        PWM_LED_WriteCompare(50);
    else
        PWM_LED_WriteCompare(800);
}



CY_ISR(GyroSampleIT){
    Timer_GY87_Sample_STATUS;
    if(gyroState == GYRO_SAMPLE)
        MPU6050_getMotion6(&CAX, &CAY, &CAZ, &CGX, &CGY, &CGZ);
}
CY_ISR(GyroEvalIT){
    Timer_GY87_Eval_STATUS;
    gyroState = GYRO_EVAL; 
}

CY_ISR(EncoderLeftIT){ encoderL.currCount++; }
CY_ISR(EncoderRightIT){ encoderR.currCount++; }
CY_ISR(EncoderEvalIT){
    Timer_Motor_Encoder_Eval_STATUS;
    encoderL.evalCount = encoderL.currCount;
    encoderR.evalCount = encoderR.currCount;
    encoderL.currCount = 0;
    encoderR.currCount = 0;   
    encoderEvalReady = true;
}

CY_ISR(ToggleSleepIT){
    sleep = !sleep;    
    motor_sate = sleep ? MOTOR_SLEEP : MOTOR_GO;
    updateLED();
}



void init(void)
{        
    UART_USB_Start();    
    UART_USB_PutString("\n\rCOM Port Open\n\r");
    
    I2C_GY87_Start();    
    MPU6050_init();
	MPU6050_initialize();
    MPU6050_setMasterClockSpeed(13); // 400 kbps
    MPU6050_setDLPFMode(1);
    UART_USB_PutString(MPU6050_testConnection() ? "MPU6050 connection successful\n\r" : "MPU6050 connection failed\n\n\r");   
    
    
    UART_USB_PutString("Calbirating...\n\r");
    gyro_calibrate(500);
    UART_USB_PutString("Calbiration done\n\n\r");
    
    Clock_Timer_G87_Sample_Start();
    Clock_Timer_G87_Eval_Start();
    Clock_Timer_Motor_Encoder_Eval_Start();
    Timer_GY87_Sample_Start();
    Timer_GY87_Eval_Start();
    Timer_Motor_Encoder_Eval_Start();
    
    Clock_Motor_PWM_Start();
    PWM_Motor_Start();
    motor_curr_input = motor_input_sleep;
    
    PWM_LED_Start();
    PWM_LED_BRIGHTNESS_Start();
    
    
    isr_Sample_GY87_StartEx(GyroSampleIT);
    isr_Eval_GY87_StartEx(GyroEvalIT);
    isr_Inc_Motor_Encoder_L_StartEx(EncoderLeftIT);
    isr_Inc_Motor_Encoder_R_StartEx(EncoderRightIT);
    isr_Eval_Motor_Encoder_StartEx(EncoderEvalIT);
    isr_Toggle_Sleep_StartEx(ToggleSleepIT);
}



int main(void)
{   
    init();   
    
    CyGlobalIntEnable; /* Enable global interrupts. */
 
    
    for(;;) {                
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
                if(sampleCount > 0)
                   gyro_getAngles();
                
                sprintf(outBuf, "GA %d %d\n",  (int)(compPitch*100), (int)(compRoll*100)); 
                UART_USB_PutString(outBuf);
               
         
                AX = 0; AY = 0; AZ = 0;
                GX = 0; GY = 0; GZ = 0;
                
                gyroState = GYRO_SAMPLE;
                gyroStarted = true;
                break;
            }
        }       
        
        
        if(encoderEvalReady){
            sprintf(outBuf, "GS %d %d\n", encoderL.evalCount, encoderR.evalCount); 
            UART_USB_PutString(outBuf);
            encoderEvalReady = false;
        }
        
        
        if(compPitch > TIPPING_TRESHOLD || compPitch < -TIPPING_TRESHOLD ||
        compRoll > TIPPING_TRESHOLD || compRoll < -TIPPING_TRESHOLD){
            sleep = true;
            motor_sate = MOTOR_SLEEP;
            
            updateLED();
        }
        
        
        if(gyroStarted){
            switch(motor_sate){
                case MOTOR_GO:                    
                    motor_evalDirection();
                    motor_evalPWM();
                    break;
                case MOTOR_BRAKE:
                    motor_curr_input = motor_input_brake;
                    motor_resetPWM();
                    if(encoderL.evalCount == 0 && encoderR.currCount == 0)
                        motor_sate = MOTOR_GO;
                    break;
                case MOTOR_SLEEP:
                    motor_curr_input = motor_input_sleep;
                    motor_resetPWM();
                    break;
            }
            
            motor_setDirection();
            motor_setPWM();
        }
    }
}

/* [] END OF FILE */
