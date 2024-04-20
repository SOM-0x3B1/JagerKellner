#include "project.h"
#include "mpu6050.h"
#include "stdio.h"
#include "stdlib.h"
#include "math.h"


#define PWM_MAX 11999

typedef enum {
    MOTOR_GO,
    MOTOR_BREAKING,
    MOTOR_SLEEPING
} MotorState;


char usbOutBuf[100];

volatile float targetAngle = 0;

int16_t CAX, CAY, CAZ; //current acceleration values
int16_t CGX, CGY, CGZ; //current gyroscope values
int16_t CT;            //current temperature   
float AXoff, AYoff, AZoff; //accelerometer offset values
float GXoff, GYoff, GZoff; //gyroscope offset values
int16_t AX, AY, AZ; //acceleration floats
int16_t GX, GY, GZ; //gyroscope floats


void calibrate(int numberOfTests){
 for(int i=0; i<numberOfTests; i++)
    {   
      MPU6050_getMotion6t(&CAX, &CAY, &CAZ, &CGX, &CGY, &CGZ, &CT);
      AXoff += CAX;
      AYoff += CAY;
      AZoff += CAZ;
      GXoff += CGX;
      GYoff += CGY;
      GZoff += CGZ;
    
      CyDelay(25);
      sprintf(usbOutBuf, "\r%d / %d ", i + 1, numberOfTests);
      USBUART_PutString(usbOutBuf);
    }
    
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
    PWM_LED_Start();
    PWM_LED_BRIGHTNESS_Start();
    
    /*USBUART_Start(0, USBUART_5V_OPERATION);
    USBUART_CDC_Init();
    while(USBUART_GetConfiguration() == 0) {}*/
    USBUART_Start();    
    USBUART_PutString("COM Port Open\n\r");
    
    I2C_GY87_Start();    
    MPU6050_init();
	MPU6050_initialize();
    USBUART_PutString(MPU6050_testConnection() ? "MPU6050 connection successful\n\r" : "MPU6050 connection failed\n\n\r");
    
    USBUART_PutString("Calbirating...\n\r");
    calibrate(100);
    USBUART_PutString("Calbiration done\n\r");
}


void getGyro(){
    MPU6050_getMotion6t(&CAX, &CAY, &CAZ, &CGX, &CGY, &CGZ, &CT);
    AX = (CAX-AXoff);
    AY = (CAY-AYoff);
    AZ = (CAZ-AZoff); 
    
    GX = (CGX-GXoff);
    GY = (CGY-GYoff);
    GZ = (CGZ-GZoff);
}

/*void ftostrf (float val, char *sout) {
    int index = 0;
    
   
    if((int)val > 0){
        int decLen = log10(val);
        for (int i = 0; i < decLen; i++)
            sout[index++] = ((int)val / pow(10, decLen - i)) + '0';  
        val /= pow(10, decLen);
    } else
        sout[index++] = '0'; 
    
    sout[index++] = '.';
    
    for (int i = 0; i < 4; i++){
        val *= 10;
        int digit = (int)val % 10;
        char c = digit + '0';
        sout[index++] =  c;
    }        
    sout[index] = 0;
}*/

void printGyro(){
    /*char strAX[10] = "\0", strAY[10] = "\0", strAZ[10] = "\0", strGX[10] = "\0", strGY[10] = "\0", strGZ[10] = "\0";
    ftostrf(AX, strAX);
    ftostrf(AY, strAY);
    ftostrf(AZ, strAZ);
    ftostrf(GX, strGX);
    ftostrf(GY, strGY);
    ftostrf(GZ, strGZ);
    
    //sprintf(usbOutBuf, "AX:%.*f, AY:%.*f, AZ:%.*f || GX:%.*f, GY:%.*f, GZ:%.*f\n\r", 6,AX, 6,AY, 6,AZ, 6,GX, 6,GY, 6,GZ);
    sprintf(usbOutBuf, "AX:%s, AY:%s, AZ:%s || GX:%s, GY:%s, GZ:%s\n\r", strAX,strAY,strAZ,strGX,strGY,strGZ);
    USBUART_PutString(usbOutBuf);*/
}


void setPWM(float mL, float mR){
    int lPWM = (int)PWM_MAX * mL;
    int rPWM = 1 - ((int)PWM_MAX * mR);   
    PWM_Motor_WriteCompare1(lPWM);
    PWM_Motor_WriteCompare1(rPWM);
}




int main(void)
{
    CyGlobalIntEnable; /* Enable global interrupts. */
    
    init();       
    
    
    MotorState motorSate = MOTOR_SLEEPING;
    float motorRPercentage = 0;
    float motorLPercentage = 0;   
    
    
    for(;;)
    {
        getGyro();
        //printGyro();
        
        sprintf(usbOutBuf, "\rAX:%6d, AY:%6d, AZ:%6d || GX:%6d, GY:%6d, GZ:%6d     ", AX, AY, AZ, GX, GY, GZ);
        USBUART_PutString(usbOutBuf);   
        
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
