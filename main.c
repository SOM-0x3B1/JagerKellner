#include "project.h"


#define PWM_MAX 11999

typedef enum {
    MOTOR_GO,
    MOTOR_BREAKING,
    MOTOR_SLEEPING
} MotorState;


volatile float targetAngle = 0;
volatile float currZAngle = 0;


void init(void)
{
    I2C_GY87_Start();
    Clock_Motor_PWM_Start();
    PWM_LED_Start();
    PWM_LED_BRIGHTNESS_Start();
    USBUART_Start(0, USBUART_5V_OPERATION);
    USBUART_CDC_Init();
    while(USBUART_GetConfiguration() == 0) {}
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
    

    /* Place your initialization/startup code here (e.g. MyInst_Start()) */
    USBUART_PutString("COM Port Open\n");
    
    for(;;)
    {
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
