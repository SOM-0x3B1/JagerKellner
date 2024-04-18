#include "project.h"


void Init(void)
{
    I2C_GY87_Start();
    USBUART_Start(0, USBUART_5V_OPERATION);
    USBUART_CDC_Init();
    while(USBUART_GetConfiguration() == 0) {}
}

int main(void)
{
    CyGlobalIntEnable; /* Enable global interrupts. */
    
    Init();

    /* Place your initialization/startup code here (e.g. MyInst_Start()) */
    USBUART_PutString("COM Port Open");
    
    for(;;)
    {
        /* Place your application code here. */
        USBUART_PutString("ALMA\n");
    }
}

/* [] END OF FILE */
