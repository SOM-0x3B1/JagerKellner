/*****************************************************************************
* File Name: i2cFunctions.c
* 
* Version 1.0
*
* Description:
*  
*
* Hardware Dependency:
* 	PSoC 6 Kit CY8CKIT-062
*
* Code Tested With:
* 	Creator 3.3
*
******************************************************************************
* Copyright (2016), Cypress Semiconductor Corporation.
******************************************************************************
* This software is owned by Cypress Semiconductor Corporation (Cypress) and is
* protected by and subject to worldwide patent protection (United States and
* foreign), United States copyright laws and international treaty provisions.
* Cypress hereby grants to licensee a personal, non-exclusive, non-transferable
* license to copy, use, modify, create derivative works of, and compile the
* Cypress Source Code and derivative works for the sole purpose of creating
* custom software in support of licensee product to be used only in conjunction
* with a Cypress integrated circuit as specified in the applicable agreement.
* Any reproduction, modification, translation, compilation, or representation of
* this software except as specified above is prohibited without the express
* written permission of Cypress.
*
* Disclaimer: CYPRESS MAKES NO WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, WITH
* REGARD TO THIS MATERIAL, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
* Cypress reserves the right to make changes without further notice to the
* materials described herein. Cypress does not assume any liability arising out
* of the application or use of any product or circuit described herein. Cypress
* does not authorize its products for use as critical components in life-support
* systems where a malfunction or failure may reasonably be expected to result in
* significant injury to the user. The inclusion of Cypress' product in a life-
* support systems application implies that the manufacturer assumes all risk of
* such use and in doing so indemnifies Cypress against all charges. Use may be
* limited by and subject to the applicable Cypress software license agreement.
*****************************************************************************/
#include <project.h>

uint8_t temp_value = 0;

void I2CReadBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *value)
{
	
    /******************INSERT CODE HERE******CODE 1.1*********/	
    /* Insert the code to read "length" number of bytes from the device address 
     * "devAddr" and register address "regAddr" passed to the function. Save this to
     * the location pointer by the pointer "value"  */
    
    
        uint8 i = 0;
        I2C_GY87_MasterSendStart(devAddr,0);
        I2C_GY87_MasterWriteByte(regAddr);
        I2C_GY87_MasterSendRestart(devAddr,1);
        while((length))
        {
            value[i] = I2C_GY87_MasterReadByte(I2C_GY87_ACK_DATA);
            i++;
            length--;
        }
        I2C_GY87_MasterReadByte(I2C_GY87_NAK_DATA);
        I2C_GY87_MasterSendStop();
    
    
    /****************END CODE 1.1 HERE*************************/
}

void I2CWriteBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *value) 
{
	
    /******************INSERT CODE HERE******CODE 1.2*********/
    /* Insert the code to write "length" number of bytes to the device address 
     * "devAddr" and register address "regAddr" passed to the function. The data is
     * available from the location pointer by the pointer "value"  */
    

    uint8 i = 0;
    I2C_GY87_MasterSendStart(devAddr,0);
    I2C_GY87_MasterWriteByte(regAddr);
    while(length)
    {
        I2C_GY87_MasterWriteByte(*(value + i));
        i++;
        length--;
    }
    I2C_GY87_MasterSendStop();


    
    
    /****************END CODE 1.2 HERE*************************/
}

/* Following functions are build on top of the functions defined above. */
void I2CReadByte(uint8_t devAddr, uint8_t regAddr, uint8_t *value) {
	I2CReadBytes(devAddr, regAddr, 1, value);
}

uint8_t I2CSplReadByte(uint8_t devAddr, uint8_t regAddr) {
	
    I2CReadBytes(devAddr, regAddr, 1, &temp_value);
    return(temp_value);
}

void I2CReadBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, \
                                                uint8_t length, uint8_t *value) 
{
   	uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
    I2CReadByte(devAddr, regAddr, value);
    *value &= mask;
    *value >>= (bitStart - length + 1);
}

void I2CReadBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t *value)
{
	I2CReadByte(devAddr, regAddr, value);
	*value = *value & (1 << bitNum);
}

void I2CWriteByte(uint8_t devAddr, uint8_t regAddr, uint8_t value) 
{
	I2CWriteBytes(devAddr, regAddr, 1, &value);
}

void I2CWriteBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, \
                                                  uint8_t length, uint8_t value) 
{
	uint8_t b;
	uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
	I2CReadByte(devAddr, regAddr, &b);
	value <<= (bitStart - length + 1);
	value &= mask;
	b &= ~(mask);
	b |= value;
	I2CWriteByte(devAddr, regAddr, b);	
}

void I2CWriteBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t value)
{
	uint8_t b;
	I2CReadByte(devAddr, regAddr, &b);
	b = (value != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
	I2CWriteByte(devAddr, regAddr, b);
}

