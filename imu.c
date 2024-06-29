#include "imu.h"

#include "project.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include "mpu6050.h"
#include "comm.h"



const double imu_accWeight = 1.0 - IMU_G_WEIGHT; // accelerometer weight

volatile const double radToDeg = 180/M_PI; // convert radian to degree

volatile bool imu_sampleReady = false;    // ready to evaluate gyro samples
volatile bool imu_evalReady = false;      // ready to evaluate gyro samples
volatile bool gyroStarted = false;         // the first evaluation has concluded

int16_t CAX = 0, CAY = 0, CAZ = 0;               // current acceleration values
int16_t CGX = 0, CGY = 0, CGZ = 0;               // current gyroscope values
volatile double AXoff = 0, AYoff = 0, AZoff = 0; // accelerometer offset values
volatile double GXoff = 0, GYoff = 0, GZoff = 0; // gyroscope offset values
volatile double AX = 0, AY = 0, AZ = 0;          // averaged acceleration values
volatile double GX = 0, GY = 0, GZ = 0;          // averaged gyroscope values
volatile int sampleCount = 0;                    // number of successfully collected samples
volatile double accPitch = 0, accRoll = 0;       // angles calculated exclusively from the accelerometer
volatile double compRoll = 0, compPitch = 0;     // anlges calculated by the complementary filter (gyro + acc)
volatile double lastCompRoll = 0, lastCompPitch = 0; 


/// calibrate IMU offsets
/// @param numberOfTests how many samples should be gathered for calibration
void imu_calibrate(int numberOfTests){
    for(int i = 0; i < numberOfTests; i++) {
        MPU6050_getMotion6(&CAX, &CAY, &CAZ, &CGX, &CGY, &CGZ);
        AXoff += CAX;
        AYoff += CAY;
        AZoff += CAZ;
        GXoff += CGX;
        GYoff += CGY;
        GZoff += CGZ;
    
        CyDelay(1); // wait 1 ms
        if((i & 3) == 0) { // write only when i % 4 == 0
            sprintf(outBuf, "\r%d / %d ", i + 1, numberOfTests);
            UART_dual_PutString(true);
        }
    }
    
    sprintf(outBuf, "\r%d / %d \n\r", numberOfTests, numberOfTests);
    UART_dual_PutString(true);
    
    // average values
    AXoff = AXoff/numberOfTests;
    AYoff = AYoff/numberOfTests;
    AZoff = AZoff/numberOfTests;
    GXoff = GXoff/numberOfTests;
    GYoff = GYoff/numberOfTests;
    GZoff = GZoff/numberOfTests;
}


/// calculate angles from evaluated IMU data
void imu_calcAngles(int sc){
    AX /= sc;
    AY /= sc;
    AZ /= sc;
    GX /= sc;
    GY /= sc;
    GZ /= sc;

    AX = ((float)CAX - AXoff) / 16384.00;
    AY = ((float)CAY - AYoff) / 16384.00;
    AZ = ((float)CAZ - (AZoff - 16384)) / 16384.00;

    GX = ((float)CGX-GXoff) / 131.07;
    GY = ((float)CGY-GYoff) / 131.07;
    GZ = ((float)CGZ-GZoff) / 131.07;                                         


    // calculate the angles exclusively from the accelerometer
    accRoll  = atan2(AY, AZ) * radToDeg;
    accPitch = atan(-AX / sqrt(AY * AY + AZ * AZ)) * radToDeg;                   

    // calculate the angles using a Complimentary filter
    compRoll = IMU_G_WEIGHT * (compRoll + GX * IMU_EVAL_INTERVAL) + imu_accWeight * accRoll; 
    compPitch = IMU_G_WEIGHT * (compPitch + GY * IMU_EVAL_INTERVAL) + imu_accWeight * accPitch;
}