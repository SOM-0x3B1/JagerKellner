// Source: https://github.com/ibrahimcahit/STM32_MPU6050_KalmanFilter/blob/main/Kalman%20Filter/mpu6050.h

/*
*************************
*                       *
* İbrahim Cahit Özdemir *
*                       *
*     October 2021      *
*                       *
*************************
*/


#ifndef KALMAN_H
#define KALMAN_H


typedef struct {
    double Q_angle;
    double Q_bias;
    double R_measure;
    double angle;
    double bias;
    double P[2][2];
} Kalman_t;


double Kalman_getAngle(Kalman_t *Kalman, double newAngle, double newRate, double dt);


#endif /* KALMAN_H_ */

/* [] END OF FILE */
