#include <stdbool.h>
#include <stdint.h>



#define IMU_G_WEIGHT 0.998             // gyorscope weight
extern const double imu_accWeight;     // accelerometer weight
#define IMU_SAMPLE_INTERVAL 0.005      // 5  ms
#define IMU_EVAL_INTERVAL 0.010        // 10  ms
#define IMU_SAMPLE_COUNT 2             // min count of samples for eval

extern volatile const double radToDeg; // convert radian to degree

extern volatile bool imu_sampleReady;  // ready to evaluate gyro samples
extern volatile bool imu_evalReady;    // ready to evaluate gyro samples
extern volatile bool gyroStarted;      // the first evaluation has concluded

extern int16_t CAX, CAY, CAZ;               // current acceleration values
extern int16_t CGX, CGY, CGZ;               // current gyroscope values
extern volatile double AXoff, AYoff, AZoff; // accelerometer offset values
extern volatile double GXoff, GYoff, GZoff; // gyroscope offset values
extern volatile double AX, AY, AZ;          // averaged acceleration values
extern volatile double GX, GY, GZ;          // averaged gyroscope values
extern volatile int sampleCount;            // number of successfully collected samples
extern volatile double accPitch, accRoll;   // angles calculated exclusively from the accelerometer
extern volatile double compRoll, compPitch; // anlges calculated by the complementary filter (gyro + acc)
extern volatile double lastCompRoll, lastCompPitch; 


/// calibrate IMU offsets
/// @param numberOfTests how many samples should be gathered for calibration
void imu_calibrate(int numberOfTests);


/// calculate angles from evaluated IMU data
void imu_calcAngles(int sc);