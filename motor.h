#include <stdbool.h>
#include <stdint.h>


#define MOTOR_PWM_MIN 400  // minimum value to start the motors
#define MOTOR_PWM_MAX 2600 // maximum motor PWM value  <--  PWM compare vlaue * (6V / accumulator voltage)
extern const int motor_fullThrottle;

typedef enum {
    MOTOR_GO,
    MOTOR_BRAKE,
    MOTOR_SLEEP
} MotorState;

typedef enum {
    FORWARD,
    BACWARDS
} Direction;

// motor input config
typedef struct MotorIn{
    uint8_t first;  // IN1, IN3
    uint8_t second; // IN2, IN4
} MotorIn;

extern const MotorIn motor_input_sleep;
extern const MotorIn motor_input_forward;
extern const MotorIn motor_input_backwards;
extern const MotorIn motor_input_brake;

extern volatile MotorIn motor_curr_input; // current motor input

extern volatile MotorState motor_sate;
extern volatile bool motor_firstStart; // the motor hasn't been moving before
extern volatile Direction motor_newDirection;
extern volatile Direction motor_lastDirection;
extern volatile float motor_LPercentage, motor_RPercentage; // PWM duty cicle percentage


typedef struct Encoder{
    unsigned int currCount; // current interrupt count
    unsigned int evalCount; // evaluated interrupt count
} Encoder;

extern Encoder encoderL;
extern Encoder encoderR;
extern unsigned int lastEncoderEvalL;
extern unsigned int lastEncoderEvalR;

extern volatile bool encoderEvalReady;



/// Get direction according to the current pitch
void motor_evalDirection(float PIDres);

/// Set direction on the motor controller
void motor_setDirection();


/// Calculate optimal motor speed
void motor_evalSpeed();


/// Set motor speed to 0
void motor_resetPWM();

/// Set motor speed to the calculated PWM
void motor_setPWM();