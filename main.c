#include "project.h"
#include "mpu6050.h"
#include "stdio.h"
#include "math.h"


volatile bool sleep = false; // suspend motors

volatile float targetPitchAngle = 0; // keep this angle to achieve balance
volatile float antiDrift_offsetAngle = 0;
volatile float antiDrift_weight = 0.005;


volatile float Kp = 165;          // (P)roportional Tuning Parameter
volatile float Ki = 0.4;           // (I)ntegral Tuning Parameter        
volatile float Kd = 140;           // (D)erivative Tuning Parameter       
volatile float iTerm = 0;        // Used to accumulate error (integral)
volatile float maxPID = 255;     // The maximum value that can be output
volatile float lastPitch = 0;    // The last sensor value
volatile float lastError = 0;    // The last error value




typedef enum {
    SOURCE_USB,
    SOURCE_BLUETOOTH,
    SOURCE_WAITING
} UART_Source;

typedef enum {
    CMD_PART_TYPE,
    CMD_PART_SPEC,
    CMD_PART_VALUE
} CommandPart;

char outBuf[100]; // UART output string buffer

volatile uint8 BTspacer = 0;

volatile bool sendGyro = true;
volatile bool sendMotor = true;

volatile UART_Source currentUARTSource = SOURCE_WAITING;


char UART_getChar(){
    switch (currentUARTSource){
        case SOURCE_USB:
            return  UART_USB_GetChar();
        case SOURCE_BLUETOOTH:
            return  UART_Bluetooth_GetChar();
        default:
            return 0;
    }
}

bool UART_isBufferEmpty(){
    if(currentUARTSource == SOURCE_USB)
        return !UART_USB_GetRxBufferSize();
    else
        return !UART_Bluetooth_GetRxBufferSize();
}

void UART_clearRxBuffer(){
    if(currentUARTSource == SOURCE_USB)
        UART_USB_ClearRxBuffer();
    else
        UART_Bluetooth_ClearRxBuffer();
}

void UART_PutRequestString(){
    if(currentUARTSource == SOURCE_USB)
        UART_USB_PutString(outBuf);
    else
        UART_Bluetooth_PutString(outBuf);    
}

void UART_PutString(){    
    if((BTspacer & 15) == 0){
        UART_Bluetooth_PutString(outBuf);
        BTspacer = 0;   
    }
    else
        BTspacer++;   
    UART_USB_PutString(outBuf);  
}





typedef enum {
    GYRO_SAMPLE, // collect samples (1000 Hz)
    GYRO_EVAL    // calculate orientation from samples (62 Hz)
} GyroState;

#define TIPPING_TRESHOLD 30                // give up control and suspend motors over this tilt angle
#define GYRO_SAMPLE_INTERVAL 0.001         // 1  ms -> 1000 Hz
#define GYRO_EVAL_INTERVAL 0.02            // 20 ms -> 50 Hz
volatile const double radToDeg = 180/M_PI; // convert radian to degree

volatile GyroState gyroState = GYRO_SAMPLE;
volatile bool gyroStarted = false;         // the first evaluation has been completed

int16_t CAX = 0, CAY = 0, CAZ = 0;               // current acceleration values
int16_t CGX = 0, CGY = 0, CGZ = 0;               // current gyroscope values
volatile double AXoff = 0, AYoff = 0, AZoff = 0; // accelerometer offset values
volatile double GXoff = 0, GYoff = 0, GZoff = 0; // gyroscope offset values
volatile double AX = 0, AY = 0, AZ = 0;          // averaged acceleration values
volatile double GX = 0, GY = 0, GZ = 0;          // averaged gyroscope values
volatile int sampleCount = 0;                    // number of successfully collected samples
volatile double accPitch = 0, accRoll = 0;       // angles calculated exclusively from the accelerometer
volatile double compRoll = 0, compPitch = 0;     // anlges calculated by the complementary filter (gyro + acc)



#define MOTOR_PWM_MIN 0  // minimum value to start the motors
#define MOTOR_PWM_MAX 2600 // maximum motor PWM value  <--  PWM compare vlaue * (6V / accumulator voltage)
const int motor_fullThrottle = MOTOR_PWM_MAX - MOTOR_PWM_MIN;

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
    uint8 first;  // IN1, IN3
    uint8 second; // IN2, IN4
} MotorIn;

const MotorIn motor_input_sleep = (MotorIn) {0, 0};
const MotorIn motor_input_forward = (MotorIn) {0, 1};
const MotorIn motor_input_backwards = (MotorIn) {1, 0};
const MotorIn motor_input_brake = (MotorIn) {1, 1};

volatile MotorIn motor_curr_input; // current motor input

volatile MotorState motor_sate = MOTOR_SLEEP;
volatile bool motor_firstStart = true; // the motor hasn't been moving before
volatile Direction motor_newDirection;
volatile Direction motor_lastDirection;
volatile float motor_LPercentage = 0, motor_RPercentage = 0; // PWM duty cicle percentage




typedef struct Encoder{
    uint currCount; // current interrupt count
    uint evalCount; // evaluated interrupt count
} Encoder;

Encoder encoderL = (Encoder) {0, 0};
Encoder encoderR = (Encoder) {0, 0};

volatile bool encoderEvalReady = false;



/// calibrate gyro offsets
/// @param numberOfTests how many samples should be gathered for calibration
void gyro_calibrate(int numberOfTests){
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
            UART_PutString();
        }
    }
    
    sprintf(outBuf, "\r%d / %d \n\r", numberOfTests, numberOfTests);
    UART_PutString();
    
    // average values
    AXoff = AXoff/numberOfTests;
    AYoff = AYoff/numberOfTests;
    AZoff = AZoff/numberOfTests;
    GXoff = GXoff/numberOfTests;
    GYoff = GYoff/numberOfTests;
    GZoff = GZoff/numberOfTests;
}


/// calculate angles from evaluated gyro data
void gyro_calcAngles(){
    AX /= sampleCount;
    AY /= sampleCount;
    AZ /= sampleCount;
    GX /= sampleCount;
    GY /= sampleCount;
    GZ /= sampleCount;

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
    compRoll = 0.995 * (compRoll + GX * GYRO_EVAL_INTERVAL) + 0.005 * accRoll; 
    compPitch = 0.995 * (compPitch + GY * GYRO_EVAL_INTERVAL) + 0.005 * accPitch;
}



/// get direction according to the current pitch
void motor_evalDirection(){    
    if(compPitch - (targetPitchAngle + antiDrift_offsetAngle) >= 0){ // tilting forwars
        motor_newDirection = FORWARD;
        motor_curr_input = motor_input_forward;
    }
    else { // tilting backwards
        motor_newDirection = BACWARDS;
        motor_curr_input = motor_input_backwards;
    }
    
    if(motor_firstStart)
        motor_lastDirection = motor_newDirection;  // skip direction change check & save current
    
    if( motor_newDirection != motor_lastDirection) // direction change check
        motor_sate = MOTOR_BRAKE;
    
    motor_lastDirection = motor_newDirection;
}

/// set direction on the motor controller
void motor_setDirection(){
    Pin_Motor_In_1_Write(motor_curr_input.first);
    Pin_Motor_In_2_Write(motor_curr_input.second);
    Pin_Motor_In_3_Write(motor_curr_input.first);
    Pin_Motor_In_4_Write(motor_curr_input.second);
}


/// calculate optimal motor speed
void motor_evalPWM(){
    // Calculate error between target and current values
	float error = (targetPitchAngle + antiDrift_offsetAngle) - compPitch;

	// Calculate the integral term
	iTerm += error;

	// Calculate the derivative term (using the simplification)
	float dTerm = error - lastError;

	// Set old variable to equal new ones
	lastError = error;

	// Multiply each term by its constant, and add it all up
	float result = (error * Kp) + (iTerm * Ki) + (dTerm * Kd);
    
    result = fabsf(result);

	// Limit PID value to maximum values
	if (result > maxPID) 
        result = maxPID;    
    
    motor_LPercentage = result / maxPID;
    motor_RPercentage = result / maxPID;
}


/// set motor speed to 0
void motor_resetPWM(){
    motor_LPercentage = 0;
    motor_RPercentage = 0;
}

/// set motor speed to the calculated PWM
void motor_setPWM(){
    int lPWM =(int)(motor_fullThrottle * motor_LPercentage);
    int rPWM =(int)(motor_fullThrottle * motor_RPercentage);
    PWM_Motor_WriteCompare1(MOTOR_PWM_MIN + lPWM);
    PWM_Motor_WriteCompare2(MOTOR_PWM_MIN + rPWM);
}



/// update the LED
void updateLED(){
    if(sleep)
        PWM_LED_WriteCompare(50);
    else
        PWM_LED_WriteCompare(800);
}



void UART_enum(){
    if(currentUARTSource != SOURCE_WAITING){
        bool fail = false;
        volatile CommandPart currPart = CMD_PART_TYPE;
        volatile char cmdType;
        volatile char cmdSpec;
        volatile int value = 0;
        bool value_negative = false;        
        
        volatile char ch;
        do {
            ch = UART_getChar();
        } while (ch == 0);
        
        while(!fail && ch != '\n' && ch != '\r'){
            switch(currPart) {
                case CMD_PART_TYPE: {
                    if(ch == 'S' || ch == 'E') {
                        cmdType = ch;
                        currPart = CMD_PART_SPEC;
                    }
                    else { 
                        UART_clearRxBuffer();
                        fail = true;
                    }
                    break;
                }
                
                case CMD_PART_SPEC: {
                    if (cmdType == 'S'){
                        if(ch == 'P' || ch == 'I' || ch == 'D' || ch == 'A') {
                            cmdSpec = ch;
                            currPart = CMD_PART_VALUE;
                        }
                        else { 
                            UART_clearRxBuffer();
                            fail = true;
                        }
                    }
                    else if (cmdType == 'E') {
                        if(ch == 'G' || ch == 'M'){
                            cmdSpec = ch;
                            currPart = CMD_PART_VALUE;
                        }
                        else{
                            UART_clearRxBuffer();
                            fail = true;
                        }
                    }
                    break;
                }
                
                case CMD_PART_VALUE: {
                    if(ch == '-')
                        value_negative = true;
                    else if(isdigit(ch)){
                        value *= 10;
                        value += ch - '0';
                    } else
                        fail = true;
                    break;
                }
            }
            if(!fail){
                do {
                    ch = UART_getChar();
                } while (ch == 0);
            }
        }        
      
        if(!fail){
            if(cmdType == 'S'){
                if(value_negative) value = -value;
                switch(cmdSpec){
                    case 'P':
                        Kp = (float)value;
                        break;
                    case 'I':
                        Ki = (float)value / 100;
                        break;
                    case 'D':
                        Kd = (float)value;
                        break;
                    case 'A':
                        targetPitchAngle = (float)value / 10;
                        break;
                }
            } 
            else if(cmdType == 'E') {
                if(cmdSpec == 'G')
                    sendGyro = (value != 0);
                else if(cmdSpec == 'M')
                    sendMotor = (value != 0);
            }
        }        
        //UART_clearRxBuffer();
        currentUARTSource = SOURCE_WAITING;
    }   
}



/// get gyro sample
CY_ISR(GyroSampleIT){
    Timer_GY87_Sample_STATUS;
    if(gyroState == GYRO_SAMPLE)
        MPU6050_getMotion6(&CAX, &CAY, &CAZ, &CGX, &CGY, &CGZ);
}
/// evaluate gyro samples 
CY_ISR(GyroEvalIT){
    Timer_GY87_Eval_STATUS;
    gyroState = GYRO_EVAL; 
}

/// encoder interrupt count
CY_ISR(EncoderLeftIT){ encoderL.currCount++; }
CY_ISR(EncoderRightIT){ encoderR.currCount++; }
/// evaluate encoder interrupt count
CY_ISR(EncoderEvalIT){
    Timer_Motor_Encoder_Eval_STATUS;
    encoderL.evalCount = encoderL.currCount;
    encoderR.evalCount = encoderR.currCount;
    encoderL.currCount = 0;
    encoderR.currCount = 0;   
    encoderEvalReady = true;
}

/// button interrupt to toggle sleep mode
CY_ISR(ToggleSleepIT){
    sleep = !sleep;    
    motor_sate = sleep ? MOTOR_SLEEP : MOTOR_GO;
    updateLED();
}


CY_ISR(UARTEvalIT){
    Timer_UART_Eval_STATUS;
    if(currentUARTSource == SOURCE_WAITING){
        if(UART_USB_GetRxBufferSize())
            currentUARTSource = SOURCE_USB;
        else if(UART_Bluetooth_GetRxBufferSize())
            currentUARTSource = SOURCE_BLUETOOTH;
        else
            currentUARTSource = SOURCE_WAITING;
    }
}



/// initialize things
void init() {        
    // init USB UART
    UART_USB_Start();    
    UART_USB_PutString("\n\rCOM Port Open\n\r");
    
    UART_Bluetooth_Start();
    /*char *config = "AT+NAME\r\n";
    UART_Bluetooth_PutString(config);*/
    UART_Bluetooth_PutString("\n\rCOM Port Open\n\r");
    
    // init gyoscope
    I2C_GY87_Start();
    MPU6050_init();
	MPU6050_initialize();
    MPU6050_setMasterClockSpeed(13); // 400 kbps
    MPU6050_setDLPFMode(1); // 184 Hz, supports 1 kHz sample rate
    sprintf(outBuf, MPU6050_testConnection() ? "MPU6050 connection successful\n\r" : "MPU6050 connection failed\n\n\r");
    UART_PutString();
    
    // calibrate gyro offsets
    sprintf(outBuf, "Calbirating...\n\r");
    UART_PutString();
    gyro_calibrate(500);
    sprintf(outBuf, "Calbiration done\n\n\r");
    UART_PutString();
    
    // start timers
    Clock_Timer_G87_Sample_Start();
    Clock_Timer_G87_Eval_Start();
    Clock_Timer_Motor_Encoder_Eval_Start();
    Clock_Timer_UART_Eval_Start();
    Timer_GY87_Sample_Start();
    Timer_GY87_Eval_Start();
    Timer_Motor_Encoder_Eval_Start();
    Timer_UART_Eval_Start();
    
    // start motor control
    Clock_Motor_PWM_Start();
    PWM_Motor_Start();
    motor_curr_input = motor_input_sleep;
    
    // start status LED 
    PWM_LED_Start();
    PWM_LED_BRIGHTNESS_Start();
    
    // register interrupts
    isr_Sample_GY87_StartEx(GyroSampleIT);
    isr_Eval_GY87_StartEx(GyroEvalIT);
    isr_Inc_Motor_Encoder_L_StartEx(EncoderLeftIT);
    isr_Inc_Motor_Encoder_R_StartEx(EncoderRightIT);
    isr_Eval_Motor_Encoder_StartEx(EncoderEvalIT);
    isr_Toggle_Sleep_StartEx(ToggleSleepIT);
    isr_UART_Eval_StartEx(UARTEvalIT);
}



int main(void) {   
    init();   
    
    CyGlobalIntEnable; /* Enable global interrupts. */
 
        
    for(;;) {              
        
        UART_enum();
        
        switch(gyroState){
            case GYRO_SAMPLE: {
                // apply offstets and accumulate samples
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
                   gyro_calcAngles();  
                
                if(sendGyro){
                    sprintf(outBuf, "GA %d %d\n\r",  (int)(compPitch*100), (int)(compRoll*100));
                    UART_PutString();
                }
               
         
                // reset samples
                AX = 0; AY = 0; AZ = 0;
                GX = 0; GY = 0; GZ = 0;           
                sampleCount = 0;   
                
                gyroState = GYRO_SAMPLE;
                gyroStarted = true;
                
                
                motor_evalDirection();
                motor_evalPWM();          
                
                
                /*if(motor_lastDirection == motor_newDirection 
                    && fabsf(motor_LPercentage + motor_RPercentage) > 0.2)
                {
                    if(motor_newDirection == FORWARD)
                        antiDrift_offsetAngle -= (motor_LPercentage + motor_RPercentage) * antiDrift_weight;
                    else
                        antiDrift_offsetAngle += (motor_LPercentage + motor_RPercentage) * antiDrift_weight;
                }*/
                
                break;
            }
        }
        
        
        if(encoderEvalReady && sendMotor){
            sprintf(outBuf, "MS %d %d\n\r", encoderL.evalCount, encoderR.evalCount); 
            UART_PutString();
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
                case MOTOR_BRAKE:
                    motor_curr_input = motor_input_sleep;
                    motor_resetPWM();
                    if(encoderL.evalCount == 0 && encoderR.currCount == 0)
                        motor_sate = MOTOR_GO;
                    break;
                case MOTOR_SLEEP:
                    motor_curr_input = motor_input_sleep;
                    motor_resetPWM();
                    break;
                default:
                    break;
            }
            
            motor_setDirection();
            motor_setPWM();
        }
    }
}

/* [] END OF FILE */
