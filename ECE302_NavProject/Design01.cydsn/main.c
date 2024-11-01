#include "project.h"
#include <stdio.h>

// Speed control constants and variables
#define TARGET_SPEED 4.0          
#define WHEEL_CIRCUMFERENCE 0.5   
#define PULSES_PER_TURN 5         
#define Kp_speed 10               
#define Ki_speed 1.0              
                
double speed = 0.0;
uint16 old = 65535;
uint16 new;
uint16 elapsed;
double PWM_base = 30;
double pwm;
double err_speed;
double acc_err_speed = 0;
char strbuf[42];

// Line-following constants and variables       
#define MIDDLE_LINE 700             
#define BLACK_THRESHOLD 50         
#define Kp_steering 0.25            
#define Ki_steering 0.1 
#define Kd_steering 0.25 

int error_steering = 0;
double steeringIntegral = 0;
double steeringDerivative = 0;
double previousSteeringError = 0;
double steeringOutput;
char outputBuffer[64];
int steeringPWM;
    
#define PWM_MIN 60
#define PWM_CENTER 75
#define PWM_MAX 90

CY_ISR_PROTO(sampleISR);


// C Sync ISR - increments line count and signals the middle line
CY_ISR(sampleISR) {
    int Sampled_Time = VID_TIMER_ReadCapture();
    error_steering = 700 - Sampled_Time;
    steeringIntegral += error_steering;
    steeringDerivative = error_steering - previousSteeringError;
    previousSteeringError = error_steering;
    
    steeringOutput = PWM_CENTER + Kp_steering * error_steering + Ki_steering * steeringIntegral + Kd_steering * steeringDerivative;
    steeringPWM = (uint8)steeringOutput;
    
    // Limit steeringPWM within the min and max bounds
    if (steeringPWM < PWM_MIN) steeringPWM = PWM_MIN;
    if (steeringPWM > PWM_MAX) steeringPWM = PWM_MAX;
    
  SERVO_PWM_WriteCompare(steeringPWM);
}


CY_ISR(inter) {
    new = TIMER_ReadCapture();
    if (new <= old)
        elapsed = old - new;
    else
        elapsed = 65535 - new + old;
    
    speed = 1256.0 / (double) elapsed;
    err_speed = TARGET_SPEED - speed;
    acc_err_speed += err_speed;

    if (error_steering > 200)
       PWM_base = 15; 
    
    pwm = PWM_base + Kp_speed * err_speed + Ki_speed * acc_err_speed;
    if (pwm < 5)
        pwm = 5;
    if (pwm > 200)
        pwm = 200;
    
    sprintf(strbuf, "%d ft/s,\r\n", (int)(speed * 1000));
    UART_PutString(strbuf);
    PWM_WriteCompare((uint8)pwm);
    TIMER_ReadStatusRegister();
    old = new;
}


// Main function
int main(void) {
    CyGlobalIntEnable;
    
    // Speed control init
    PWM_Start();
    TIMER_Start();
    HE_ISR_Start();
    UART_Start();

    // Steering control init
    VID_TIMER_Start();
    SERVO_PWM_Start();
    VDAC_Start();
    COMP_Start();
    INT_SAMPLE_Start();
    
    INT_SAMPLE_SetVector(sampleISR);
    HE_ISR_SetVector(inter);

   // Main loop
    for (;;) {
    }
    
    
}