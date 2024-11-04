// ALL DEBUG OUTPUTS ARE IN P5[1-5]

#include "project.h"
#include <stdio.h>

// Speed control constants and variables
#define TARGET_SPEED 4.0          
#define WHEEL_CIRCUMFERENCE 0.5   
#define PULSES_PER_TURN 5         
#define Kp_speed 100               
#define Ki_speed 15            
                
volatile double speed = 0.0;
uint16 old = 65535;
uint16 new;
uint16 elapsed;
volatile double PWM_base = 1000;
volatile double pwm;
double err_speed;
double acc_err_speed = 0;
char strbuf[42];

// Line-following constants and variables       
#define MIDDLE_LINE 120
#define Kp_steering 10
#define Ki_steering 0.5
#define Kd_steering 0.5

double error_steering = 0;
double steeringIntegral = 0;
double steeringDerivative = 0;
double previousSteeringError = 0;
double steeringOutput = 0;
int steeringPWM = 0;
double sampledTime;

#define PWM_MIN 1000
#define PWM_CENTER 1500
#define PWM_MAX 2000


char str_buf [32];

// C Sync ISR - increments line count and signals the middle line
CY_ISR(steer_inter) {
    
    // Read the capture value from the sample timer
    sampledTime =  65535 - (double) VID_TIMER_ReadCapture();
    // Calculate steering error
    error_steering = MIDDLE_LINE - sampledTime;

    // steering calculations
    // steeringDerivative = error_steering - previousSteeringError;
    // previousSteeringError = error_steering;
    steeringOutput = PWM_CENTER + (Kp_steering * error_steering);
    
    // DEBUG
    UART_PutString("\r\n NAV INTR");
    sprintf(str_buf, "\r\n time:  %f", sampledTime);
    sprintf(str_buf, "\r\n steering error:  %f", error_steering);
    UART_PutString(str_buf);
    
    steeringPWM = (uint16)steeringOutput;
    
    // limit steering PWM within the min and max bounds
    if (steeringPWM < PWM_MIN) steeringPWM = PWM_MIN;
    if (steeringPWM > PWM_MAX) steeringPWM = PWM_MAX;
    
    // update servo PWM
    SERVO_PWM_WriteCompare(steeringPWM);
}


CY_ISR(speed_inter) {
    new = TIMER_ReadCapture();
    if (new <= old)
        elapsed = old - new;
    else
        elapsed = 65535 - new + old;
    
    speed = 1256.0 / (double) elapsed;
    err_speed = TARGET_SPEED - speed;
    acc_err_speed += err_speed;

    if (error_steering > 200)
       PWM_base = 1500; 
    
    pwm = PWM_base + Kp_speed * err_speed + Ki_speed * acc_err_speed;
    if (pwm < 500)
        pwm = 500;
    if (pwm > 2500)
        pwm = 2500;
    
    sprintf(strbuf, "%d ft/s,\r\n", (int)(speed * 1000));
    UART_PutString(strbuf);
    PWM_WriteCompare((uint16)pwm);
    TIMER_ReadStatusRegister();
    old = new;
}

// Main function
int main(void) {
    CyGlobalIntEnable;
    
    // Speed control init
    PWM_Start();
    TIMER_Start();
    UART_Start();

    // Steering control init
    VID_TIMER_Start();
    SERVO_PWM_Start();
    VDAC_Start();
    COMP_Start();
    
    // Interrupts
    INT_SAMPLE_Start();
    INT_SAMPLE_SetVector(steer_inter);
    
    HE_ISR_Start();
    HE_ISR_SetVector(speed_inter);
    
    // DEBUG
    UART_PutString("Test");

   // Main loop
    for (;;) {
    }
}