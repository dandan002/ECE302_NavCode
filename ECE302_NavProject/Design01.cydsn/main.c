// ALL DEBUG OUTPUTS ARE IN P5[1-5]

#include "project.h"
#include <stdio.h>

// Speed control constants and variables
#define TARGET_SPEED 3.65
#define WHEEL_CIRCUMFERENCE 0.5
#define PULSES_PER_TURN 5

#define Kp 10
#define Ki 1.0
#define base 120

double speed = 0.0;
double error = 0.0;
uint16 old = 65535;
uint16 new;
uint16 elapsed;
double PWM_base = 30;
double pwm;

double err;
double acc_err = 0;

char strbuf[32];

// Line-following constants and variables       
#define MIDDLE_LINE 675
double Kp_steering = 1;
double Kd_steering = -0.05;

double error_steering = 0;
double prev_steering = 0; // for kd calculation
double steeringOutput = 0;
int steeringPWM = 0;
double sampledTime;

#define PWM_MIN 1020
#define PWM_CENTER 1500
#define PWM_MAX 1980

char str_buf [32];

// C Sync ISR - increments line count and signals the middle line
CY_ISR(steer_inter) {
    
    // Read the capture value from the sample timer
    sampledTime = 65535 - (double) VID_TIMER_ReadCapture();
    
    // Calculate steering error
    error_steering = sampledTime - MIDDLE_LINE;
    double delta_error = (error_steering - prev_steering)/1400;
    
    // If turning, turn off kd
    if (error_steering > 150 || error_steering < -150) {
        Kd_steering = 0;
    } else {
        Kd_steering = -0.05;
    }
    
    // Calculate the steering output using PID
    steeringOutput = PWM_CENTER + (Kp_steering * error_steering) + (Kd_steering * delta_error);
    steeringPWM = (uint16)steeringOutput; // cast to uint16
    
    // Limit steering PWM within the min and max bounds
    if (steeringPWM < PWM_MIN) steeringPWM = PWM_MIN;
    if (steeringPWM > PWM_MAX) steeringPWM = PWM_MAX;
    
    // Store previous error
    prev_steering = error_steering;
    
    // Update servo
    SERVO_PWM_WriteCompare(steeringPWM);
    
    // DEBUG
    //UART_PutString("\r\n NAV INTR");
    sprintf(str_buf, "\r\n time:, %f,", sampledTime);
    UART_PutString(str_buf);
    /*sprintf(str_buf, "\r\n steering error:  %f", error_steering);
    UART_PutString(str_buf);
    sprintf(str_buf, "\r\n steering pwm:  %f", steeringOutput);
    UART_PutString(str_buf);
    */
}


CY_ISR(speed_inter) {
    // Read timer capture value
    new = TIMER_ReadCapture();
    
    // Calculate elapsed time, handling timer overflow
    if(new <= old)
        elapsed = old - new;
    else
        elapsed = 65535 - new + old;
    
    // Calculate speed based on elapsed time
    speed = 1256.0 / (double) elapsed;
    
    // Calculate error and accumulate for integral control
    err = TARGET_SPEED - speed;
    acc_err += err;
    
    // Calculate PWM value using PID control
    pwm = PWM_base + Kp * err + Ki * acc_err;
   
    
    // Limit PWM value to valid range
    if (pwm < 5)
        pwm = 5;
    if (pwm > 200)
        pwm = 200;

    // slow down at curves 
    /* 
    if (error_steering > 150 || error_steering < -150) {
        pwm -= 15;
    }
    */
   
    
        
    // Send speed data to XBee for telemetry
    // ft/s is really mili-ft/s
    // Cast to int because XBee output only works with int
    /*
    UART_PutString("\r\n SPEED INTER");
    sprintf(strbuf,"\r\n %f ft/s", speed);
    UART_PutString(strbuf);
    sprintf(strbuf,"\r\n pwm = %f", pwm);
    UART_PutString(strbuf);
    */
    
    // Update PWM value to control motor speed
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
    

   // Main loop
    for (;;) {
    }
}