// Motor on PA6
//    - M1PWM2, M1PWM1a

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <motor.h>
#include "wait.h"
#include "uart0.h"
#include "tm4c123gh6pm.h"

#define PWM_LOAD 1024
#define MAX_INTENSITY  (PWM_LOAD - 1)

#define MOTOR_MASK 64

//----------------------------
// Global variables
//----------------------------
uint16_t intensity;

void initMotor(void)
{
    SYSCTL_RCGCPWM_R |= SYSCTL_RCGCPWM_R1;
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R0; //Port A
    _delay_cycles(3);

    // Set up GPIO used for the motor
    GPIO_PORTA_DEN_R |= MOTOR_MASK;
    GPIO_PORTA_DIR_R |= MOTOR_MASK;
    GPIO_PORTA_AFSEL_R |= MOTOR_MASK;
    GPIO_PORTA_PCTL_R &= ~GPIO_PCTL_PA6_M;
    GPIO_PORTA_PCTL_R |= GPIO_PCTL_PA6_M1PWM2;

    SYSCTL_SRPWM_R = SYSCTL_SRPWM_R1;                // reset PWM1 module
    SYSCTL_SRPWM_R = 0;                              // leave reset state
    PWM1_1_CTL_R = 0;                                // turn-off PWM1 generator 1 (drives outs 2 and 3)
    PWM1_1_GENA_R = PWM_1_GENA_ACTCMPAD_ONE | PWM_1_GENA_ACTLOAD_ZERO;

    PWM1_1_LOAD_R = PWM_LOAD;                        // set frequency to 40 MHz sys clock / 2 / 20000 = 1 kHz
    PWM1_1_CMPA_R = 0;                               // At first, motor is off

    PWM1_1_CTL_R = PWM_1_CTL_ENABLE;                 // turn-on PWM1 generator 1
    PWM1_ENABLE_R = PWM_ENABLE_PWM2EN;

}

void setDutyCycle(uint32_t percent)
{
    PWM1_1_CMPA_R = (percent * MAX_INTENSITY)/100;
}
