// Ikechukwuka Ofili

// Hardware configuration:
// Ultrasonic sensor echo input:
//    ECHO0 on PD0 (WT2CCP0)
//    ECHO1 on PD1 (WT2CCP0)
//    ECHO2 on PD2 (WT3CCP0)
//
// Ultrasonic sensor echo input:
//    TRIG0 on PE4
//    TRIG1 on PE5
//    TRIG2 on PB4

#include <stdint.h>
#include <stdbool.h>
#include <sensors.h>
#include "tm4c123gh6pm.h"
#include "wait.h"

#define TRIG0        (((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 4*4)))
#define TRIG1        (((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 5*4)))
#define TRIG2        (((volatile uint32_t *)(0x42000000 + (0x400053FC-0x40000000)*32 + 4*4)))

// Add more triggers as you go
#define TRIGGERS      ((volatile uint32_t * []){ TRIG0, TRIG1, TRIG2 })
#define TRIGGER_AT(n) (*TRIGGERS[n])

// Ultrasonic sensor masks
#define TRIG0_MASK 16
#define TRIG1_MASK 32
#define TRIG2_MASK 16

#define ECHO0_MASK 1
#define ECHO1_MASK 2
#define ECHO2_MASK 4

// Costants and conversions
#define CLK_PERIOD_MS   25e-6
#define SPEED_O_SOUND   345
#define CLKS_TO_MM(x)   (uint32_t) ( (x) * CLK_PERIOD_MS * SPEED_O_SOUND )

//-----------------------------------------------------------------------------
// Global variables
//-----------------------------------------------------------------------------
uint8_t count; //make-shift semaphore
uint32_t dist[NUM_SENSORS];
uint8_t n = 0;

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

void initSensors(void)
{
    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1;
    SYSCTL_RCGCWTIMER_R |= SYSCTL_RCGCWTIMER_R2 | SYSCTL_RCGCWTIMER_R3; //Clock the wide timers 2 and 3
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1 | SYSCTL_RCGCGPIO_R3 | SYSCTL_RCGCGPIO_R4;
    _delay_cycles(3);

    // Configure the ultra-sonic sensors' trigger pins
        GPIO_PORTE_DIR_R |= TRIG0_MASK | TRIG1_MASK; // We will be sending pulses over this so they are outputs
        GPIO_PORTE_DEN_R |= TRIG0_MASK | TRIG1_MASK;
        GPIO_PORTB_DIR_R |= TRIG2_MASK;
        GPIO_PORTB_DEN_R |= TRIG2_MASK;

        // Configure the ECHO pins
        GPIO_PORTD_DIR_R &= ~(ECHO0_MASK | ECHO1_MASK | ECHO2_MASK); // echo is an input
        GPIO_PORTD_AFSEL_R |= ECHO0_MASK | ECHO1_MASK | ECHO2_MASK;
        GPIO_PORTD_PCTL_R &= 0xFFFF000; // Keep everyone except PD0, 1 and 2
        GPIO_PORTD_PCTL_R |= GPIO_PCTL_PD0_WT2CCP0 | GPIO_PCTL_PD1_WT2CCP1 | GPIO_PCTL_PD2_WT3CCP0; // select wide timers as the alternate function
        GPIO_PORTD_DEN_R |= ECHO0_MASK | ECHO1_MASK | ECHO2_MASK;

        // Configure wide timer 2 A and B
        WTIMER2_CTL_R &= ~(TIMER_CTL_TAEN | TIMER_CTL_TBEN);
        WTIMER2_CFG_R = TIMER_CFG_16_BIT; // configure the timer for 32-bit mode

        WTIMER2_TAMR_R = TIMER_TAMR_TAMR_CAP | TIMER_TAMR_TACMR | TIMER_TAMR_TACDIR; // Configure for edge-time capture mode, count up
        WTIMER2_TBMR_R = TIMER_TBMR_TBMR_CAP | TIMER_TBMR_TBCMR | TIMER_TBMR_TBCDIR;

        WTIMER2_CTL_R &= ~(TIMER_CTL_TAEVENT_M | TIMER_CTL_TBEVENT_M); // clear any previous events
        WTIMER2_CTL_R |= TIMER_CTL_TAEVENT_BOTH | TIMER_CTL_TBEVENT_BOTH;

        WTIMER2_IMR_R = TIMER_IMR_CAEIM | TIMER_IMR_CBEIM; // enable interrupts for edge mode

        WTIMER2_CTL_R |= TIMER_CTL_TAEN | TIMER_CTL_TBEN;

        // Configure Wide timer 3
        WTIMER3_CTL_R &= ~TIMER_CTL_TAEN;
        WTIMER3_CFG_R = TIMER_CFG_16_BIT; // configure the timer for 32-bit mode

        WTIMER3_TAMR_R = TIMER_TAMR_TAMR_CAP | TIMER_TAMR_TACMR | TIMER_TAMR_TACDIR; // Configure for edge-time capture mode, count up

        WTIMER3_CTL_R &= ~TIMER_CTL_TAEVENT_M; // clear any previous events
        WTIMER3_CTL_R |= TIMER_CTL_TAEVENT_BOTH;

        WTIMER3_IMR_R = TIMER_IMR_CAEIM; // enable interrupts for edge mode

        WTIMER3_CTL_R |= TIMER_CTL_TAEN;

        // Enable the controller for the three interrupts
        NVIC_EN3_R = (1 << (INT_WTIMER2A - 16 - 96)) | (1 << (INT_WTIMER2B - 16 - 96)) | (1 << (INT_WTIMER3A - 16 - 96));

        // Configure Timer 1 as the time base
        TIMER1_CTL_R &= ~TIMER_CTL_TAEN;                 // turn-off timer before reconfiguring
        TIMER1_CFG_R = TIMER_CFG_32_BIT_TIMER;           // configure as 32-bit timer (A+B)
        TIMER1_TAMR_R = TIMER_TAMR_TAMR_PERIOD;          // configure for periodic mode (count down)
        TIMER1_TAILR_R = 4000000;                        // set load value to 4e6 for 10 Hz interrupt rate
        TIMER1_IMR_R = TIMER_IMR_TATOIM;                 // turn-on interrupts for timeout in timer module
        TIMER1_CTL_R |= TIMER_CTL_TAEN;                  // turn-on timer

        NVIC_EN0_R = 1 << (INT_TIMER1A-16);              // turn-on interrupt 37 (TIMER1A) in NVIC
}

void stopReading()
{
    TIMER1_CTL_R &= ~TIMER_CTL_TAEN;
    NVIC_DIS0_R = 1 << (INT_TIMER1A-16);
}

void restartReading()
{
    TIMER1_CTL_R |= TIMER_CTL_TAEN;
    NVIC_EN0_R = 1 << (INT_TIMER1A-16);

    count = 0;
}

void Sensor2ISR(void)
{
    if (n == 2)
    {
        dist[2] = CLKS_TO_MM(WTIMER3_TAV_R/2);
        WTIMER3_TAV_R = 0;

        count++;
    }
    WTIMER3_ICR_R |= TIMER_ICR_CAECINT;
}

void Sensor1ISR(void)
{
    if (n == 1)
    {
        dist[1] = CLKS_TO_MM(WTIMER2_TBV_R/2);
        WTIMER2_TBV_R = 0;

        count++;
    }
    WTIMER2_ICR_R |= TIMER_ICR_CBECINT;
}

void Sensor0ISR(void)
{
    if (n == 0)
    {
        dist[0] = CLKS_TO_MM(WTIMER2_TAV_R/2);
        WTIMER2_TAV_R = 0;

        count++;
    }
    WTIMER2_ICR_R |= TIMER_ICR_CAECINT;
}

void timer1ISR(void)
{
    if (count < 2) {
        dist[n] = 0;
    }

    if (++n >= NUM_SENSORS) //Get ready to measure the next one
        n = 0;

    count = 0;  // Do this now to avoid any synchronization issues

    // Pulse the trigger.
    TRIGGER_AT(n) = 1;
    waitMicrosecond(10);
    TRIGGER_AT(n) = 0;

    TIMER1_ICR_R = TIMER_ICR_TATOCINT;
}
