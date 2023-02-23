/*
 * Copyright (c) 2015, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

 /*
  *  ======== empty.c ======== */
  /* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/runtime/System.h>
/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Swi.h>
/* TI-RTOS Header files */
#include <stdbool.h>
#include <stdint.h>
#include <inttypes.h>
#include "inc/hw_memmap.h"
#include "driverlib/adc.h"
#include "driverlib/gpio.h"
#include "driverlib/pwm.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "driverlib/timer.h"
#include <string.h>
#include <math.h>

/* Board Header file */
//#include "Board.h"

#define TASKSTACKSIZE   512

Task_Struct task0Struct;
Char task0Stack[TASKSTACKSIZE];
extern const ti_sysbios_knl_Semaphore_Handle semPID;
extern Swi_Handle swi0;

//TIME CALCULATION
int s = 0;
int ms = 0;

//LINE DETECTION
int time = 0;
int SingleBlackLine = 11;
int line = 0;

//DATA COLLECTION
bool collection = false;
int sample = 0;
volatile int PrevError = 0;

volatile uint16_t buffer1[20];
volatile uint16_t buffer2[20];

volatile uint16_t *active = buffer1;
volatile uint16_t *back = buffer2;

volatile int bufferPtr = 0;

uint16_t data;

//Motor functions
void Start_Motor();
void forward();
void backward();
void Brake();
void TurnRight(float value);
void TurnLeft(float value);
void uturnCCL();

//Menu Functions
void Run();
void EmergencyStop();

//Black Line Detection Functions
int ReadLightSensor();
void BlackLineDetection();

typedef void (*function)();

struct functions {
    char command[2];
    function func;
};

struct functions commands[2] = {
    { "GO", Run },
    { "ES", EmergencyStop }
};

//Configure a 10ms timer to get the amount of time it takes robot to complete maze
void MazeTimer(){
    //Set CPU Clock to 40MHz. 400MHz PLL/2 = 200 DIV 5 = 40MHz
    SysCtlClockSet(SYSCTL_SYSDIV_5|SYSCTL_USE_PLL|SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN);
    // WTimer3 Configuration
    SysCtlPeripheralEnable(SYSCTL_PERIPH_WTIMER3);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_WTIMER3)) {
    }
    TimerClockSourceSet(WTIMER1_BASE, TIMER_CLOCK_SYSTEM);

    //Configure WTimer3B Periodic Mode
    TimerConfigure(WTIMER3_BASE, TIMER_CFG_B_PERIODIC | TIMER_CFG_SPLIT_PAIR );
    //period = (0.01 sec)
    TimerLoadSet(WTIMER3_BASE, TIMER_B, 40000000/100);
    TimerIntEnable(WTIMER3_BASE, TIMER_TIMB_TIMEOUT);

}

/*This is the interrupt handler of the 10ms timer (MazeTimer). This timer measures
how long the robot runs for:
Every time this interrupt is triggered, a counter increases by 1.
ms = this counter signifies the milliseconds that go by.
1 second = (100 * 10 milliseconds), therefore, when the counter reaches 100,
that would mean 1 second has gone by and another counter is increased by 1,
s = this counter signifies the seconds that go by.
To keep this logic consistent, we reset the ms value to 0 when it reaches 100.
*/
void milliseconds(){
    TimerIntClear(WTIMER3_BASE, TIMER_TIMB_TIMEOUT);

    if(ms == 99){
        ms = 0;
        s++;
    } else {
        ms++;
    }

}



void LEDConfig(){
    // ADD Tiva-C GPIO setup - enables port, sets pins 1-3 (RGB) pins for output
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);
}



void DataTimer(){
    //Set CPU Clock to 40MHz. 400MHz PLL/2 = 200 DIV 5 = 40MHz
    SysCtlClockSet(SYSCTL_SYSDIV_5|SYSCTL_USE_PLL|SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN);

    // Timer 2 setup code
    SysCtlPeripheralEnable(SYSCTL_PERIPH_WTIMER1);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_WTIMER1)) {
    }

    TimerClockSourceSet(WTIMER1_BASE, TIMER_CLOCK_SYSTEM);
    //Configure WTimer2 Periodic Mode
    TimerConfigure(WTIMER1_BASE, TIMER_CFG_A_PERIODIC | TIMER_CFG_SPLIT_PAIR );

    //uint32_t Period = (SysCtlClockGet() / 4000000);           // period = (100ms) = (40M)*(1/4000000) = 0.1)*
    TimerLoadSet(WTIMER1_BASE, TIMER_A, (SysCtlClockGet()/10));          // set Timer 2 period

    TimerIntEnable(WTIMER1_BASE, TIMER_TIMA_TIMEOUT);
}

/*This function starts being triggered every 100 ms once the first black
 * single line is cross. The time is enabled in the BlackLineDetection()
 * function. This function handles the data collection. First the buffer
 * is filled. Once full, the buffer is switched and the SWI is called. The
 * SWI is where the green LED is taken care of and the data is transmitter
 * CHAR by CHAR*/

void DataTimerHandler(){

    TimerIntClear(WTIMER1_BASE, TIMER_TIMA_TIMEOUT);

    //fill buffer with PrevError value calculated in PID function
    if(sample < 20){
        back[sample] = (uint16_t)abs(PrevError);
        sample++;
    } else {
        sample = 0;
    }
    //If buffer is full and back buffer is buffer 1
    if(sample == 20 && bufferPtr == 0){
        //swap buffers
        bufferPtr = 1;
        active = buffer2;
        back = buffer1;
        //post swi for data transmission
        Swi_post(swi0);
    //Else if buffer is full and back buffer is buffer 1
    } else if(sample == 20 && bufferPtr == 1) {
        //swap buffers
        bufferPtr = 0;
        active = buffer1;
        back = buffer2;
        //post swi for data transmission
        Swi_post(swi0);

    }

}

//SWI transmitting data
void TransmitData() {

    //Set port f and pin 3 & 2 for LED
    //Turn on green led, turn off blue

    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, GPIO_PIN_3);
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0x0);

    uint8_t b1;
    uint8_t b2;
    int i;

    //If we are in data collection mode (between thin line 1 and 2)
    if (collection) {


        for (i = 0; i < 20; i++) {
            //Split 16 bit integer into two 8 bit integers
            b1 = (uint8_t)((active[i] >> 8) & 0xFF);
            b2 = (uint8_t)(active[i] & 0xFF);
            //Print both 8 bit integers out
            UARTCharPut(UART1_BASE, b1);
            UARTCharPut(UART1_BASE, b2);
        }

        UARTCharPut(UART1_BASE, '2');
        UARTCharPut(UART1_BASE, '4');
        UARTCharPut(UART1_BASE, 'C');
        UARTCharPut(UART1_BASE, 'R');
        UARTCharPut(UART1_BASE, 'L');
        UARTCharPut(UART1_BASE, 'F');

    }
    //Else if we are not in data collection
    else {
        //For every remaining sample in non-full buffer
        for (i = 0; i < sample; i++) {
            //Split 16 bit integer into two 8 bit integers
            b1 = (uint8_t)((back[i] >> 8) & 0xFF);
            b2 = (uint8_t)(back[i] & 0xFF);
            //Print both 8 bit integers out
            UARTCharPut(UART1_BASE, b1);
            UARTCharPut(UART1_BASE, b2);
        }
    }
    //Turn off green led, turn on blue
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 0x0);
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2);

}

//Configure a Timer to trigger an interrupt every 5ms
void LightSensorTimerInit(){
    //Configure Timer0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_WTIMER0);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_WTIMER0)) {
    }
    uint32_t Period = SysCtlClockGet();
    //Set Clock Source
    TimerClockSourceSet(WTIMER0_BASE, TIMER_CLOCK_SYSTEM);
    //Configure WTimer2 Periodic Mode
    TimerConfigure(WTIMER0_BASE, TIMER_CFG_B_PERIODIC | TIMER_CFG_SPLIT_PAIR );
    //Load WTimer2 with a period of 0.005 s
    TimerLoadSet(WTIMER0_BASE, TIMER_B, (SysCtlClockGet()/1000) * 5);
    //Enable WTimer2 to trigger an interrupt when period is reached
    TimerIntEnable(WTIMER0_BASE, TIMER_TIMB_TIMEOUT);
    //Enable WTimer2
    TimerEnable(WTIMER0_BASE, TIMER_B);
}


/*This function Initializes the GPIO for the reflectance sensor. It
 * also charges and discharges the reflectance sensor to be able to determine
 * how long it takes the reflectance sensor to discharge.
 */
void LightSensorInit(void) {
    //Configure GPIO Pin C
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    //Configure Port C Pin 5
    GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE, GPIO_PIN_5);
    //write into pin C5. Turns on pin 5 to charge light sensor
    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, GPIO_PIN_5);
    // Wait so light sensor gets fully charged
    SysCtlDelay(200);
    // Turn pin 5 off pin 5
    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0x0);
    //Turns pin 5 into input
    GPIOPinTypeGPIOInput(GPIO_PORTC_BASE, GPIO_PIN_5);
}

//This function is just the handler for the Light SensorTimer which runs every 5 ms.
//it calls the BlackLineDetection() sensor to handle thin vs thick line and data collection
void LightSensorTimerHandler(){
    TimerIntClear(WTIMER0_BASE, TIMER_TIMB_TIMEOUT);
    BlackLineDetection();
}

//Timer Configuration for PID control. Runs every 50ms.
void ADCTimer_Init(void)
{
    //Configure WTimer2
    SysCtlPeripheralEnable(SYSCTL_PERIPH_WTIMER2);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_WTIMER2)) {
    }
    //Set Clock Source
    TimerClockSourceSet(WTIMER2_BASE, TIMER_CLOCK_SYSTEM);
    //Configure WTimer2 Periodic Mode
    TimerConfigure(WTIMER2_BASE, TIMER_CFG_PERIODIC | TIMER_CFG_SPLIT_PAIR);
    //Load WTimer2 with a period of 50 ms
    TimerLoadSet(WTIMER2_BASE, TIMER_A, 2000000);
    //Enable WTimer2 to trigger an interrupt when period is reached
    TimerIntEnable(WTIMER2_BASE, TIMER_TIMA_TIMEOUT);
    //Enable WTimer2
    TimerEnable(WTIMER2_BASE, TIMER_A);

}

/*ADC handler that post semaphore so that the PID function can be unlocked
 *every 50 ms.*/
void ADCTimer_Handler() {

    TimerIntClear(WTIMER2_BASE, TIMER_TIMA_TIMEOUT);

    //post semaphore to unlock PID function
    Semaphore_post(semPID);
}

//This is a task that is unlocked my semaphore to use PID control to navigate robot
void PID() {

    /*Initialize max steer value
     Initialize integral value*/
    float MAX = 99;
    float Integral = 0;

    while (1) {
        Semaphore_pend(semPID, BIOS_WAIT_FOREVER);
        //This triggers ADC
        ADCProcessorTrigger(ADC0_BASE, 1);

        uint32_t voltage[2];
        ADCSequenceDataGet(ADC0_BASE, 1, voltage);

        //If front sensor is to close to wall
        if(voltage[0] > 2000){
            //uint32_t voltage2[1];
            while(voltage[0] > 1500)
            {
                uturnCCL();
                ADCProcessorTrigger(ADC0_BASE, 1);
                ADCSequenceDataGet(ADC0_BASE, 1, voltage);
                //UARTprintf("Front Distance: %i\r\n", (int)voltage[0]);
            }
                //move robot forward
                GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1, 0x0);
                GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_2, 0x0);

          }
        //If there is no wall to close to front of robot continue to PID control
        else {

        //Calculate error value
        float Error = 100*((float)1500 - voltage[1]) / (float)1500;

        //initialize kp, kd, ki values
        float kp = 2;
        float ki = 0;
        float kd = 0;
        //calculate PID values
        float P = kp * Error;
        float I = ki * (Integral + Error * 0.05);
        float D = kd * (Error - PrevError) / 0.05;

        //calculate steer value
        float steerVal = fabs(P + I + D);
        //make sure steerVal does not exceed max
        if (steerVal > MAX) {
            steerVal = MAX;
        }
        //If right sensor is to far from wall
        if (Error > 0) {
            //steerVal = 0;
            TurnRight(steerVal);
        }
        //If right sensor is to close to wall
        else {
            //steerVal = 0;
            TurnLeft(steerVal);
        }

        //Record current error before it updates
        //Update integral value with updated one

        PrevError = Error;
        Integral = I;

        }

    }
}

//ADC Configuration for Front and Right Distance Sensors
void ADCinit(void) {
    //Configure the ADC0 for use
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);

    //Configure GPIO Pin B for use
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_ADC0) || !SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOB)) {
    }

    ADCClockConfigSet(ADC0_BASE, ADC_CLOCK_RATE_FULL | ADC_CLOCK_SRC_PIOSC, 1);

    GPIOPinTypeADC(GPIO_PORTB_BASE, GPIO_PIN_4 | GPIO_PIN_5);

    ADCSequenceConfigure(ADC0_BASE, 1, ADC_TRIGGER_PROCESSOR, 0);

    ADCSequenceStepConfigure(ADC0_BASE, 1, 0, ADC_CTL_CH10);
    ADCSequenceStepConfigure(ADC0_BASE, 1, 1, ADC_CTL_CH11 | ADC_CTL_END);

    ADCSequenceEnable(ADC0_BASE, 1);

}

//PWM Configuration
void PWMinit(void) {

    SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

    //Configure the PWM0 for use
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_PWM0)) {
    }

    //Set Clock for PWM
    SysCtlPWMClockSet(SYSCTL_PWMDIV_1);

    //Configure PWM Module 0 Generator 2
    PWMGenConfigure(PWM0_BASE, PWM_GEN_2, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
    //Set Period to 20kHz (N = (1 / f) * PWMClk.)
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_2, 2000);

    //Set the pulse width of PWM0
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_4, 2000); //Left Wheel
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_5, 2000); //Right Wheel

    //Configure GPIO Pin E for use
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOE)) {
    }

    //Configure GPIO Pins to send PWM to motors through Motor Driver
    GPIOPinTypePWM(GPIO_PORTE_BASE, GPIO_PIN_4);
    GPIOPinTypePWM(GPIO_PORTE_BASE, GPIO_PIN_5);
    GPIOPinConfigure(GPIO_PE4_M0PWM4); //Left Wheel
    GPIOPinConfigure(GPIO_PE5_M0PWM5); //Right Wheel

    //Enable generator 2 in module 0
    PWMGenEnable(PWM0_BASE, PWM_GEN_2);

    //Configure GPIO Pins 1 and 2 for output
    GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_1);
    GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_2);
    //Write 0 to Pins 1 and 2
    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1, 0x0);
    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_2, 0x0);

    //Configure GPIO Port A for use
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA)) {
    }
    //Configure GPIO Port A Pin 5 for mode
    GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_5);
    //Configure Mode = 1
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_5, 0x20);

}

//Terminal Output Configuration
void UART0_init() {

    SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF); //enable GPIO port for LED
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_2); //enable pin for LED PF2

    UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 115200, (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));

    // Initialize the UART for console I/O.
    UARTStdioConfig(0, 115200, SysCtlClockGet());

}

//Bluetooth Configuration
void UART1_init() {

    SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

    GPIOPinConfigure(GPIO_PB0_U1RX);
    GPIOPinConfigure(GPIO_PB1_U1TX);
    GPIOPinTypeUART(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF); //enable GPIO port for LED
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3); //enable pin for LED PF2

    UARTConfigSetExpClk(UART1_BASE, SysCtlClockGet(), 115200, (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));

    //Enable FIFO
    UARTFIFOEnable(UART1_BASE);
    UARTFIFOLevelSet(UART1_BASE, UART_FIFO_TX1_8, UART_FIFO_RX1_8);

    //Enable Interrupts
    IntMasterEnable(); //enable processor interrupts
    IntEnable(INT_UART1); //enable the UART interrupt
    UARTIntEnable(UART1_BASE, UART_INT_RX); //only enable RX and TX interrupts

    // Initialize the UART for console I/O.
    UARTStdioConfig(1, 115200, SysCtlClockGet());
}

void UART1IntHandler(void)
{
    UARTIntClear(UART1_BASE, UARTIntStatus(UART1_BASE, true)); //clear the asserted interrupts

    //UARTprintf("We have entered interrupt\n");

    while (UARTCharsAvail(UART1_BASE)) { //loop while there are chars

        char one = UARTCharGet(UART1_BASE);
        char two = UARTCharGet(UART1_BASE);

        char cmd[2] = { one, two };

        //UARTprintf("%s\n", cmd);
        int i;
        for (i = 0; i < 14; i++) {
            //if we find a command that matches
            if (strncmp(cmd, commands[i].command, 2) == 0) {

                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2); //blink LED
                SysCtlDelay(SysCtlClockGet() / 1000000); //delay ~1 ms
                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0); //turn off LED
                //run function
                (commands[i].func)();

            }
        }

    }

}

/*
 *  ======== main ========
 */
int main(void)
{
    IntMasterEnable();
    //UART0_init();
    UART1_init();
    PWMinit();
    ADCinit();
    ADCTimer_Init();
    LightSensorTimerInit();
    DataTimer();
    LEDConfig();
    MazeTimer();

    //UARTprintf("Beginning of Program:\n");
    //UARTprintf("Enter CODE: ");

    /* Start BIOS */
    BIOS_start();

    return (0);
}

/***************************MOTOR CONTROL FUNCTIONS******************************/
void Start_Motor() {
    //Enables both motors to start
    PWMOutputState(PWM0_BASE, PWM_OUT_4_BIT, true);
    PWMOutputState(PWM0_BASE, PWM_OUT_5_BIT, true);
}

void forward() {
    //Enables both wheels to go forward
    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1, 0x0);
    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_2, 0x0);
}

void backward() {
    //Enables both wheels to go backward
    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1, 0x2);
    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_2, 0x4);
}

void Brake() {
    //Stops the robot
    PWMOutputState(PWM0_BASE, PWM_OUT_4_BIT, false);
    PWMOutputState(PWM0_BASE, PWM_OUT_5_BIT, false);
}

void TurnRight(float value) {
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_4, 2000);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_5, (int)(2000 * (100 - value) * 0.01));
}

void TurnLeft(float value) {
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_4, (int)(2000 * (100-value) * 0.01));
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_5, 2000);
}

void uturnCCL() {
    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1, 0x2);
    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_2,0x0);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_4, 2000);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_5, 2000);
}

/***************************MENU FUNCTIONS************************************/
//Makes the robot start running
void Run() {

    forward();
    Start_Motor();
    TimerEnable(WTIMER3_BASE, TIMER_B); //Enables 1 ms timer
}
/*This function stops our robot using the Brake() function, disables our timer
 * used to time the robot, and prints the Total time. It is called in our
 * BlackLineDetection() function once a thick line is detected.*/
void EmergencyStop() {
    UARTprintf("Emergency Stop\n");
    Brake();
    TimerDisable(WTIMER3_BASE, TIMER_B);
    UARTprintf("Total Time: %i.%i seconds\n", s, ms);
}


/******************************BLACK LINE***********************************/
//This function returns a value used to determine if the surface is black or white
int ReadLightSensor() {

    //initializes light sensor
    LightSensorInit();

    //resets light value
    int light = 0;

    /*light will increment as long as the pin is high.
     * Light acts as a counter. The amount of time is takes the pin to go from
     * high to low determines whether it is a white or black surface. For our robot
     * if light is higher than 5000 then it is a black surface. Essentially, the
     * longer it takes to discharge the darker the surface.
     */
    while (GPIOPinRead(GPIO_PORTC_BASE, GPIO_PIN_5)) {
        light++;
    }

    return light;
}


/*This function is allows data collection to start on the first thin line
  stop at the second thin line. It also stops the robot once the robot hits a
  thick line.*/
void BlackLineDetection() {
    //Every 5ms this function will read the real-time light sensor value
    int light = ReadLightSensor();

    /*if the light value is anything above 5000 then it is above a black surface
     * a counter is started to see how many times the BlackLineDetection()
     * function determined the sensor was over a black surface (every 5ms).
     * The time value will allow us to determine whether the line was thin
     * in thick.
     */
    if (light > 5000)
    {
        time++;
    }


    /* If light < 5000 then the reflectance sensor is now reading white.
     */
    else {

        /*If time > 0 then robot just transitioned from black to white. We must
          now determine whether it is a thin or thick line.*/
        if (time > 0) {

            /*For our robot, if the interrupt has <= 11 black readings
             * then it is a single black line. We use a global variable counter
             * to determine if it is the first or second black line.
             */
            if (time <= SingleBlackLine) {

                /*if it is the first single black line, then we will turn on blue
                 * LED and start the timer used to start collecting and transmitting data.
                 * The timer enabled triggers the DataTimerHandler where buffer is filled
                 * and SWI is called.*/
                if(line == 0){
                    //Start Data Collection
                    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2);

                    collection = true;
                    //Enable Timer for data collection every 100ms
                    TimerEnable(WTIMER1_BASE, TIMER_A);
                } else if(line == 1){
                    //Stop Data Collection
                    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0x0);

                    collection = false;
                    //Disable 100ms Timer
                    TimerDisable(WTIMER1_BASE, TIMER_A);
                    Swi_post(swi0);
                }

                //increments the thin line count
                line++;

                //if time is greater than 11, then it is a thick line and robot is stopped
            }
            else {
                EmergencyStop();
            }

            //time is reset to determine thickness of next black line
            time = 0;

        }
    }

}



