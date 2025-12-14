/**
 * @file main.c
 *
 * @brief Main source code for the Line_Follower program.
 *
 * This file contains the main entry point for the Line_Follower program.
 * The main controller demonstrates a Line Follower robot without using an advanced algorithm.
 *
 * It interfaces the following peripherals using GPIO to demonstrate line following:
 *  - 8-Channel QTRX Sensor Array module
 *
 * Timers are used in this lab:
 *  - SysTick:  Used to generate periodic interrupts at a specified rate (1 kHz)
 *  - Timer A0: Used to generate PWM signals that will be used to drive the DC motors
 *  - Timer A1: Used to generate periodic interrupts at a specified rate (1 kHz)
 *
 * @note For more information regarding the 8-Channel QTRX Sensor Array module,
 * refer to the product page: https://www.pololu.com/product/3672
 *
 * @author
 *
 */

#include "msp.h"
#include "inc/Clock.h"
#include "inc/CortexM.h"
#include "inc/GPIO.h"
#include "inc/EUSCI_A0_UART.h"
#include "inc/Motor.h"
#include "inc/Timer_A1_Interrupt.h"
#include "inc/SysTick_Interrupt.h"
#include "inc/Reflectance_Sensor.h"

// Initialize constant PWM duty cycle values for the motors
#define PWM_NOMINAL 2500
#define SPEED_BASE    6000
#define SPEED_DELTA   2500
#define SPEED_TURN_HI 8000
#define SPEED_TURN_LO 2000
#define SPEED_SEARCH  3000
#define SPEED_BACK    3500
typedef unsigned char bool;
#define true 1
#define false 0
// Initialize a global variable for Timer A1 to keep track of elapsed time in milliseconds
static uint32_t Timer_A1_ms_elapsed = 0;

typedef enum {
    ST_FOLLOW = 0,
    ST_TURN_LEFT,
    ST_TURN_RIGHT,
    ST_TURN_AROUND,
    ST_BACK_UP,
    ST_DONE
} Robot_State;

volatile Robot_State g_state = ST_FOLLOW;
volatile bool g_explored_T = false;

typedef enum
{
    POS_CENTER             = 0b00011000,
    POS_SLIGHT_LEFT        = 0b00011100,
    POS_SLIGHT_RIGHT       = 0b00111000,
    POS_FAR_LEFT           = 0b00000001,
    POS_FAR_RIGHT          = 0b10000000,
    POS_LEFT_TURN          = 0b11111000,
    POS_RIGHT_TURN         = 0b00011111,
    POS_T_INTERSECTION     = 0b11111111,
    POS_DEAD_END           = 0b00000000
} Line_Position;

volatile Line_Position g_pos = POS_CENTER;

void Detect_Line_Position(uint8_t reflectance_sensor_data)
   {
       // LED
       P1->OUT &= ~BIT0;                  // LED1 (P1.0)
       P2->OUT &= ~(BIT0 | BIT1 | BIT2);  // RGB LED  (Red=P2.0, Green=P2.1, Blue=P2.2)

       switch(reflectance_sensor_data)
       {
           case 0x18: // 0001_1000b
               // Center
               P2->OUT |= BIT1; // Green
               break;

           case 0x1C: // 0001_1100b
               // Slightly Left from Center
               P2->OUT |= (BIT0 | BIT1); // Yellow (Red + Green)
               break;

           case 0x38: // 0011_1000b
               // Slightly Right from Center
               P2->OUT |= (BIT0 | BIT2); // Pink (Red + Blue)
               break;

           case 0x01: // 0000_0001b
               // Far Left from Center
               P2->OUT |= (BIT0 | BIT1 | BIT2); // White (R+G+B)
               break;

           case 0x80: // 1000_0000b
               // Far Right from Center
               P2->OUT |= (BIT1 | BIT2); // Sky Blue (Green + Blue)
               break;

           case 0xF8: // 1111_1000b
               // Left Turn
               P1->OUT |= BIT0;                     // LED1 On
               P2->OUT |= (BIT0 | BIT1 | BIT2);     // White
               break;

           case 0x1F: // 0001_1111b
               // Right Turn
               P1->OUT |= BIT0;         // LED1 On
               P2->OUT |= (BIT1 | BIT2); // Sky Blue (Green + Blue)
               break;

           case 0xFF: // 1111_1111b
               // T-Intersection
               P2->OUT |= BIT2; // Blue
               break;

           case 0x00: // 0000_0000b
               // Dead End
               P2->OUT |= BIT0; // Red
               break;

           default:
               // Default -> LEDs all off
               break;
       }
   }


void Timer_A1_Periodic_Task(void)
{
    // Increment Timer_A1_ms_elapsed by 1 every time the Timer A1 periodic interrupt occurs
    Timer_A1_ms_elapsed++;

    // Start the process of reading the reflectance sensor array every 10 ms (i.e. 10, 20, 30, ...)
    if ((Timer_A1_ms_elapsed % 10) == 0)
    {
        Reflectance_Sensor_Start();
    }

    // Finish reading the reflectance sensor array after 1 ms (i.e. 11, 21, 31, ...)
    if ((Timer_A1_ms_elapsed % 10) == 1)
    {
        uint8_t Reflectance_Sensor_Data = Reflectance_Sensor_End();
        Detect_Line_Position(Reflectance_Sensor_Data);
        g_pos = (Line_Position)Reflectance_Sensor_Data;

    switch(g_state){
      case ST_FOLLOW:
        switch(g_pos){
          case POS_CENTER:
            Motor_Forward(SPEED_BASE, SPEED_BASE);
            break;
          case POS_SLIGHT_LEFT:
            Motor_Forward(SPEED_BASE - SPEED_DELTA, SPEED_BASE + SPEED_DELTA);
            break;
          case POS_SLIGHT_RIGHT:
            Motor_Forward(SPEED_BASE + SPEED_DELTA, SPEED_BASE - SPEED_DELTA);
            break;
          case POS_FAR_LEFT:
            Motor_Left(SPEED_TURN_LO, SPEED_TURN_HI);
            break;
          case POS_FAR_RIGHT:
            Motor_Right(SPEED_TURN_HI, SPEED_TURN_LO);
            break;
          case POS_LEFT_TURN:
            g_state = ST_TURN_LEFT;
            break;
          case POS_RIGHT_TURN:
            g_state = ST_TURN_RIGHT;
            break;
          case POS_T_INTERSECTION:
            //
            g_explored_T = true;
            g_state = ST_TURN_LEFT;
            break;
          case POS_DEAD_END:
            //
            if(g_explored_T){
              Motor_Stop();
              g_state = ST_DONE;
            }else{
              g_state = ST_TURN_AROUND;
            }
            break;
          default:
            //
            Motor_Right(SPEED_SEARCH, SPEED_SEARCH/3);
            break;
        }
        break;

      case ST_TURN_LEFT:
        //
        Motor_Left(SPEED_TURN_LO, SPEED_TURN_HI);
        if(g_pos==POS_CENTER || g_pos==POS_SLIGHT_LEFT || g_pos==POS_SLIGHT_RIGHT){
          g_state = ST_FOLLOW;
        }
        break;

      case ST_TURN_RIGHT:
        Motor_Right(SPEED_TURN_HI, SPEED_TURN_LO);
        if(g_pos==POS_CENTER || g_pos==POS_SLIGHT_LEFT || g_pos==POS_SLIGHT_RIGHT){
          g_state = ST_FOLLOW;
        }
        break;

      case ST_TURN_AROUND:
        //
        Motor_Left(SPEED_TURN_LO, SPEED_TURN_HI);
        if(g_pos==POS_CENTER || g_pos==POS_SLIGHT_LEFT || g_pos==POS_SLIGHT_RIGHT){
          g_state = ST_FOLLOW;
        }
        break;

      case ST_BACK_UP:
        //
        Motor_Backward(SPEED_BACK, SPEED_BACK);
        //
        g_state = ST_FOLLOW;
        break;

      case ST_DONE:
        Motor_Stop();
        break;
    }



    }
}



int main(void)
{
    // Ensure that interrupts are disabled during initialization
        DisableInterrupts();

        // Initialize the 48 MHz Clock
        Clock_Init48MHz();

        // Initialize the built-in red LED and the RGB LED on the MSP432 microcontroller
        LED1_Init();
        LED2_Init();

        // Initialize the EUSCI_A0_UART module
        EUSCI_A0_UART_Init_Printf();

        // Initialize the 8-Channel QTRX Reflectance Sensor Array module
        Reflectance_Sensor_Init();

        // Initialize the SysTick timer to generate periodic interrupts every 1 ms
        SysTick_Interrupt_Init(SYSTICK_INT_NUM_CLK_CYCLES, SYSTICK_INT_PRIORITY);

        // Initialize Timer A1 periodic interrupts every 1 ms
        Timer_A1_Interrupt_Init(&Timer_A1_Periodic_Task, TIMER_A1_INT_CCR0_VALUE);

        // Enable the interrupts used by the modules
        EnableInterrupts();

        Motor_Init();


    while(1)
    {

    }
}
