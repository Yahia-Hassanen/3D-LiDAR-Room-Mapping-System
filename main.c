// Embedded Spatial Mapping System
// Stepper Motor Control and Input Interface
// Microcontroller: TM4C1294NCPDT
// Author: Yahia Hassanen
// Date: March 2026
//
// Description:
// Firmware implementing stepper motor control, push-button interface,
// and LED status indicators for a spatial scanning system. The program
// manages motor rotation, direction control, angle resolution switching,
// and homing functionality using a polling-based state machine.


#include <stdint.h>
#include "tm4c1294ncpdt.h"
#include "PLL.h"
#include "SysTick.h"


uint8_t input = 0;
int step_tracker = 0;
int run_steps = 0;

// initalizing ports  
// PM0 and PM1 as inputs for external push buttons (active-low configuration).
void PortM_Init(void){
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R11;
    while((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R11) == 0){};
    GPIO_PORTM_DIR_R &= ~0x03;
    GPIO_PORTM_DEN_R |= 0x03;
    GPIO_PORTM_PUR_R |= 0x03;  
    return;
}

// PJ0 and PJ1 correspond to onboard user switches SW1 and SW2
void PortJ_Init(void){
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R8;
    while((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R8) == 0){};
    GPIO_PORTJ_DIR_R &= ~0x03;
    GPIO_PORTJ_DEN_R |= 0x03;
    GPIO_PORTJ_PCTL_R &= ~0x000000FF;
    GPIO_PORTJ_AMSEL_R &= ~0x03;
    GPIO_PORTJ_PUR_R |= 0x03;
    return;
}

// Controls onboard LEDs D1 (PN1) and D2 (PN0)
void PortN_Init(void){
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R12;
    while((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R12) == 0){};
    GPIO_PORTN_DIR_R |= 0x03;
    GPIO_PORTN_DEN_R |= 0x03;
    return;
}

// Controls onboard LEDs D3 (PF4) and D4 (PF0)
void PortF_Init(void){
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R5;
    while((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R5) == 0){};
    GPIO_PORTF_DIR_R |= 0x11;
    GPIO_PORTF_DEN_R |= 0x11;
    return;
}

// Inputs for stepper motor
void PortH_Init(void){
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R7;
    while((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R7) == 0){};
    GPIO_PORTH_DIR_R |= 0x0F;
    GPIO_PORTH_AFSEL_R &= ~0x0F;
    GPIO_PORTH_DEN_R |= 0x0F;
    GPIO_PORTH_AMSEL_R &= ~0x0F;
    return;
}





// Reads the state of all four push buttons and stores them in a single 4-bit variable for easier processing.
void read_input(void){
    input = (GPIO_PORTM_DATA_R & 0x03);
    input <<= 2;
    input |= (GPIO_PORTJ_DATA_R & 0x03);
}


// Rotates the motor by a given number of steps.
void spin(int steps, int dir){
  uint32_t delay = 180;	  									// value that made sense from previous labs, not too slow and not too fast making it stall)
	
	if(dir >= 0){  																			// Clockwise sequence
		for(int i=0; i<steps; i++){												// Iterate by the number of steps
			GPIO_PORTH_DATA_R = 0b00000011;									// Lab 4 full-step procedure
			SysTick_Wait10us(delay);											
			GPIO_PORTH_DATA_R = 0b00000110;													
			SysTick_Wait10us(delay);
			GPIO_PORTH_DATA_R = 0b00001100;													
			SysTick_Wait10us(delay);
			GPIO_PORTH_DATA_R = 0b00001001;													
			SysTick_Wait10us(delay);
			      
			step_tracker++;   // current position
      run_steps++;      // total motion this run

			// Otherwise, increment. To be used in the reset function
		}
	}
	else{																								// CCW rotation
		for(int i=0; i<steps; i++){												// Iterate by the number of steps
			GPIO_PORTH_DATA_R = 0b00001001;								
			SysTick_Wait10us(delay);											
			GPIO_PORTH_DATA_R = 0b00001100;													
			SysTick_Wait10us(delay);
			GPIO_PORTH_DATA_R = 0b00000110;									
			SysTick_Wait10us(delay);
			GPIO_PORTH_DATA_R = 0b00000011;						
			SysTick_Wait10us(delay);
			
			
			step_tracker--;   // current position
      run_steps++;      // total motion this run
												// Otherwise, increment. To be used in the reset function
		}
	}
}

// Returns the motor to the home position (0 degrees)
void reset(void){
	if(step_tracker > 0) spin(step_tracker, -1);
	if(step_tracker < 0) spin(step_tracker*-1, 1);
	
	step_tracker = 0;
  GPIO_PORTH_DATA_R = 0x00;
    return;

	return;
}


// Motor operating state.
// Handles direction toggle, angle toggle, home, and stop.
void ON_state(void){

    GPIO_PORTN_DATA_R = 0b00000011;   // motor running + CW
    GPIO_PORTF_DATA_R = 0b00010000;   // 11.25° mode

    int dir = 1;
    int flash = 16;

    while(1){

        spin(16, dir);

        if(run_steps % flash == 0){
            GPIO_PORTF_DATA_R |= 0b00000001;
            SysTick_Wait10us(220);
            GPIO_PORTF_DATA_R &= ~0b00000001;
        }

        // stop after one full rotation since start or last direction change
        if(run_steps >= 512){
            GPIO_PORTN_DATA_R = 0;
            GPIO_PORTF_DATA_R = 0;
            GPIO_PORTH_DATA_R = 0;
            return;
        }

        read_input();

        // BUTTON 1 : direction
        if(input == 0b1101){
            while(input == 0b1101) read_input();

            dir *= -1;
            run_steps = 0;   // reset one-rotation counter at direction change

            GPIO_PORTN_DATA_R ^= 0b00000010;
        }

        // BUTTON 2 : angle toggle
        if(input == 0b0111){
            while(input == 0b0111) read_input();

            if(flash == 16){
                flash = 16;
                GPIO_PORTF_DATA_R &= ~0b00010000;
            }
            else{
                flash = 16;
                GPIO_PORTF_DATA_R |= 0b00010000;
            }
        }

        // BUTTON 3 : home
        if(input == 0b1011){
            while(input == 0b1011) read_input();

            reset();

            GPIO_PORTN_DATA_R = 0;
            GPIO_PORTF_DATA_R = 0;
            return;
        }

        // BUTTON 0 : stop
        if(input == 0b1110){
            while(input == 0b1110) read_input();

            GPIO_PORTN_DATA_R = 0;
            GPIO_PORTF_DATA_R = 0;
            GPIO_PORTH_DATA_R = 0;
            return;
        }
    }
}


int main(void){
    
    PLL_Init();     // Initialize system clock (configured to 120 MHz)
    SysTick_Init(); // Initialize SysTick timer for delays

    PortM_Init();
    PortJ_Init();
    PortN_Init();
    PortF_Init();
    PortH_Init();

    while(1){
    // reset output ports
        GPIO_PORTN_DATA_R = 0x00;
        GPIO_PORTF_DATA_R = 0x00;
        GPIO_PORTH_DATA_R = 0x00;

        step_tracker = 0;
        run_steps = 0;

        read_input();

        if(input == 0b1110){
            while(input == 0b1110) read_input();
            ON_state();
        }
    }
    return 0;
}