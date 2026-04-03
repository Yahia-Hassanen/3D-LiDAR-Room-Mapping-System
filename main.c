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
#include "uart.h"
#include "onboardLEDs.h"
#include "VL53L1X_api.h"
#include <string.h>


// I2C Leader Control Status (MCS) Bit Masks and Configuration

#define I2C_MCS_ACK             0x00000008  
#define I2C_MCS_DATACK          0x00000008  
#define I2C_MCS_ADRACK          0x00000004  
#define I2C_MCS_STOP            0x00000004  
#define I2C_MCS_START           0x00000002 
#define I2C_MCS_ERROR           0x00000002  
#define I2C_MCS_RUN             0x00000001  
#define I2C_MCS_BUSY            0x00000001  
#define I2C_MCR_MFE             0x00000010  
#define MAXRETRIES              5         



// Global variable definitions
uint16_t dev = 0x29;    // VL53L1X I2C address
int status = 0;       // holds the API call status 
int step_tracker = 0;   // tracks stepper motor position in steps


// Initalzing I2C
void I2C_Init(void){
    // Enable clock for I2C0 & Port B
    SYSCTL_RCGCI2C_R |= SYSCTL_RCGCI2C_R0;           												
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1;          												
    
    // Wait so Port is ready 
    while((SYSCTL_PRGPIO_R&0x0002) == 0){};																		

    // Enable alternate function on PB2 and PB3
    GPIO_PORTB_AFSEL_R |= 0x0C; 

    // Configure PB3 as open drain (         																	    
    GPIO_PORTB_ODR_R |= 0x08;             																

    // Enable digital I/IO  on PB2 & PB3
    GPIO_PORTB_DEN_R |= 0x0C;             																	

    // Configure PB2,PB3 as I2C
    GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFF00FF)+0x00002200;
    
    // Enable I2C master function
    I2C0_MCR_R = I2C_MCR_MFE;       
    
    // Configure for 100 kbps clock 
    I2C0_MTPR_R = 0b0000000000000101000000000111011;                      
}


// Initalizing ports  

// PM as toggle to test bus speed
void PortM_Init(void){
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R11;
    while((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R11) == 0){};
    GPIO_PORTM_DIR_R |= 0b00000001;
    GPIO_PORTM_DEN_R |= 0x03;
    return;
}


void PortG_Init(void) {
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R6;
    while ((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R6) == 0) {};
    GPIO_PORTG_DIR_R &= 0x00;  // Configure PG0 as an input (Hi-Z) initially
    GPIO_PORTG_AFSEL_R &= ~0x01;  // Disable alternate functions on PG0 and enable digital I/O
    GPIO_PORTG_DEN_R |= 0x01;

  // Disable analog functionality on PG0
    GPIO_PORTG_AMSEL_R &= ~0x01;
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

// PJ0 and PJ1 correspond to onboard user switches SW1 and SW2
void PortJ_Init(void){
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R8;
    while((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R8) == 0){};
    GPIO_PORTJ_DIR_R &= ~0x03;
    GPIO_PORTJ_DEN_R |= 0x03;
    GPIO_PORTJ_PCTL_R &= ~0x000000FF;
    GPIO_PORTJ_AMSEL_R &= ~0x03;
    GPIO_PORTJ_PUR_R |= 0x03;   //pull up resistor so pin reads HI when button is open. Active low bc when pressed it connects pin to GND
    return;
}


// EnableInt - Enables global interrupts using an assembly instruction.

void EnableInt(void) {
  __asm("    cpsie   i\n");
}

//DisableInt - Disables global interrupts using an assembly instruction.

void DisableInt(void) {
  __asm("    cpsid   i\n");
}

// WaitForInt - Puts the processor into a low-power state until an interrupt occurs.
void WaitForInt(void) {
  __asm("    wfi\n");
}


//PortJ_Interrupt_Init - Sets up edge-triggered interrupts for Port J (PJ0 and PJ1).

void PortJ_Interrupt_Init(void) {

  GPIO_PORTJ_IS_R = 0;       // Edge-sensitive (as oppsed to level)
  GPIO_PORTJ_IBE_R = 0;      
  GPIO_PORTJ_IEV_R = 0;      // Falling edge trigger
  GPIO_PORTJ_ICR_R = 0x03;   // Clear prior interrupt flags
  GPIO_PORTJ_IM_R = 0x03;    // Unmask interrupts on PJ0 and PJ1
  
  // Enable the Port J interrupt in NVIC 
  NVIC_EN1_R = 0x00080000;
  
  // Set the interrupt priority for Port J 
  NVIC_PRI12_R = 0xA0000000;
  
  // Enable global interrupts
  EnableInt();
}


// Resets the VL53L1X sensor using the XSHUT pin on Port G.

void VL53L1X_XSHUT(void){
    GPIO_PORTG_DIR_R |= 0x01;                                        // make PG0 output
    GPIO_PORTG_DATA_R &= 0b11111110;                                 //PG0 = 0 ie. set low to reset sensor
    FlashAllLEDs();
    SysTick_Wait10ms(10);
    GPIO_PORTG_DIR_R &= ~0x01;                                            // make PG0 input
}



// Rotates the motor by a given number of steps.
void spin(int steps, int dir){
  
    uint32_t phase_delay = 600;	  								
	
	if(dir >= 0){
        // ── Clockwise Rotation ──
        for(int i = 0; i < steps; i++){
            GPIO_PORTH_DATA_R = 0b00000011;   // Coils A+B
            SysTick_Wait10us(phase_delay);
            GPIO_PORTH_DATA_R = 0b00000110;   // Coils B+C
            SysTick_Wait10us(phase_delay);
            GPIO_PORTH_DATA_R = 0b00001100;   // Coils C+D
            SysTick_Wait10us(phase_delay);
            GPIO_PORTH_DATA_R = 0b00001001;   // Coils D+A
            SysTick_Wait10us(phase_delay);
 
            // Wrap position counter at one full revolution
    
            if (step_tracker == 512)

               step_tracker = 1;
            else 
                step_tracker++;
        }
		}
	else
    {																								// CCW rotation
		for(int i=0; i<steps; i++)
        {				
            FlashLED3(1);								
			GPIO_PORTH_DATA_R = 0b00001001;   // Coils D+A
            SysTick_Wait10us(phase_delay);
            GPIO_PORTH_DATA_R = 0b00001100;   // Coils C+D
            SysTick_Wait10us(phase_delay);
            GPIO_PORTH_DATA_R = 0b00000110;   // Coils B+C
            SysTick_Wait10us(phase_delay);
            GPIO_PORTH_DATA_R = 0b00000011;   // Coils A+B
            SysTick_Wait10us(phase_delay);
			

            if (step_tracker == 512)
                step_tracker = -1;
            else 
                step_tracker--;
        }
	}
}


/*
 * GPIOJ_IRQHandler - Interrupt Service Routine for Port J.
 * Handles push-button events and sensor ranging sequence.
 */
void GPIOJ_IRQHandler(void) {
 
    // SW2 press -> Begin scan sequence
    if (GPIO_PORTJ_RIS_R & 0x02) {
 
        uint16_t Distance;
        uint16_t SignalRate;
        uint16_t AmbientRate;
        uint16_t SpadNum;
        uint8_t  RangeStatus;
        uint8_t  dataReady = 0;   // Must be 0 so first poll waits for genuine data
 
        status = VL53L1X_StartRanging(dev);   // Begin continuous ranging on sensor
 
        for (int i = 0; i < 64; i++) {
 
            // Poll until the sensor flags a new measurement is available
            while (dataReady == 0) {
                status = VL53L1X_CheckForDataReady(dev, &dataReady);
                VL53L1_WaitMs(dev, 5);
            }
            dataReady = 0;   // Reset flag so next iteration waits for fresh data
 
            // Read all measurement fields from the sensor over I2C
            status = VL53L1X_GetRangeStatus(dev, &RangeStatus);
            status = VL53L1X_GetDistance(dev, &Distance);
            FlashLED2(1);    // PN0 — measurement status LED
 
            status = VL53L1X_GetSignalRate(dev, &SignalRate);
            status = VL53L1X_GetAmbientRate(dev, &AmbientRate);
            status = VL53L1X_GetSpadNb(dev, &SpadNum);
            status = VL53L1X_ClearInterrupt(dev);   // Must clear to allow next measurement
 
            // Format distance as ASCII and send to PC via UART
            sprintf(printf_buffer, "%u\n", Distance);
            UART_printf(printf_buffer);
            FlashLED4(1);    // PF0 — UART transmission status LED
 
            SysTick_Wait10ms(1);   // Brief pause before advancing motor
 
           
            spin(8, 1);  // 64 samples x 8 steps = 512 steps = full 360 degree revolution
        }
 
        spin(512, -1);                   //  Motor back to home (CCW full revolution)
        VL53L1X_StopRanging(dev);
        status = VL53L1X_ClearInterrupt(dev);
        GPIO_PORTJ_ICR_R = 0x02;        // Clear PJ1 interrupt flag
    }
 
    // SW1 (PJ0): Stop acquisition and home motor
    if (GPIO_PORTJ_RIS_R & 0x01) {
 
        // Signal Python script to stop reading and initiate visualization
        sprintf(printf_buffer, "STOP\n");
        UART_printf(printf_buffer);
    
    // Return motor to home position based on the current step_tracker value
    if (step_tracker > 0)
      spin(step_tracker, -1);
    
      if (step_tracker < 0)
      spin(-step_tracker, 1);
    
    // Clear the interrupt flag for PJ0
    GPIO_PORTJ_ICR_R = 0x01;
  }
}


int main(void){
 
    uint8_t sensor_state = 0;
 
    PLL_Init();         
    SysTick_Init();      
    onboardLEDs_Init();  
 
    I2C_Init();         
    UART_Init();        
 
    PortM_Init();        
    PortH_Init();        
    PortG_Init();        
    PortJ_Init();        
    PortJ_Interrupt_Init(); // Falling-edge interrupts on PJ0 and PJ1
 
    // Wait for VL53L1X boot sequence to complete
    while(sensor_state == 0){
        status = VL53L1X_BootState(dev, &sensor_state);
        SysTick_Wait10ms(10);
    }
 
    FlashAllLEDs();   // Visual confirmation: sensor is booted and ready
 
    status = VL53L1X_ClearInterrupt(dev);    // Clear  stale interrupt from boot
    status = VL53L1X_SensorInit(dev);        // Load  ensor 
    Status_Check("SensorInit", status);      
 
    // Idle loop 
    while(1){
        WaitForInt();
    }
 
}