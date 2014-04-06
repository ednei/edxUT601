// ***** 0. Documentation Section *****
// SwitchLEDInterface.c for Lab 8
// Runs on LM4F120/TM4C123
// Use simple programming structures in C to toggle an LED
// while a button is pressed and turn the LED on when the
// button is released.  This lab requires external hardware
// to be wired to the LaunchPad using the prototyping board.
// January 11, 2014

// Lab 8
//      Jon Valvano and Ramesh Yerraballi
//      November 21, 2013

// ***** 1. Pre-processor Directives Section *****
#include "TExaS.h"
#include "tm4c123gh6pm.h"

#define PE0 0x01 //0b 0000 0001
#define PE1 0x02 //0b 0000 0010

// ***** 2. Global Declarations Section *****

// FUNCTION PROTOTYPES: Each subroutine defined
void DisableInterrupts(void); // Disable interrupts
void EnableInterrupts(void);  // Enable interrupts

// ***** 3. Subroutines Section *****

// PE0 connected to positive logic momentary switch using 10 k ohm pull down resistor
// PE1 connected to positive logic LED through 470 ohm current limiting resistor
// To avoid damaging your hardware, ensure that your circuits match the schematic
// shown in Lab8_artist.sch (PCB Artist schematic file) or 
// Lab8_artist.pdf (compatible with many various readers like Adobe Acrobat).

/* Set PortE pin PE0 as input for switch and PE1 as output for LED.*/ 
void InitPorts(){
	volatile unsigned long delay;
	SYSCTL_RCGC2_R |= 0x10;           	// 1) activate clock for Port E
	delay = SYSCTL_RCGC2_R;           	// allow time for clock to start
	
	GPIO_PORTE_PCTL_R &= ~0x000000FF; 	// 2) Select Alternative digital function : 4 bits for each pin, set PE0 e PE1 as regular GPIO
  GPIO_PORTE_AMSEL_R &= ~(PE0 | PE1);	// 3) disable analog function on PE0 and PE1
	
	GPIO_PORTE_DIR_R &= ~(PE0); 				// 4) set PE0 direction to input
  GPIO_PORTE_DIR_R |=  (PE1);        	// 5) set PE1 direction to output
	  
  GPIO_PORTE_AFSEL_R &= ~(PE0 | PE1); // 6) regular port function
  GPIO_PORTE_DEN_R |= (PE0 | PE1);   	// 7) enable digital port
}

//generates a time*100ms delay
void Delay100ms(unsigned long time){
  unsigned long i;
  while(time > 0){
    i = 1333333;  // this number means 100ms
    while(i > 0){
      i = i - 1;
    }
    time = time - 1; // decrements every 100 ms
  }
}

// Make PE1 high
void LED_On(void){
  GPIO_PORTE_DATA_R |= PE1;
}

void LED_Toggle(){
	GPIO_PORTE_DATA_R ^= PE1;
}

// Make PA2 low
void LED_Off(void){
  GPIO_PORTE_DATA_R &= ~PE1;
}

//positive logic means PE0=1 when switch is pressed
unsigned long Switch_IsPressed(void){
  return GPIO_PORTE_DATA_R & PE0; 
}


int main(void){ 
//**********************************************************************
// The following version tests input on PE0 and output on PE1
//**********************************************************************
  TExaS_Init(SW_PIN_PE0, LED_PIN_PE1);  // activate grader and set system clock to 80 MHz
  
	EnableInterrupts();           // enable interrupts for the grader
	
	InitPorts();
	
	LED_On();
  while(1){
		Delay100ms(10);
		if(Switch_IsPressed())
		{
			LED_Toggle();
		}else{
			LED_On();
		}
  }
}
