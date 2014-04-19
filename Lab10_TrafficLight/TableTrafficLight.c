// ***** 0. Documentation Section *****
// TableTrafficLight.c for Lab 10
// Runs on LM4F120/TM4C123
// Index implementation of a Moore finite state machine to operate a traffic light.  
// Daniel Valvano, Jonathan Valvano
// November 7, 2013

// east/west red light connected to PB5
// east/west yellow light connected to PB4
// east/west green light connected to PB3
// north/south facing red light connected to PB2
// north/south facing yellow light connected to PB1
// north/south facing green light connected to PB0
// pedestrian detector connected to PE2 (1=pedestrian present)
// north/south car detector connected to PE1 (1=car present)
// east/west car detector connected to PE0 (1=car present)
// "walk" light connected to PF3 (built-in green LED)
// "don't walk" light connected to PF1 (built-in red LED)

// ***** 1. Pre-processor Directives Section *****
#include "TExaS.h"
#include "tm4c123gh6pm.h"

typedef enum t_StreetSemaphoroId{
	WEST,
	SOUTH
} StreetSemaphoroId;

typedef enum t_StreetSemaphoroState{
	STREET_SEMAPHORO_OFF,
	STREET_SEMAPHORO_RED,	
	STREET_SEMAPHORO_GREEN,
	STREET_SEMAPHORO_YELLOW
}StreetSemaphoroState;

typedef enum t_WalkSemaphoroState{
	WALK_SEMAPHORO_OFF,
	WALK_SEMAPHORO_RED,
	WALK_SEMAPHORO_GREEN
}WalkSemaphoroState;
	

//Fist letter is walk semaphoro current output, second south semaphoro, third west semaphoro
// N = Semaphoro off, R = Semaphoro Red, G = Semaphoro Green, Y = Semaphoro Yellow 
typedef enum t_IntersectionState{
	N_NRR,
	N_RRR,
	N_RRG,
	N_RRY,
	N_RGR,
	N_RYR,
	N_GRR,
	B_NRR,
	B_RRR
}IntersectionState;
#define MAX_INTERSECTION_STATES (1+B_RRR) //<= must to be always the last enum value

typedef struct t_IntersectionStateInfo{
	unsigned long StreetSemaphoro;
	unsigned long WalkSemaphoro;
	StreetSemaphoroState WestStreetSemaphoro:4;
	StreetSemaphoroState SouthStreetSemaphoro:4;
	WalkSemaphoroState walkSemaphoro:5;
	unsigned long TrasintionDelaySecs;
	IntersectionState NextState[8];
}IntersectionStateInfo;

#define TRANSACTION_DELAY_SECS 1

/*Rules: 1) Walkers have priority 2) South have priority over West 3) Semaphoro cant go from green to yellow and then back to green*/
const IntersectionStateInfo IntersectionMachine[MAX_INTERSECTION_STATES]={
// 		 	      [   000    001    010    011    100    101    110    111] <= Possible inputs:[walk,south,west]
/*{0x24,0x00,1,{N_NRR, N_RRG, N_RGR, N_RRG, N_GRR, N_GRR, N_GRR, B_RRR}}, //State N_NRR
	{0x24,0x02,1,{N_RRR, N_RRG, N_RGR, N_RRG, N_GRR, N_GRR, N_GRR, N_NRR}}, //State N_RRR 
	{0x0C,0x02,1,{N_RRG, N_RRG, N_RRY, N_RRY, N_RRY, N_RRY, N_RRY, N_RRY}}, //State N_RRG 
	{0x14,0x02,1,{N_RRY, N_RRR, N_RGR, N_RGR, N_GRR, N_GRR, N_GRR, N_RGR}}, //State N_RRY
	{0x21,0x02,1,{N_RGR, N_RYR, N_RGR, N_RYR, N_RYR, N_RYR, N_RYR, N_RYR}}, //State N_RGR
	{0x22,0x02,1,{N_RYR, N_RRG, N_RRR, N_RRG, N_GRR, N_GRR, N_GRR, N_GRR}}, //State N_RYR	
	{0x24,0x08,1,{N_GRR, N_RRR, N_RRR, N_RRR, N_RRR, N_RRR, N_RRR, N_RRR}}, //State N_GRR
	{0x24,0x00,1,{B_NRR, N_RRG, N_RGR, N_RRG, N_GRR, N_GRR, N_GRR, N_RRG}}, //State B_NRR  
	{0x24,0x02,1,{B_RRR, N_RRG, N_RGR, N_RRG, N_GRR, N_GRR, N_GRR, B_NRR}}, //State B_RRR*/
	
	{0x24,0x00,STREET_SEMAPHORO_RED		 ,STREET_SEMAPHORO_RED		,WALK_SEMAPHORO_OFF		,TRANSACTION_DELAY_SECS,{N_RRR, N_RRG, N_RGR, N_RRG, N_GRR, N_GRR, N_GRR, B_RRR}}, //State N_NRR
	{0x24,0x02,STREET_SEMAPHORO_RED		 ,STREET_SEMAPHORO_RED		,WALK_SEMAPHORO_RED		,TRANSACTION_DELAY_SECS,{N_RRR, N_RRG, N_RGR, N_RRG, N_GRR, N_GRR, N_GRR, N_NRR}}, //State N_RRR P1
	{0x0C,0x02,STREET_SEMAPHORO_GREEN  ,STREET_SEMAPHORO_RED		,WALK_SEMAPHORO_RED		,TRANSACTION_DELAY_SECS,{N_RRG, N_RRG, N_RRY, N_RRY, N_RRY, N_RRY, N_RRY, N_RRY}}, //State N_RRG P2
	{0x14,0x02,STREET_SEMAPHORO_YELLOW ,STREET_SEMAPHORO_RED		,WALK_SEMAPHORO_RED		,TRANSACTION_DELAY_SECS,{N_RRY, N_RRR, N_RGR, N_RGR, N_GRR, N_GRR, N_GRR, N_RGR}}, //State N_RRY
	{0x21,0x02,STREET_SEMAPHORO_RED    ,STREET_SEMAPHORO_GREEN	,WALK_SEMAPHORO_RED		,TRANSACTION_DELAY_SECS,{N_RGR, N_RYR, N_RGR, N_RYR, N_RYR, N_RYR, N_RYR, N_RYR}}, //State N_RGR
	{0x22,0x02,STREET_SEMAPHORO_RED    ,STREET_SEMAPHORO_YELLOW	,WALK_SEMAPHORO_RED		,TRANSACTION_DELAY_SECS,{N_RYR, N_RRG, N_RRR, N_RRG, N_GRR, N_GRR, N_GRR, N_GRR}}, //State N_RYR	
	{0x24,0x08,STREET_SEMAPHORO_RED    ,STREET_SEMAPHORO_RED		,WALK_SEMAPHORO_GREEN	,TRANSACTION_DELAY_SECS,{N_GRR, N_RRR, N_RRR, N_RRR, N_RRR, N_RRR, N_RRR, N_RRR}}, //State N_GRR
	{0x24,0x00,STREET_SEMAPHORO_RED    ,STREET_SEMAPHORO_RED		,WALK_SEMAPHORO_OFF		,TRANSACTION_DELAY_SECS,{B_NRR, N_RRG, N_RGR, N_RRG, N_GRR, N_GRR, N_GRR, N_RRG}}, //State B_NRR
	{0x24,0x02,STREET_SEMAPHORO_RED		 ,STREET_SEMAPHORO_RED		,WALK_SEMAPHORO_RED		,TRANSACTION_DELAY_SECS,{B_RRR, N_RRG, N_RGR, N_RRG, N_GRR, N_GRR, N_GRR, B_NRR}}  //State B_RRR
};

// ***** 2. Global Declarations Section *****

// FUNCTION PROTOTYPES: Each subroutine defined
void DisableInterrupts(void); // Disable interrupts
void EnableInterrupts(void);  // Enable interrupts
void Port_Init(void);
void SysTick_Init(void);
void SysTick_Wait10ms(unsigned long delay);
void UpdateSemaphoros(IntersectionStateInfo stateInfo);
unsigned long ReadSensors(void);

// ***** 3. Subroutines Section *****

int main(void){ 
	IntersectionState currentIntersectionState;
	TExaS_Init(SW_PIN_PE210, LED_PIN_PB543210); // activate grader and set system clock to 80 MHz
	Port_Init();
	SysTick_Init();
	EnableInterrupts();
	currentIntersectionState = N_RRR;
  while(1){
		UpdateSemaphoros(IntersectionMachine[currentIntersectionState]);
	  SysTick_Wait10ms(IntersectionMachine[currentIntersectionState].TrasintionDelaySecs);
		currentIntersectionState = IntersectionMachine[currentIntersectionState].NextState[ReadSensors()];
  }
}

unsigned long ReadSensors(){
	return GPIO_PORTE_DATA_R & 0x07;
}

void UpdateStreetSemaphoro(StreetSemaphoroState westSemaphoroState,StreetSemaphoroState southSemaphoroState){
	unsigned long semaphoroMask = 0x00;
		
	switch(westSemaphoroState){
			case STREET_SEMAPHORO_OFF:
				break;
			case STREET_SEMAPHORO_RED:
				semaphoroMask = 0x04;
				break;
			case STREET_SEMAPHORO_YELLOW:
				semaphoroMask = 0x02;
				break;
			case STREET_SEMAPHORO_GREEN:
			default:
				semaphoroMask = 0x01;//for security by defaulf semaphoro is red
		}
		semaphoroMask <<= 3;
		
		switch(southSemaphoroState){
			case STREET_SEMAPHORO_OFF:
				break;
			case STREET_SEMAPHORO_RED:
				semaphoroMask |= 0x04;
				break;
			case STREET_SEMAPHORO_YELLOW:
				semaphoroMask |= 0x02;
				break;
			case STREET_SEMAPHORO_GREEN:
			default:
				semaphoroMask |= 0x01;//for security by defaulf semaphoro is red
		}
		
		GPIO_PORTB_DATA_R = semaphoroMask;
}

void UpdateWalkSemaphoro(WalkSemaphoroState walkSemaphoroState ){
	switch(walkSemaphoroState){
		case WALK_SEMAPHORO_OFF:
			GPIO_PORTF_DATA_R = 0x00;
			break;
		case WALK_SEMAPHORO_GREEN:
			GPIO_PORTF_DATA_R = 0x08;
			break;
		case WALK_SEMAPHORO_RED:
		default:
			GPIO_PORTF_DATA_R = 0x02;
	}		
}

void UpdateSemaphoros(IntersectionStateInfo stateInfo){
	UpdateStreetSemaphoro(stateInfo.WestStreetSemaphoro,stateInfo.SouthStreetSemaphoro);
	UpdateWalkSemaphoro(stateInfo.walkSemaphoro);
}
	
void SysTick_Init(void){
  NVIC_ST_CTRL_R = 0;                   // disable SysTick during setup
  NVIC_ST_RELOAD_R = 0x00FFFFFF;        // maximum reload value
  NVIC_ST_CURRENT_R = 0;                // any write to current clears it             
  NVIC_ST_CTRL_R = 0x00000005;          // enable SysTick with core clock
}

// The delay parameter is in units of the 80 MHz core clock. (12.5 ns)
void SysTick_Wait(unsigned long delay){
  NVIC_ST_RELOAD_R = delay-1;  // number of counts to wait
  NVIC_ST_CURRENT_R = 0;       // any value written to CURRENT clears
  while((NVIC_ST_CTRL_R&0x00010000)==0){ // wait for count flag
  }
}
// 800000*12.5ns equals 10ms
void SysTick_Wait10ms(unsigned long delay){
  unsigned long i;
  for(i=0; i<delay; i++){
    SysTick_Wait(800000);  // wait 10ms
  }
}

//PortE will be used to read the 3 sensors input:
void PortE_Init(void)
{
	//1) Enable Port clock was already done on function Port_Init
	//2) unlock GPIO is unnecessary for Port E
  GPIO_PORTE_AMSEL_R &= ~0x07;         //3)All pins set as digital signal.
  GPIO_PORTE_PCTL_R  &= ~0x00000FFF;	//4)PE0, PE1, PE2 Pins set as standard GPIO
  GPIO_PORTE_DIR_R   &= ~0x07;      	//5)PE0, PE1, PE2 are inputs
  GPIO_PORTE_AFSEL_R &= ~0x07;        //6)PE0, PE1, PE2 don't use alernative function
  GPIO_PORTE_PUR_R   &= ~0x07;        //7)Disable pull-up on PE0, PE1, PE2
	GPIO_PORTE_PDR_R   &= ~0x07;        //8)Disable pull-down on PE0, PE1, PE2
  GPIO_PORTE_DEN_R   |=  0x07;        //9)Enable digital I/O on PE0, PE1, PE2
}

//PortB will be used to output the 3 LED values for each semaphoro:
void PortB_Init(void)
{
	//1) Enable Port clock was already done on function Port_Init
	//2) unlock GPIO is unnecessary for Port B
  GPIO_PORTB_AMSEL_R &= ~0x3F;      		//3) All pins set as digital signal.
  GPIO_PORTB_PCTL_R  &= ~0x00FFFFFF;	//4) PB0, PB1, PB2, PB3, PB4, PB5 Pins set as standard GPIO
  GPIO_PORTB_DIR_R	 |=  0x3F;       	//5) PB0, PB1, PB2, PB3, PB4, PB5 are outputs
  GPIO_PORTB_AFSEL_R &= ~0x3F;        //6) PB0, PB1, PB2, PB3, PB4, PB5 don't use alternative function
  GPIO_PORTB_PUR_R   &= ~0x3F;        //7) Disable pull-up on PB0, PB1, PB2, PB3, PB4, PB5
	GPIO_PORTB_PDR_R   &= ~0x3F;        //8) Disable pull-down on PB0, PB1, PB2, PB3, PB4, PB5
  GPIO_PORTB_DEN_R   |=  0x3F;        //9) Enable digital I/O on PB0, PB1, PB2, PB3, PB4, PB5
}

void PortF_Init(void)
{
	GPIO_PORTF_LOCK_R = 0x4C4F434B;   	// 2.1) unlock GPIO Port F
  GPIO_PORTF_CR_R = 0x1F;           	// 2.2)allow changes to PF4-0
  // only PF0 needs to be unlocked, other bits can't be locked
  GPIO_PORTF_AMSEL_R &=  0x0A;      	//3) disable analog on PB
  GPIO_PORTF_PCTL_R  &= ~0x0000F0F0;	//4)All Pins set as standard GPIO
  GPIO_PORTF_DIR_R 	 |=  0x0A;       	//5)PF1 and PF3 will be outputs
  GPIO_PORTF_AFSEL_R &= ~0x0A;      	//6)PF1 and PF3 dont use alternative function
  GPIO_PORTF_PUR_R   &= ~0x0A;      	//7)Disable pull-up on PF1 and PF3
	GPIO_PORTF_PDR_R   &= ~0x0A;      	//8)Disable pull-down on PF1 and PF3
  GPIO_PORTF_DEN_R   |=  0x0A;      	//Enable digital I/O on PF1 and PF3
}

void Port_Init(void)
{
	volatile unsigned long delay;
	//Start clock for Ports F, E and B = 0b110010= 0x32
	SYSCTL_RCGC2_R |= 0x0000032;     // 1) activate clock for Port F
  delay = SYSCTL_RCGC2_R;           // allow time for clock to start
  
	PortB_Init();
	PortE_Init();
	PortF_Init();
}


