// TrafficLight.c
// Runs on LM4F120 or TM4C123
// Index implementation of a Moore finite state machine to operate
// a traffic light.
// Your Name: Ashish Pawar
// created: October 2, 2014
// last modified: April 1, 2014
// 

/* This example accompanies the book
   "Embedded Systems: Introduction to ARM Cortex M Microcontrollers",
   ISBN: 978-1469998749, Jonathan Valvano, copyright (c) 2013
   Volume 1 Program 6.8, Example 6.4
   "Embedded Systems: Real Time Interfacing to ARM Cortex M Microcontrollers",
   ISBN: 978-1463590154, Jonathan Valvano, copyright (c) 2013
   Volume 2 Program 3.1, Example 3.1

 Copyright 2013 by Jonathan W. Valvano, valvano@mail.utexas.edu
    You may use, edit, run or distribute this file
    as long as the above copyright notice remains
 THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 VALVANO SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL,
 OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 For more information about my classes, my research, and my books, see
 http://users.ece.utexas.edu/~valvano/
 */


#include "PLL.h"
#include "SysTick.h"

#define GPIO_PORTB_DATA_R       (*((volatile unsigned long *)0x400053FC)) // 
#define GPIO_PORTB_DIR_R        (*((volatile unsigned long *)0x40005400))
#define GPIO_PORTB_AFSEL_R      (*((volatile unsigned long *)0x40005420))
#define GPIO_PORTB_DEN_R        (*((volatile unsigned long *)0x4000551C))
#define GPIO_PORTB_AMSEL_R      (*((volatile unsigned long *)0x40005528))
#define GPIO_PORTB_PCTL_R       (*((volatile unsigned long *)0x4000552C))

#define GPIO_PORTC_DATA_R       (*((volatile unsigned long *)0x400063FC))
#define GPIO_PORTC_DIR_R        (*((volatile unsigned long *)0x40006400))	
#define GPIO_PORTC_AFSEL_R      (*((volatile unsigned long *)0x40006420))
#define GPIO_PORTC_DEN_R        (*((volatile unsigned long *)0x4000651C))
#define GPIO_PORTC_AMSEL_R      (*((volatile unsigned long *)0x40006528))
#define GPIO_PORTC_PCTL_R       (*((volatile unsigned long *)0x4000652C))
#define GPIO_PORTC_LOCK_R       (*((volatile unsigned long *)0x40006520))
#define GPIO_PORTC_CR_R         (*((volatile unsigned long *)0x40006524))



#define GPIO_PORTE_DATA_R       (*((volatile unsigned long *)0x400243FC)) // 
#define GPIO_PORTE_DIR_R        (*((volatile unsigned long *)0x40024400))
#define GPIO_PORTE_AFSEL_R      (*((volatile unsigned long *)0x40024420))
#define GPIO_PORTE_DEN_R        (*((volatile unsigned long *)0x4002451C))
#define GPIO_PORTE_AMSEL_R      (*((volatile unsigned long *)0x40024528))
#define GPIO_PORTE_PCTL_R       (*((volatile unsigned long *)0x4002452C))
#define GPIO_PORTE_PUR_R				(*((volatile unsigned long *)0x40024510))

#define GPIO_PORTF_DATA_R       (*((volatile unsigned long *)0x400253FC))
#define GPIO_PORTF_DIR_R        (*((volatile unsigned long *)0x40025400))
#define GPIO_PORTF_AFSEL_R      (*((volatile unsigned long *)0x40025420))
#define GPIO_PORTF_PUR_R        (*((volatile unsigned long *)0x40025510))
#define GPIO_PORTF_DEN_R        (*((volatile unsigned long *)0x4002551C))
#define GPIO_PORTF_LOCK_R       (*((volatile unsigned long *)0x40025520))
#define GPIO_PORTF_CR_R         (*((volatile unsigned long *)0x40025524))
#define GPIO_PORTF_AMSEL_R      (*((volatile unsigned long *)0x40025528))
#define GPIO_PORTF_PCTL_R       (*((volatile unsigned long *)0x4002552C))

#define SYSCTL_RCGC2_R          (*((volatile unsigned long *)0x400FE108))
#define SYSCTL_RCGC2_GPIOE      0x00000010  // port E Clock Gating Control
#define SYSCTL_RCGC2_GPIOB      0x00000002  // port B Clock Gating Control



void initializeE();
void initializeB();
void initializeF();
void initializeC();
void delay(unsigned long halfsecs);	
unsigned long takeInput();
void dispOutPut(unsigned long mOut,unsigned long sOut,unsigned long wOut,unsigned long tm);
// Linked data structure
struct State {
	unsigned long m_Out;
	unsigned long s_Out;
	unsigned long w_Out;
	unsigned long time;
	const struct State *next[8];
}; // the size of the Next array determined by the input size
typedef const struct State STyp;

void delay(unsigned long halfsecs);	
//define your states here e.g. #define stateName 0, etc.
#define GoM   &FSM[0]
#define WaitM &FSM[1]
#define GoS   &FSM[2]
#define WaitS &FSM[3]
#define Walk  &FSM[4]
//Declare your states here 
STyp FSM[5] = {
									{0x08,0x20,0x00,300,{GoM,WaitM,WaitM,WaitM,GoM,WaitM,WaitM,WaitM}},
									{0x10,0x20,0x00,100,{GoS,Walk,GoS,Walk,GoS,Walk,GoS,Walk}},
									{0x20,0x08,0x00,300,{WaitS,WaitS,GoS,WaitS,WaitS,WaitS,WaitS,WaitS}},
									{0x20,0x10,0x00,100,{GoM,Walk,GoM,Walk,GoM,Walk,GoM,Walk}},
									{0x20,0x20,0xF0,400,{GoM,Walk,GoS,GoS,GoM,GoM,GoM,GoM}},
	
								};
	
unsigned long Data;  // index to the current state 
unsigned long input; 
								
int main(void)
{ 
	STyp *c_state;
	c_state = GoM;																										//Initialize GoM as statring state
	//volatile unsigned long delay;
  PLL_Init();       // 80 MHz, Program 10.1
  SysTick_Init();   // Program 10.2
  //SYSCTL_RCGC2_R |= 0x12;      // 1) B E
  //delay = SYSCTL_RCGC2_R;      // 2) no need to unlock
	//int input;  

	// initialize PortB and PortE here. 
	initializeE();
	initializeF();
	initializeB();
	initializeC();  
	
  while(1)
	{

		/*Data = GPIO_PORTE_DATA_R & 0x03;
		if(Data == 0x02)
    GPIO_PORTE_DATA_R = 0x30;
		if(Data == 0x01)
    GPIO_PORTE_DATA_R = 0x08;
		//GPIO_PORTC_DATA_R = 0xF0;
		//while(((GPIO_PORTF_DATA_R & 0x01)== 0x01)&((GPIO_PORTF_DATA_R & 0x02)==0x02));
		//if( GPIO_PORTF_DATA_R == 0x02)
		//GPIO_PORTC_DATA_R = 0x30;
		//if(GPIO_PORTF_DATA_R == 0x01)
    //GPIO_PORTC_DATA_R = 0xC0;		
		
		//GPIO_PORTC_DATA_R = 0xFF;
		*/
			dispOutPut(c_state->m_Out,c_state->s_Out,c_state->w_Out,c_state->time); 				// pass all output values of current state
			input = takeInput();																														// Took input from all sensor 
			c_state = c_state->next[input];																									// tranfer to next state
		
		
	}	


}

void dispOutPut(unsigned long mOut,unsigned long sOut,unsigned long wOut,unsigned long tm)
{
	//pass output of each state to Data Register
	GPIO_PORTE_DATA_R = mOut;																																		
	GPIO_PORTB_DATA_R = sOut;
	GPIO_PORTC_DATA_R = wOut;
	//Wait for Specific time acording to state's wait
	SysTick_Wait10ms(tm);
}

unsigned long takeInput()
{
	unsigned long snsrInput = GPIO_PORTF_DATA_R & 0x0C;											//read input of PortF
	unsigned long w1 = GPIO_PORTF_DATA_R & 0x01;														//take w1 input
	unsigned long w2 = (GPIO_PORTF_DATA_R & 0x02)>>1;												//take w2 input and shift it by 1 to get one bit o/p
	unsigned long w3 = GPIO_PORTE_DATA_R & 0x01;														//take w3 input
	unsigned long w4 = (GPIO_PORTE_DATA_R & 0x02)>>1;												//take w4 input and shift it by 1 to get one bit o/p
	unsigned long W = w1|w2|w3|w4;																					//ORR 4 walk inputs
	unsigned long snsrInputTmp = snsrInput >> 1;														//shift input by one bit so that we get sensor inputs at bit position 2,1  
	snsrInputTmp |= W;																											// or the Walk output at bit position 0 and pass it at to sensor input to get desired input
	return(snsrInputTmp);																										// return sensor input
}
	void initializeE()
	{
		SYSCTL_RCGC2_R |= 0x10;
		delay(1);
		GPIO_PORTE_AFSEL_R = 0x00;
		GPIO_PORTE_AMSEL_R = 0x00;
		GPIO_PORTE_PCTL_R = 0x00000000;
		//GPIO_PORTE_PUR_R = 0x03;
		GPIO_PORTE_DIR_R |= 0x38;																							//port 3-5 as output for main street
		GPIO_PORTE_DIR_R &= 0xFC;																							//port 0-1 as input for W3 and W4
		GPIO_PORTE_DEN_R = 0xFF;
		
}
	
	void initializeB()
	{
		SYSCTL_RCGC2_R |= 0x02;
		delay(1);
		GPIO_PORTB_AFSEL_R = 0x00;
		GPIO_PORTB_AMSEL_R = 0x00;
		GPIO_PORTB_PCTL_R = 0x00000000;
		//GPIO_PORTE_PUR_R = 0x03;
		GPIO_PORTB_DIR_R |= 0x38;																							//port 3-5 as output for side street
		//GPIO_PORTE_DIR_R &= 0xFC;
		GPIO_PORTB_DEN_R = 0xFF;
		
                    
	}
	void initializeF()
{
 	SYSCTL_RCGC2_R |= 0x20;
	delay(1);
	GPIO_PORTF_AFSEL_R = 0x00;
	GPIO_PORTF_AMSEL_R = 0x00;
	//GPIO_PORTE_PCTL_R = 0x00000000;
	GPIO_PORTF_LOCK_R=0x4C4F434B;
	GPIO_PORTF_CR_R=0xFF;
	//GPIO_PORTE_DIR_R = 0x38;
	GPIO_PORTF_DIR_R &= 0xF0;																								//port 0-3 as input 0-1 as W1,W2 and 2-3 as S ans M sensor
	GPIO_PORTF_DEN_R = 0xFF;
}	

	void initializeC()
{
 	SYSCTL_RCGC2_R |= 0x04;
	delay(1);
	GPIO_PORTC_AFSEL_R = 0x00;
	GPIO_PORTC_AMSEL_R = 0x00;
	GPIO_PORTC_DIR_R |= 0xF0;																											//port 4-7 as output for walk
	GPIO_PORTC_DEN_R = 0xF0;
}	
void delay(unsigned long halfsecs){
  unsigned long count;
  
  while(halfsecs > 0 ) { // repeat while there are still halfsecs to delay
    count = 1538460; // 400000*0.5/0.13 that it takes 0.13 sec to count down to zero
    while (count > 0) {
      count--;
    } // This while loop takes approximately 3 cycles
    halfsecs--;
  }
	
}
