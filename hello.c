// Philip Chan and Thien Nguyen
//      PA5 - SSI0Tx to SI
//      PA4 - SSI0Rx to 
//      PA3 - SSI0Fss to OC
//      PA2 - SSI0CLK to CL
//			PA6 - DC
//			PA7 - reset
//			PB2 - IR output

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/ssi.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"
#include "driverlib/rom.h"

// IR includes
#include "driverlib/timer.h"
#include "inc/tm4c123gh6pm.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "driverlib/debug.h"
#include "driverlib/interrupt.h"
#include "inc/hw_ints.h"
#include "inc/hw_nvic.h"

#include "Adafruit_SSD1351.h"

#define CTRL_ONE 1
#define CTRL_TWO 2
#define CTRL_THREE 3
#define CTRL_FOUR 4
#define CTRL_FIVE 5
#define CTRL_SIX 6
#define CTRL_SEVEN 7
#define CTRL_EIGHT 8
#define CTRL_NINE 9
#define CTRL_ZERO 10
#define CTRL_VUP 11
#define CTRL_VDOWN 12
#define CTRL_CHUP 13
#define CTRL_CHDOWN 14
#define CTRL_MUTE 15
#define CTRL_ENTER 16
#define CTRL_UP 17
#define CTRL_DOWN 18
#define CTRL_LEFT 19
#define CTRL_RIGHT 20
#define CTRL_ERROR -1


// IR detection
static int interrupted = 0;
static int edge1 = 0;
static int edge2 = 0;
static int total_time = 0;
static int count = 0;
static int times[52];
static int done_flag = 0;
static int last_time = 0;
static int start_flag = 0;

// Char detection and selection
static int repetitions = 0;
static int print_code = 0;
static int last_print_code = 0;
static int error_sequence = 0;
static int error_address = 0;
static int display_now = 0;
static int valid = 0;

#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
}
#endif

#define NUM_SSI_DATA            3
#define	BLACK           0x0000
#define	BLUE            0x001F
#define	RED             0xF800
#define	GREEN           0x07E0
#define CYAN            0x07FF
#define MAGENTA         0xF81F
#define YELLOW          0xFFE0  
#define WHITE           0xFFFF

void ConfigureUART(void);

void IR_Handler (void);

void decode(int times[], int size);

void Timer1A_Int(void);

char char_selector(int code, int reps);

// ===========================================================================================
// Main

int main(void)
{
		ConfigureUART();
    ROM_FPULazyStackingEnable();
		int ir_input;
		char displayed_text;
	
	
    //
    // Set the clocking to run directly from the crystal.
    //
    ROM_SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);


		ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);


		ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
		ROM_TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
		ROM_TimerLoadSet(TIMER0_BASE, TIMER_A, SysCtlClockGet());
		TimerEnable(TIMER0_BASE, TIMER_A);
	
		ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);
		ROM_TimerConfigure(TIMER1_BASE, TIMER_CFG_ONE_SHOT);
		ROM_TimerLoadSet(TIMER1_BASE, TIMER_A, SysCtlClockGet()*2);
		//TimerEnable(TIMER1_BASE, TIMER_A);
		
		

		GPIOPinTypeGPIOInput(GPIO_PORTB_BASE, GPIO_PIN_2);
		

		GPIOPadConfigSet(GPIO_PORTB_BASE, GPIO_PIN_2, GPIO_STRENGTH_4MA, GPIO_PIN_TYPE_STD);
		
		
		GPIOIntTypeSet(GPIO_PORTB_BASE, GPIO_PIN_2, GPIO_FALLING_EDGE);

		GPIOIntEnable(GPIO_PORTB_BASE, GPIO_INT_PIN_2);
		
		
		
		
		IntEnable(INT_GPIOB);
		IntEnable(INT_TIMER1A);
		TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
		
		IntMasterEnable();
		
		
		
		
		UARTprintf("Starting Program...\n\n");
		
		
// =================================================================================================
// While
		
		
    while(1)
    {               
      
			ROM_SysCtlSleep();
			
			
			if (done_flag == 1)
			{
				decode(times, count);
				done_flag = 0;
				count = 0;
				start_flag = 0;
			}
			
			if(!display_now)	// if not in the process of displaying
			{
				if (print_code != 0 && valid == 1)	// if valid sequence found from button input
				{	// if switching numbers, print digit immediately
					if (last_print_code != 0 && print_code != last_print_code)
					{
						UARTprintf("%c",displayed_text);
						TimerEnable(TIMER1_BASE, TIMER_A);
						ROM_TimerLoadSet(TIMER1_BASE, TIMER_A, SysCtlClockGet());
						repetitions = 0;
					}
					else	// otherwise, reset the 3 second timer
					{
					
						TimerEnable(TIMER1_BASE, TIMER_A);
						ROM_TimerLoadSet(TIMER1_BASE, TIMER_A, SysCtlClockGet());
					}
					repetitions++;	// choose next character in set
					displayed_text = char_selector(print_code,repetitions);
					last_print_code = print_code;
					valid = 0;
				}
			}
			else
			{
				UARTprintf("char %c\n",displayed_text);
				/*UARTprintf("repts %i\n",repetitions);
				UARTprintf("last code %i\n",last_print_code);
				UARTprintf("current code %i\n",print_code);
				UARTprintf("display? %i\n\n",print_code);*/
				repetitions = 0;
				last_print_code = 0;
				print_code = 0;
				display_now = 0;
			}
			
			
			
			
			GPIOIntClear(GPIO_PORTB_BASE, GPIO_INT_PIN_2);
			
			TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
			GPIOIntEnable(GPIO_PORTB_BASE, GPIO_INT_PIN_2);
    }
}


// ==================================================================================================
// Functions
// ==================================================================================================


void ConfigureUART(void)
{
    //
    // Enable the GPIO Peripheral used by the UART.
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    //
    // Enable UART0
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    //
    // Configure GPIO Pins for UART mode.
    //
    ROM_GPIOPinConfigure(GPIO_PA0_U0RX);
    ROM_GPIOPinConfigure(GPIO_PA1_U0TX);
    ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    //
    // Use the internal 16MHz oscillator as the UART clock source.
    //
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);

    //
    // Initialize the UART for console I/O.
    //
    UARTStdioConfig(0, 115200, 16000000);
}


void Timer1A_Int(void)
{
	TimerIntDisable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
	TimerDisable(TIMER1_BASE, TIMER_A);
	TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
	
	display_now = 1;
}


void IR_Handler (void) {
	// Disable interrupt for awhile to avoid switch bounce
	GPIOIntDisable(GPIO_PORTB_BASE, GPIO_INT_PIN_2);

	// Clear interrupt request
	GPIOIntClear(GPIO_PORTB_BASE, GPIO_INT_PIN_2);
	
// Get pulse width
	edge2 = edge1;
	edge1 = TimerValueGet(TIMER0_BASE,TIMER_A);
	total_time = edge2 - edge1;
	if (total_time < 0)
		total_time = total_time + SysCtlClockGet();
	
// assign time	
	if (count < 60)
		times[count] = total_time;
	else
		done_flag = 1;
	
// check if continuously held
	if (last_time != 425000)
	{
		if (total_time > 400000 && total_time < 410000)
		{
			start_flag = 1;
		}
		else
		{
			if ((last_time + total_time) > 420000 && (last_time + total_time) < 430000)
				start_flag = 1;
		}
	}
	if (total_time > 420000 && total_time < 430000)
	{
		total_time = 425000;
		done_flag = 1;
	}
	else
	{
		if ((last_time + total_time) > 420000 && (last_time + total_time) < 430000)
		{
			total_time = 425000;
			done_flag = 1;
		}
	}
// if not continuously held
	if (start_flag == 1)
	{
		count = count + 1;
	}
	
	last_time = total_time;
}	


void decode(int times[], int size)
{
	int value[60];
	int index = 0;
	int start = 0;
	int sequence = 0;
	int address = 0;
	
	
			for (int i=0; i<size;i++)
			{
				if (times[i] > 123000 && times[i] < 127000)
				{
					value[index] = 1;
					index = index + 1;
				}
		else
		{
			if (times[i] > 73000 && times[i] < 77000)
			{
				value[index] = 0;
				index = index + 1;
			}
			else
			{
				if ((times[i]+times[i+1]) > 123000 && (times[i]+times[i+1]) < 127000)
				{
					value[index] = 1;
					index = index+1;
					i++;
				}
				else
				{
					if ((times[i]+times[i+1]) > 73000 && (times[i]+times[i+1]) < 77000)
					{
						value[index] = 0;
						index = index+1;
						i++;
					}
					else
					{
						value[index] = -1;
						index = index+1;
					}
				}
			}
		}
			
	

	} // for(int i=0; i<size;i++)
		
	
	// check for 1111 ---- ---- 0000
	for (int i=0; i<index;i++)
	{
		if (value[i] == 1)
		{
			start = 1;
			for (int j=0; j<4;j++)
			{
				if (value[i+j] != 1)
					start = 0;
			}
			if (start == 1)
			{
				for (int j=0; j<4;j++)
				{
					if (value[i+j+12] != 0)
					start = 0;
				}
			}
		}
		if (start == 1)
		{
			sequence = 0;

			for (int j = 8; j<12;j++)
			{
				sequence = sequence * 10 + value[i+j];
			}
			for (int j = 4; j < 8; j++)
			{
				address = address * 10 + value[i+j];
			}
			if (address == 11)
			{
				switch(sequence)
				{
					case 0:
						print_code = CTRL_ZERO;
						break;
					case 1:
						print_code = CTRL_ONE;
						break;
					case 10:
						print_code = CTRL_TWO;
						break;
					case 11:
						print_code = CTRL_THREE;
						break;
					case 100:
						print_code = CTRL_FOUR;
						break;
					case 101:
						print_code = CTRL_FIVE;
						break;
					case 110:
						print_code = CTRL_SIX;
						break;
					case 111:
						print_code = CTRL_SEVEN;
						break;
					case 1000:
						print_code = CTRL_EIGHT;
						break;
					case 1001:
						print_code = CTRL_NINE;
						break;
					case 1111:
						print_code = CTRL_MUTE;
						break;
					default:
						print_code = CTRL_ERROR;
						error_sequence = sequence;
						error_address = address;
						address = 0;
						sequence = 0;
				}
			} // if address == 11
			if (address == 101)
			{
					
				switch(sequence)
				{
					case 110:
						print_code = CTRL_LEFT;
						break;
					case 111:
						print_code = CTRL_RIGHT;
						break;
					case 1001:
						print_code = CTRL_UP;
						break;
					case 1000:
						print_code = CTRL_DOWN;
						break;
					default:
						print_code = CTRL_ERROR;
						error_sequence = sequence;
						error_address = address;
						address = 0;
						sequence = 0;
				}
			} // if address == 101
			if (address == 10)
			{
				switch(sequence)
				{
					case 1111:
						print_code = CTRL_VUP;
						break;
					case 1110:
						print_code = CTRL_VDOWN;
						break;
					case 1101:
						print_code = CTRL_CHUP;
						break;
					case 1100:
						print_code = CTRL_CHDOWN;
						break;
					default:
						print_code = CTRL_ERROR;
						error_sequence = sequence;
						error_address = address;
						address = 0;
						sequence = 0;
				}
			} // if address == 10
			if (address == 1111)
			{
				switch(sequence)
				{
					case 100:
						print_code = CTRL_ENTER;
						break;
					default:
						print_code = CTRL_ERROR;
						error_sequence = sequence;
						error_address = address;
						address = 0;
						sequence = 0;
				}
			}
			ROM_SysCtlDelay(SysCtlClockGet()/15);
			if (print_code != CTRL_ERROR && print_code != 0)
				valid = 1;
		}
	} // (int i=0; i<size;i++)
}


char char_selector(int code, int reps)
{
	char temp;
	switch(code)
	{
		case CTRL_ONE:
					temp = '1';
			break;
		case CTRL_TWO:
			switch(reps)
			{
				case 1:
					temp = 'a';
					break;
				case 2:
					temp = 'b';
					break;
				case 3:
					temp = 'c';
					break;
				default:
					temp = '2';
			}
			break;
		case CTRL_THREE:
			switch(reps)
			{
				case 1:
					temp = 'd';
					break;
				case 2:
					temp = 'e';
					break;
				case 3:
					temp = 'f';
					break;
				default:
					temp = '3';
			}
			break;
		case CTRL_FOUR:
			switch(reps)
			{
				case 1:
					temp = 'g';
					break;
				case 2:
					temp = 'h';
					break;
				case 3:
					temp = 'i';
					break;
				default:
					temp = '4';
			}
			break;
		case CTRL_FIVE:
			switch(reps)
			{
				case 1:
					temp = 'j';
					break;
				case 2:
					temp = 'k';
					break;
				case 3:
					temp = 'l';
					break;
				default:
					temp = '5';
			}
			break;
		case CTRL_SIX:
			switch(reps)
			{
				case 1:
					temp = 'm';
					break;
				case 2:
					temp = 'n';
					break;
				case 3:
					temp = 'o';
					break;
				default:
					temp = '6';
			}
			break;
		case CTRL_SEVEN:
			switch(reps)
			{
				case 1:
					temp = 'p';
					break;
				case 2:
					temp = 'q';
					break;
				case 3:
					temp = 'r';
					break;
				case 4:
					temp = 's';
					break;
				default:
					temp = '7';
			}
			break;
		case CTRL_EIGHT:
			switch(reps)
			{
				case 1:
					temp = 't';
					break;
				case 2:
					temp = 'u';
					break;
				case 3:
					temp = 'v';
					break;
				default:
					temp = '8';
			}
			break;
		case CTRL_NINE:
			switch(reps)
			{
				case 1:
					temp = 'w';
					break;
				case 2:
					temp = 'x';
					break;
				case 3:
					temp = 'y';
					break;
				case 4:
					temp = 'z';
					break;
				default:
					temp = '9';
			}
			break;
		case CTRL_ZERO:
			switch(reps)
			{
				case 1:
					temp = ' ';
					break;
				default:
					temp = '0';
			}
			break;
			
			
		case CTRL_VUP:
			break;
		case CTRL_VDOWN:
			break;
		case CTRL_CHUP:
			break;
		case CTRL_CHDOWN:
			break;
		case CTRL_UP:
			break;
		case CTRL_DOWN:
			break;
		case CTRL_LEFT:
			break;
		case CTRL_RIGHT:
			break;
		case CTRL_MUTE:
			temp = '\n';
			break;
		case CTRL_ENTER:
			temp = '\n';
			break;
		default:
			temp = '?';
	}
	return temp;
}
