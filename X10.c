#include <xc.h>
#include <stdint.h>

/*
X10 test 2014-07-22

DEVICE      12F675
OSCILLATOR  4MHz internal
COMPILER	XC8


*/


#define _XTAL_FREQ    4000000


/*HARDWARE NOTES on the Vacuum cleaner board
Pin assignments are
[1] Gnd
[2] GP5 TRIAC
[3] GP4 TRIAC
[4] GP3 input only
[5] GP2 ZC detect, connected to AC2
[6] GP1 spare output for LED
[7] GP0 drives relay, active low
[8] Vcc
*/

//Note that GP2 rising edge is the one closest to the ZC event. The PIC Vcc (+ve) rail is tied to AC1, and GP2 will go high 0.6v prior to the ZC event.

//2014-07-22 works!  The LED toggles once per second, confirming there is a single +ve edge on GP2 per cycle

/*
next test;  fire the triac, which i presume requires me to pull GP4/5 low so that the gate is -5v below AC1.  It needs to be held on
until latch current takes over, or a few ms into the cycle.

Then i want it to step brighten/dim based on
(a) fire alternate half cycles gives 50%
(b) skip cycles and half cycles, so we can arrange for 25% jumps based on how many half cycles i fire, e.g. 3 in 4
these are all ZC dtriggered and in theory will minimise heat dissipation in triac because its not phase triggered
the above is for fun, because I will not need the triac in future.


Note the relay transistor has a pull up resistor to AC1.  Its apparently PNP, and the relay is on a -25v supply.  So its going to be active low on the base,
which you pull from AC1 to -5v below AC1
*/




/*
X10 implementation will be;
GP4/5 assigned to 7.68 MHz crystal
GP3 is radio RS232 input
GP2 is ZC detect
GP1 will drive X10 output
GP0 can still drive a relay
*/







/*
#define IO_FAN		sPORTA.RA4
#define IO_EXHAUST	8  //RB3  this is mask 8
#define IO_INLET	1  //RB0 this is mask 1
#define IO_DH_STS	PORTAbits.RA5  //dh status
#define IO_DH_CTRL	sPORTB.RB5  //dh control
#define TRIS_DH_CTRL	TRISBbits.TRISB5
#define leftButtonGPIO	PORTBbits.RB6
#define rightButtonGPIO	PORTBbits.RB7
#define IO_FAN_GPIO	PORTAbits.RA4
*/


//DECLARE FUNCTIONS




/*
set config bits do not use brackets.  The options are described in the following file
C:/Program Files/Microchip/xc8/v1.11/docs/chips/12f675.html
Set to use internal 4MHz oscillator, all GP pins as IO
*/

#pragma config BOREN = OFF, CPD = OFF, FOSC = INTRCIO, MCLRE = OFF, WDTE = OFF, CP = OFF, PWRTE = OFF



/*
Create shadow registers for PORTA and PORTB.  These are needed because of read-modify-write operations.
usage is sPORTA.port to refer to entire port or sPORTA.RA0 etc for individual bits
*/
volatile union {
uint8_t		port;
struct {
	unsigned	GP0	:1;   
	unsigned	GP1	:1;   
	unsigned	GP2	:1;
	unsigned	GP3	:1;
	unsigned	GP4	:1;
	unsigned	GP5	:1;
	unsigned	GP6	:1;
	unsigned	GP7	:1;
	};
} sGPIO;




volatile uint8_t    ticks;	//increments every 50th of a second
volatile uint8_t	Hz;

//conditional compilation switches, see XC8 manual section 2.5.15
//comment out one or other or neither.

enum state232
{
TX_START, // start transmit
TX_BIT,  //bit pending
RX_IDLE,  //idle
RX_START,  //start bit seen
RX_BIT, //bit pending
RX_BUFFER //data has been received user must deal with it
};

const char msgText[]="ringmybell\n\r";


//MAIN PROGRAM LOOP

void main(void)
{
//boot sequence
//Boot with PORTA all inputs.
//GP3 is always an input
//make GP1 an output;
//GP2 detects ZC events

//GP2 can be a clock input for TMR0


TRISIO=0b11111101;

sGPIO.port = 0xFF; //make all PORTA outputs hi 
GPIO = sGPIO.port;


//NOTE: with this PIC, inputs default to analogue mode, you must set them as digital inputs
CMCON = 0x07;  //disable comparator and put in lowest power mode
ANSEL=0; //set all inputs to digital mode

//you also need to set the IOC register to enable intterupt on change to be recognised

//need to set up the timers
// set up timer 1 with 8 bit prescalar and enable it
//bits <4,5> control prescalar 0=1, 3=8
	
//so we are driving it with 1Mhz or 1 uS periods.  for 1200 baud we count
T1CON=0x01;

//enable peripheral ints and enable Timer1 interrupt
INTCONbits.PEIE=0;
PIE1bits.TMR1IE=0;

//enable GP2 +ve edge triggered int
OPTION_REGbits.INTEDG=1;  //+ve edge
INTCONbits.INTE=1;  //enable edge triggering
ticks=0;


ei();




//program loop
while(1){



GPIO = sGPIO.port;

NOP();
}
} //end MAIN

//MY PIC CALIB is ox3410
/*INTERRUPT ROUTINE
//Timer 1 is dedicated to RS232 transmit
//we use Int on change IOC to detect a start bit on GP1

*/
interrupt ISR(void)
{



if (PIR1bits.TMR1IF)
{
//Timer1 int handler, reset counter with 208 ticks up to 0, i.e. FF30h
// NOTE: look at the compiler .lst file to check the number of operations we execute before we get to the line where the timer
// is reloaded.  this will allow us to fine tune the delay, seems to be about 13 ops

//going for FCCC which is 1200 baud as i doubt radios can handle more reliably.  This is 833 instructions less 13 adjustment

TMR1L=0xCC;
TMR1H=0xFC;
PIR1bits.TMR1IF = 0; //clear Timer 1 int flag

}

//ODD.  led comes on immediately, goes off after about 1 sec and then stays off.


if (INTCONbits.INTF)
{
ticks++;
INTCONbits.INTF=0;
if (ticks>=50){
	//toggle GP1 and reset ticks
	ticks=0;
	sGPIO.GP1 = ~sGPIO.GP1;
	}
}


}  //end ISR

