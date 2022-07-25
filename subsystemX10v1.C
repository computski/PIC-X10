/*SUBSYSTEM X10  ver1
2022-07-18

X10 home automation line driver.  Looks for 50Hz zero crossing events, and asserts a 1mS pulse of 100KHz on the line
within 200uS of this.

Communication with a ESP-01 is achieved through a CTS pin and a data pin.  CTS is read on its falling edge, which is
synced with ZC events.  the data is then asserted on the next ZC event.

test mode is provided, by tying GP3 to high, unit will alternately send ALL ON, ALL OFF to house code B.


Note: IOC can be used on any pin, including GP3 so we can use this for ZC detect.  The condition is cleared by any read/write
to GPIO.  therefore we need to assert the writes in the ZC service routine (i.e. CTS is clocked by 50Hz).
This is 'safe' compared to read/writes in the main loop which might cause ZC detect to fail if the RW coincides with the ZC event.

we need WDT active
we need a 1mS timer to cut off the X10 pulse
we need a general timer

GP3 used as ZC will indicate a ZC when it changes state. (goes hi at start of +ve cycle and remains high, goes low at end of this and stays low for neg cycle)
Therefore the CTS hi is driven from this, but it needs a 1mS timer (same as PWM cut off) to drop it to a logic low, where it reads DATA
i.e ESP01 should set-up data when it sees CTS go high and has 1mS in which to do this.  it should hold data stable during CTS=low

it might be possible to time 1mS using the PWM hardware...

timer0
timer1
timer2  used for PWM.  counts up to value held in PR2, at which point it resets to zero and sets TMR2IF.  the TMR2 post-scaler is part of this chain
and can be set to from 1:1 to 1:16

PWM output is on GP2 only (the INT pin so we cannot use this for ZC detect).
*/



#include <xc.h>
#include <stdint.h>


#define nTEST

#ifdef TEST
//test will run through in 4 mins
#define HOUR 60
#else
//production
#define HOUR 3600
#endif


#ifdef _12F683
#define _XTAL_FREQ    8000000
#pragma config BOREN = OFF, config WDTE = OFF, CPD = OFF, FOSC = INTOSCIO, MCLRE = OFF, CP = OFF, PWRTE = OFF
#endif


#ifdef _16F886
/*C:/Program Files/Microchip/xc8/v1.11/docs/chips/16f88.html*/
#pragma config FOSC =INTRC_NOCLKOUT, WDTE = OFF, CP = OFF, PWRTE = OFF, MCLRE=0; IESO=OFF, FCMEN=OFF, BOREN=OFF, LVP = OFF

#endif


eeprom char msg1[]="X1 SUBSYSTEM v1 2022-07-18      ";


/*
Shadow register for GPIO.  Required because of read-modify-write operations.
usage is sGPIO.port to refer to entire port or sGPIO.GP0 etc for individual bits
*/

volatile union {
uint8_t                 port;
struct {
                unsigned             TEST_MODE   :1;                          //[7] GP0 
                unsigned             DATA        :1;                          //[6] GP1
                unsigned             LINE        :1;                          //[5] GP2 PWM out
                unsigned             ZC_DETECT   :1;                          //[4] Input
                unsigned             SPARE1	    :1;                           //[3] spare
                unsigned             SPARE2      :1;                          //[2] spare
                };
} sGPIO;



/*
will run the whole system at 8MHz.
*/

volatile uint8_t	ticks;
volatile bit 		tickEvent;  //every 200mS 
unsigned long seconds;
uint8_t LEDshift;
int8_t hour;




const uint8_t LEDtable [5]={0b1,0b101,0b10101,0b1010101,0xFF};



/**********************************************************************************************************************************/
/*DECLARE FUNCTIONS*/
 
/*****************************/



main(void){
//boot sequence



OSCCONbits.IRCF=0b111; //8MHZ
//OSCCONbits.IRCF=0b110; //4MHZ
ANSEL=0x00;  //only on 12F675 and 683
TRISIO=0b001111;   //GP0,1,3 as inputs, rest output. GP2 input for now

//set up TMR1.  with a 4MHz clock
/* do not bother, will just use __delay_ms()
TMR1ON=1;
T1CONbits.T1CKPS=0b11;
TMR1CS=0;
PEIE=1;
TMR1IE=1;
*/

/*set up ADC */
//ANSELbits.ADCS=0b101;
//ADCON0bits.ADFM=1;  //right justify



//Boot with all outputs high
sGPIO.port=0xFF;

//enable WPU on 0
OPTION_REGbits.nGPPU=0;
WPU=0b000001;

//enable IOC on GP3




//set up PWM at 100kHz
//1. Disable the PWM pin (CCP1) output drivers by setting the associated TRIS bit.
//2. Set the PWM period by loading the PR2 register.
PR2 = 0x13;

//3. Configure the CCP module for the PWM mode by loading the CCP1CON register.
CCP1CON=0b00001100;   //PWM mode active hi

//4. Set the PWM duty cycle by loading the CCPR1L register and DC1B bits of the CCP1CON register.
CCPR1L=0x0A;

//5. Configure and start Timer2:
//• Clear the TMR2IF interrupt flag bit of the PIR1 register.
//• Set the Timer2 prescale value by loading the T2CKPS bits of the T2CON register.
//• Enable Timer2 by setting the TMR2ON bit of the T2CON register.
//6. Enable PWM output after a new PWM cycle has started:
//• Wait until Timer2 overflows (TMR2IF bit of the PIR1 register is set).
//• Enable the CCP1 pin output driver by clearing the associated TRIS bit



//prob should boot with duty cycle=0 (i.e. PWM off) or better, just gate the PWM on off using the TRIS bit 



//enable GP2 edge ints, and look for -ve edge as tilt switch is active low
//OPTION_REGbits.INTEDG=0;
 
//Timer 0 and ints
OPTION_REGbits.T0CS=0; //Timer0 driven by clock
OPTION_REGbits.PSA=1; //WDT prescaler

OPTION_REGbits.PS = 0x00;  //WDT 1:1
INTCONbits.T0IE=0;  //T0 ints OFF


//we don't use INT edges for interrupts, disable for now 
INTCONbits.INTE=0;

/*won't bother with interrupts, just poll TMR1IF*/
//ei();
 
seconds=0;


//debug block, drive LED on and disable HEATER
#ifdef this_block_disabled
sGPIO.LED=0;
sGPIO.HEATER=0;

while(1){

		GPIO=sGPIO.port;
	__delay_ms(1000);
//toggle LED
if (seconds<10){
sGPIO.LED ^= 1;
seconds++; 
}
else{
sGPIO.LED=1;
sGPIO.HEATER=1;
}

}
#endif
//end debug block




//main loop
while(1){


if (1){
	/*I was going to poll TMR1IF, but we might as well execute a delay to create the timing*/
	/*flag set every 200mS*/
	
	//TMR1L=0x57;
	//TMR1H=0x9E;

//2021-08-17 original code incremented seconds until we reached multiples of seconds-in-an-hour e.g. 3600, 7200, 10800 etc
//and would calculate the hour by using a divisor.  We now wish to count down, so either need to seed seconds and run it down
//or instead reset seconds after 3600 and decrement hour instead.
	
	ticks++;

	
	if (ticks>=5){
		seconds++;
		if (seconds>=HOUR){
			//reset seconds
			seconds=0;
			//hour is signed. time is up if it goes negative
			hour--;
		}


		ticks=0;
		//every 5th second, load the LEDshift register
	
		if ((seconds % 5)==0){
			NOP();
			//note that hour is 3 through 0 when active, -ve when time is up
			LEDshift=LEDtable[hour];
		}
	
	}//end ticks block
	TMR1IF=0;

	
		
		//clock through LEDshift
		sGPIO.LED = (LEDshift & 0x01)==1?1:0;
		LEDshift>>=1;
		sGPIO.HEATER=1;  //heater on
		GPIO=sGPIO.port;

		//if time is up, SLEEP.
		//the first hour upto 3600 sec

	if (hour<0){
			di();
			sGPIO.LED =1;
			sGPIO.HEATER=0;  //heater off
			GPIO=sGPIO.port;		
			SLEEP();
			NOP();
			//note, WDT needs to be disabled else it will wake us from sleep
		}



	}//200mS flag


	/*generate a delay, timing will be slightly off because the code loop above takes time to execute
	for greatest accuracy we should run off TMR1IF*/	
	__delay_ms(200);


};//end infinite loop
};//end main


/**********************************************************************************************************************************/
/*FUNCTIONS*/

/*routine to assert the output state ports, will ensure that GP 5,4,1,0 always have the same value before being
written to GPIO, it tracks the value on GP1.  */

void driveGPIO(void){
CLRWDT();
#ifdef _12F683
GPIO=sGPIO.port;
#else
PORTB=sGPIO.port;
#endif

}


/**********************************************************************************************************************************/
/*ISR*/

interrupt ISR(void) {
if (T0IF){
                ticks++;		
				T0IF=0;
         		}

if (TMR1IF){
                ticks++;
				TMR1IF=0;
}



if (INTF){
                //only ever branch here when waking from SLEEP
                INTCONbits.INTE=0; //disable further ints
                INTF=0; //clear this one
}

if (ADIF){ADIF=0;}  //only branch here when waking from ADC sleep

#ifdef _16F886
/*IR bit clocking or edge detecting*/
if (RBIF){RBIF=0;}
#endif


} 





