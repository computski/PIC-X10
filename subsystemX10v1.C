/*SUBSYSTEM X10  ver1
2022-07-18

X10 home automation line driver.  Looks for 50Hz zero crossing events, and asserts a 1mS pulse of 120KHz on the line
within 200uS of this.

Communication with a ESP-01 is achieved through a CTS pin and a data pin.  CTS is read on its falling edge, which is
synced with ZC events.  the data is then asserted on the next ZC event.

test mode is provided, by tying GP0 low, unit will alternately send ALL ON, ALL OFF to house code B.


Note: IOC can be used on any pin, including GP3 so we can use this for ZC detect.  The condition is cleared by any read/write
to GPIO.  therefore we need to assert the writes in the ZC service routine (i.e. CTS is clocked by 50Hz).
This is 'safe' compared to read/writes in the main loop which might cause ZC detect to fail if the RW coincides with the ZC event.

we need WDT active
we need a 1mS timer to cut off the X10 pulse, use TMR0
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

C does not allow functions inside structs https://stackoverflow.com/questions/17052443/c-function-inside-struct


Note; we gate PWM on/off by setting the period registers to zero to hold it active low.  This is preferable to using TRIS which might couple noise into the NPN drive transistor
or WPU it active which we don't want.

looks like there is a 20-30uS delay between seeing a ZC transition and asserting a 120kHz carrier.  using TRISIO might be faster?
*/



/*
50Hz sync and 120kHz generation.
We have 10mS ZC events.  during this time, TMR2 postscaler is clocking upwards
will use the TMR2IF to increment a counter
*/

#include <xc.h>
#include <stdint.h>


#define TESTPWM
//debug, if TESTPWM we leave PWM running all the time

#define nNOSYNC
//debug, if NOSYNC then PWM is left at factory frequency


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
/*C:\Program Files (x86)\Microchip\xc8\v1.36\docs\chips\12f683.html*/
#pragma config FOSC =INTRC_NOCLKOUT, WDTE = OFF, CP = OFF, PWRTE = OFF, MCLRE=0; IESO=OFF, FCMEN=OFF, BOREN=OFF, LVP = OFF

#endif








eeprom char msg1[]="X10 SUBSYSTEM v1 2022-08-22     ";


volatile uint8_t outBuffer=0b11001010;
//volatile uint8_t outBuffer=0b11111111;



//LETS GET FREQ LOCK WORKING FIRST
//next test is lock on real 50Hz
//and also do a 60Hz test


/*
Shadow register for GPIO.  Required because of read-modify-write operations.
usage is sGPIO.port to refer to entire port or sGPIO.GP0 etc for individual bits
*/

volatile union {
uint8_t                 port;
struct {
                unsigned             CLK         :1;                          //[7] GP0 output
                unsigned             DATA        :1;                          //[6] GP1 input
                unsigned             LINE        :1;                          //[5] GP2 PWM out
                unsigned             ZC_DETECT   :1;                          //[4] Input
                unsigned             SPARE1	    :1;                           //[3] GP4spare
                unsigned             SPARE2     :1;                          //[2]  GP5 test (input)
                };
} sGPIO;



volatile struct {
	uint8_t  CYcount;
	uint16_t T2count;
	float AvgError;
	unsigned  sync50 :1;
	unsigned  sync60 :1;
	unsigned  requestIntEnable :1;
	unsigned  ZCblank :1;
} sync120KHZ;



/*
will run the whole system at 8MHz.
*/

volatile uint16_t	ticks;
volatile bit buffer;

/**********************************************************************************************************************************/
/*DECLARE FUNCTIONS*/
 
/*****************************/



main(void){
//boot sequence




OSCCONbits.IRCF=0b111; //8MHZ
//OSCCONbits.IRCF=0b110; //4MHZ
ANSEL=0x00;  //only on 12F675 and 683.  All digital IO
TRISIO=0b001010;   //GP1,3, as inputs, rest output. 
//enable WPU on 1,5
OPTION_REGbits.nGPPU=0;
WPU=0b100010;



//set up TMR0
OPTION_REGbits.T0CS=0;  //internal clock running on instruction cycle
OPTION_REGbits.PSA=0;   //prescaler for T0 use
OPTION_REGbits.PS=0b010; //div 8
INTCONbits.T0IE=0;  //T0 ints OFF
//we need to count through 250 for 1mS
//int is generated on rollover to zero, counting up

//set up TMR1 for general 10mS heartbeat
//we need to count through 40,000
//int is generated on rollover to zero, counting up
TMR1ON=1;
T1CONbits.T1CKPS=0b11;  //div 8 prescale
TMR1CS=0;  //instruction clock
PEIE=1;
TMR1IE=0;  //debug, no TMR1 ints for now
 



//Boot with all outputs low.
//Later we will need to boot high and add a delay, this gives the ESP-01 time to boot without our pulling
//any of its pins low which would cause its boot to fail
sGPIO.port=0x00;



//enable IOC on GP3
IOC=0b001000;
INTCONbits.GPIE=1;  //enable IOC int
INTCONbits.GIE=1;  //enable umasked ints



//set up PWM at 100kHz
//1. Disable the PWM pin (CCP1) output drivers by setting the associated TRIS bit.
//2. Set the PWM period by loading the PR2 register.
PR2 = 0x10;

//3. Configure the CCP module for the PWM mode by loading the CCP1CON register.
CCP1CON=0b00001100;   //PWM mode active hi, DC1B bits both low


//4. Set the PWM duty cycle by loading the CCPR1L register and DC1B bits of the CCP1CON register.
CCPR1L=0x08;
CCP1CONbits.DC1B=0b10;  //now set DC1B bits to 0x2

//5. Configure and start Timer2:
//• Clear the TMR2IF interrupt flag bit of the PIR1 register.
//• Set the Timer2 prescale value by loading the T2CKPS bits of the T2CON register.
//• Enable Timer2 by setting the TMR2ON bit of the T2CON register.
T2CONbits.T2CKPS = 0b00;   //set as 1.  other options 4,16
TMR2ON=1;
T2CONbits.TOUTPS=0b1111;  //postscaler of 16

//T2CONbits.TOUTPS=0b0111;  //postscale of 8
T2IE=1;

//NOTE: frequency will be 117,647Hz, we cannot hit 120kHz exactly without de-tuning the oscillator
//postscaler will trigger an int at 14.7k or 68uS period.  when tuned it will be 15Khz or 66.6 uS period.



//6. Enable PWM output after a new PWM cycle has started:
//• Wait until Timer2 overflows (TMR2IF bit of the PIR1 register is set).
//• Enable the CCP1 pin output driver by clearing the associated TRIS bit
TRISIObits.TRISIO2=0;


OSCTUNE=0;  //factory calib
//OSCTUNE=0b01111; //max freq

//a 2004 datasheet mentions 12% osc tune, i.e. 6% +/- but a 2007 datasheet removes this statement
//the datasheet says it takes 1mS for the freq to stabilise, so if we are to tune it we have to wait and see where we end up before
//we try and tune again.
//e.g. 10mS gating for changes.  read TMR2 postscaler counts every 10mS driven by the ZC, average these, then make a decision to tune, and then
//wait to see outcomes

//how to use TMR2IF.  occurs every 68uS.  instruction clock is 0.5uS we have sufficient time to incr a counter
//then read this off every ZC event into a double buffer that the main loop then averages and decides if a 
//OSCTUNE is required




//initialise the 120KHz sync mechanism
sync120KHZ.CYcount=100;  //First pass is 100 cycle-counts
sync120KHZ.T2count=0x00;
sync120KHZ.AvgError=1500;  //assume we start on-frequency
sync120KHZ.sync50=0;
sync120KHZ.sync60=0;
sync120KHZ.requestIntEnable=1;

//concept: The first ZCcount is 100 and is used to sample the mains frequency relative to factory calibrated clock.  From this
//wee determine whehter we are syncing to 50 or 60Hz.  Then we go into lock mode.




//main loop
while(1){


//120KHz generation.
//first run a 2S sample period against factory clock calibration to determine if we are to lock onto 50Hz or 60Hz
//then run an 80/20 exponential smoothing calculation every 10 cycles and from this adjust OSCTUNE up or down to lock
//the 120KHz PWM output to the mains cycle.
//Note that we sync to a full mains cycle, because the ZC periods are not equal due to one being a rising voltage, one falling
//T2 with a postscaler of 16 will overrun 150 times in a 20mS cycle.  i.e. 1500 on a 10 cycle measurement is locked @50Hz
//and 1250 is locked @60Hz.  There is a deadband of 0.5% at lock to avoid creating jitter.
//With a real 50Hz signal, saw spikes in the 1mS clock like.  To improve noise immunity IOC is suspended for the 1mS clock
//period.  Retriggers on the IOC for GP3 will also introduce jitter.
//At the factory calibration the PWM will run at 100kHz so we are always wanting to tune upwards to a higher freq

//IMPORTANT.  If we disable PWM output by setting the period rather than gating TRISIO, then we might stop T2 entirely or 
//possibly we incorrectly count cycles because we change the duty.  not sure.  TRISIO is probably safer.





if (sync120KHZ.CYcount==0){
//average in the T2 postscaler count. Suspend T2 count whilst we do this
TMR2IE=0;
//exponentially smooth in a reading

sync120KHZ.AvgError=0.8*sync120KHZ.AvgError + 0.2* sync120KHZ.T2count;

#ifndef NOSYNC
if (sync120KHZ.sync50==1){
	//50Hz sync
	//make corrections for being over/under and we also want a deadband of say 1%
	if (sync120KHZ.AvgError>1508){
		//too fast, decrease toward 0b00000
		OSCTUNE+=OSCTUNE==0?0:-1;
	}
	else if(sync120KHZ.AvgError<1492){
		//too slow, increase toward 0b01111 max
		OSCTUNE+=OSCTUNE==0b01111?0:1;
	}
}else if(sync120KHZ.sync60==1){
//60Hz sync
		if (sync120KHZ.AvgError>1256){
		//too fast, decrease toward 0b00000
		OSCTUNE+=OSCTUNE==0?0:-1;
	}
	else if(sync120KHZ.AvgError<1244){
		//too slow, increase toward 0b01111 max
		OSCTUNE+=OSCTUNE==0b01111?0:1;
	}

}else{
//counts above 13480 are 50Hz, below are 60Hz.  At factory we expect to see 14705 for 50Hz and 12254 for 60Hz
//boot - determine whether ZC is 50 or 60Hz
if (sync120KHZ.T2count>13480){
	sync120KHZ.sync50=1;
sGPIO.SPARE1=1;
	}else{
sync120KHZ.sync50=1;
sGPIO.SPARE2=1;
	}
}
#endif



//reset measurement period to 10 cycles
sync120KHZ.CYcount=10;
sync120KHZ.T2count=0;

//Accuracy.  Do not re-enable TMR2IE here because we don't know when the next ZC int may arrive, this will
//cause jitter in our measurement.  Set a flag and let the IOC Int re-enable it sycrhonously at next ZC event.
//Do this AFTER setting ZCcount
sync120KHZ.requestIntEnable=1;

}



//seems to be 16 sec?
if (ticks>=100){
ticks=0;
//toggle spare1

}



/*
if (1){
	
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


	
	__delay_ms(200);
*/

};//end infinite loop
};//end main


/**********************************************************************************************************************************/
/*FUNCTIONS*/




/**********************************************************************************************************************************/
/*ISR*/
//we monitor GPIF, which will be driven by ZC detection on GP3
//read/write to port will clear the IOC states, so GPIO is only ever writen/read in the ISR
//a ZC will cause a 1mS pulse on CLK and also generate a 1mS X10 pulse to line (or not if a space is required)
//PWM and the data CLK signal will only operate if ZC is detected.  

//GPIF will be every 10mS, TMR2IF will be every 68uS when PWM is 100kHz
//T0IF will be 1mS after it is triggered
//We can only read/write from/to GPIO during the GPIF int else we will stop IOC from working.


interrupt ISR(void) {

if (GPIF) {
//GP3 changed state, which means a ZC transition.  start a 1mS timer and clock out the data
T0IF=0;
TMR0=0x05;
T0IE=1;  //enable timer0 int overflow

//enable or disable PWM output as required. 1 in the buffer will enable the PWM to line
//disable for now TRISIObits.TRISIO2= !buffer;
//take CLK high
sGPIO.CLK=1;
//assert port
GPIO=sGPIO.port;


if (sync120KHZ.requestIntEnable==1){
	//re-enable T2 ints sycnrhonously with the IOC ZC int
	TMR2IE=1;
	sync120KHZ.requestIntEnable=0;
}else{
	//sampling; decrement ZC count toward zero
	//but heed ZCblanking which skips alternate ZC events
	sync120KHZ.ZCblank =!sync120KHZ.ZCblank;
	if (sync120KHZ.ZCblank) {
	sync120KHZ.CYcount-=sync120KHZ.CYcount==0?0:1;
	}
}

//process outbound data
//rotate buffer through itself DEBUG

#ifndef TESTPWM

if ((outBuffer & 0b10000000)==0){
outBuffer =outBuffer<<1;
//leave pwm duty at zero
}else{
outBuffer =outBuffer<<1;
outBuffer++;
//set 50% duty cycle to activate the PWM, this is gated by the outgoing databits
CCPR1L=0x08;
CCP1CONbits.DC1B=0b10;
}

#endif

//disable IOC until end of the T0IF interrupt, this should give better noise immunity.  Clear the GPIF
IOC=0b000000;
GPIF=0;
}


if (T0IF){
	//end of 1mS pulse
	//disable PWM output
	//TRISIObits.TRISIO2=1;   //debug leave active for testing
	//disable TMR0 overflow int
	T0IE=0;
	T0IF=0;
	//read data into buffer
	buffer = (GPIO & 0x01)==0?0:1;
	//take CLK low
	sGPIO.CLK=0;
	//assert port
	GPIO=sGPIO.port;
#ifndef TESTPWM
	//zero percent duty cycle to PWM to inhibit it
	CCPR1L=0x0;
	CCP1CONbits.DC1B=0b00;
#endif
	//re-enable IOC, look for next ZC event
	IOC=0b001000;	
    }


if (TMR1IF){
	//10mS interrupt F6
	TMR1H=0xF6;
	TMR1L=0x3C;
	ticks++;
	TMR1IF=0;
}

//TMR2 incrments off the T2 postscalar and will set a flag when it rolls over
//will be 0.136 mS or less between these ints
if (TMR2IF){
	sync120KHZ.T2count++;
	TMR2IF=0;
}

}//end of ISR 





