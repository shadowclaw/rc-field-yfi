#include <avr/io.h>


// Auto Pilot modes
// ----------------
#define MANUAL 0
#define STABILIZE 1
#define FLY_BY_WIRE 2
#define AUTO 5
#define RTL 6
#define LOITER 7

// Flight Modes
// these Flight modes can be changed to either here or directly in events.pde
// options are MANUAL, STABILIZE, FLY_BY_WIRE, AUTO, RTL, LOITER
#define POSITION_1 MANUAL
#define POSITION_2 FLY_BY_WIRE
#define POSITION_3 STABILIZE



// set by the control switch read by the ATTiny
// --------------------------------------------
byte control_mode			= MANUAL;


// System Timers
// --------------
unsigned long fast_loopTimer	= 0;	// Time in miliseconds of main control loop
unsigned long slow_loopTimer	= 0;	// Time in miliseconds of main control loop
unsigned long deltaMiliSeconds 	= 0;	// Delta Time in miliseconds



// Basic Initialization
//---------------------
void setup() {
	Serial.begin(38400);
	init_ardupilot();
}

void loop(){
	// system Timers
	// ----------------------------------------
	deltaMiliSeconds 	= millis() - fast_loopTimer;

	if (deltaMiliSeconds < 20) {
		// force main loop to run at 50Hz
		return;
	}
	fast_loopTimer			= millis();

	// Read 3-position switch on radio (usually Ch5)
	// -------------------------------------------------
	read_control_switch();
}


void print_control_mode(void)
{
	switch (control_mode){
		case MANUAL:
			Serial.println("##0| MANUAL");
			break;
		case STABILIZE:
			Serial.println("##1| STABILIZE");
			break;
		case FLY_BY_WIRE:
			Serial.println("##2| FLY BY WIRE");
			break;
		case AUTO:
			Serial.println("##5| AUTOPILOT");
			break;
		case RTL:
			Serial.println("##6| RTL");
			break;
		case LOITER:
			Serial.println("##7| LOITER");
			break;
	}
}
