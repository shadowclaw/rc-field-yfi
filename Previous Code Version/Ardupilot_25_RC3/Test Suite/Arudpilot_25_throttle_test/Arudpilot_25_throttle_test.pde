#include <avr/io.h>
#include <avr/eeprom.h>
#include <math.h>
#include "easystar_25.h"


// Radio values
// ------------
int ch1_trim = 0;			// set by init sequence
int ch2_trim = 0;			// set by init sequence
int ch3_trim = CH3_TRIM;	// set by init sequence unless THROTTLE_IN == 0
int ch3_timer_trim = 0;		// set by init sequence
int ch3_fs;					

int ch1_in;							// store rudder position from radio
int ch2_in;							// store elevator position from radio
int ch3_in;							// store thottle position from radio

int ch1_out = 1500;				// actual µs values to servos
int ch2_out = 1500;				// actual µs values to servos
int ch3_out = CH3_TRIM;			// actual µs values to servos

// servo limits - set during initialization
// ----------------------------------------
int ch1_min		= CH1_MIN;		// lowest 	~ 1000
int ch1_max		= CH1_MAX; 		// highest 	~ 2000
int ch2_min		= CH2_MIN;		//
int ch2_max		= CH2_MAX;		//

// attitude control output
// -----------------------
float 	servo_roll			= 0;	 		// degrees to servos
float 	servo_pitch			= 0;	 		// degrees to servos
int 	servo_throttle		= 0;			// 0-125 value

// System Timers
// --------------
unsigned long loopTimer			= 0;	// Time in miliseconds of main control loop
unsigned long deltaMiliSeconds 	= 0;	// Delta Time in miliseconds


// Basic Initialization
//---------------------
void setup() {
 	Serial.begin(57600);
	pinMode(2,INPUT);	// PD2 - INT0 		- Rudder in							- INPUT Rudder
	pinMode(3,INPUT);	// PD3 - INT1 		- Elevator in 						- INPUT Elevator
	pinMode(13,INPUT); //  PB5 - SCK		- Yellow LED pin   					- INPUT Throttle
	
	init_PWM();
	init_radio();
}


void loop(){
	// system Timers
	// ----------------------------------------
	deltaMiliSeconds 	= millis() - loopTimer;
	if (deltaMiliSeconds < 20) {
		// force main loop to run at 50Hz
		return;
	}
	loopTimer			= millis();
		
	// Filters radio input - adjust filters in the radio.pde file
	// ----------------------------------------------------------
	read_radio();

	// send the throttle value to the PWM out
	// --------------------------------------
	update_throttle();

	print_radio();
}

void print_radio(void)
{
	Serial.print("MS: ");
	Serial.print(deltaMiliSeconds, DEC);
	Serial.print("\t ch1:");
	Serial.print(ch1_in,DEC);
	Serial.print("\t ch2:");
	Serial.print(ch2_in,DEC);
	Serial.print("\t ch3:");
	Serial.print(ch3_in,DEC);
	Serial.print("\t\t ch3_OUT:");
	Serial.println(ch3_out,DEC);
}

void set_failsafe(boolean mode)
{

}
void read_XY_analogs(void)
{

}
