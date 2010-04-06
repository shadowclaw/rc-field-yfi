#include <avr/io.h>
#include <avr/eeprom.h>
#include <math.h>
#include "defines.h"

//To use the header file in your library, use brackets:
//#include <easystar.h>

//To use the header file in your local folder, use quotes:
#include "easystar_25.h"

 /*
ArduPilot By Jordi Munoz and Jason Short
ArduPilot Version 2.5.0
Developed by
	-Chris Anderson
	-Jordi Munoz
	-Jason Short
	-Doug Wiebel
	-HappyKillMore
	-Jose Julio	
	-Bill Premerlani
	-James Cohen.
	-JB from rotorFX.
	-Automatik.
	-Fefenin
	-Peter Meister
	-Remzibi
	-Your Name Here.


*/


// set by the control switch read by the ATTiny
// --------------------------------------------
byte control_mode			= MANUAL;
boolean invalid_location 	= true;		// used to indicate we can't navigate witout good GPS data - the equations will choke

// Radio values
// ------------
int ch1_trim = 0;			// set by init sequence
int ch2_trim = 0;			// set by init sequence
int ch3_trim = CH3_TRIM;	// set by init sequence unless THROTTLE_IN == 0
int ch3_timer_trim = (float)(CH3_TRIM - 1000) * .125;// set by init sequence
int ch3_fs;					// set your failsafe throttle >50\u00b5s below ch3 trim

int ch1_in;						// store rudder position from radio
int ch2_in;						// store elevator position from radio
int ch3_in;						// store thottle position from radio

int ch1_out = 1500;				// actual µs values to servos
int ch2_out = 1500;				// actual µs values to servos
int ch3_out = CH3_TRIM;			// actual µs values to servos

// servo limits - set during initialization
// ----------------------------------------
int ch1_min		= CH1_MIN;		// lowest 	~ 1000
int ch1_max		= CH1_MAX; 		// highest 	~ 2000
int ch2_min		= CH2_MIN;		//
int ch2_max		= CH2_MAX;		//

// If the radio goes out the Throttle should go below trim on radio
// ----------------------------------------------------------------
boolean failsafe			= false;		// did our throttle dip below the failsafe value?
byte 	config_tool_options	= 0;			// 

// attitude control output
// -----------------------
float 	servo_roll			= 0;	 		// degrees to servos
float 	servo_pitch			= 0;	 		// degrees to servos
int 	servo_throttle		= 0;			// 0-125 value

// GPS variables
// -------------
byte 	GPS_fix				= BAD_GPS;		// This variable store the status of the GPS
boolean gps_failure			= false;		// not currently used
float 	ground_speed 		= 0;			// meters/second
float 	climb_rate 			= 0;			// meters/second
byte 	GPS_update			= GPS_NONE;		// do we have GPS data to work with?
const float t7				= 10000000.0;	// used to scale GPS values for EEPROM storage

// navigation 
// ----------
long 	ground_course 		= 0;			// degrees * 100 dir of plane 
long 	target_bearing		= 0;			// degrees * 100 location of the plane to the target
long 	bearing_error		= 0; 			// degrees * 100
long 	crosstrack_bearing	= 0;			// degrees * 100 location of the plane to the target
int 	altitude_error		= 0;			// meters * 100 we are off in altitude
int 	max_altitude		= 0;			// meters - read by config tool!
byte 	max_speed			= 0;			// m/s
byte 	wp_radius			= 15;			// meters - set by config tool!
boolean wp_mode				= ABS_WP;		// ABS_WP or REL_WP
float 	airspeed_scaler		= 1;

// these are the values returned from navigation control functions
// ----------------------------------------------------
long 	nav_roll			= 0;					// target roll angle in degrees * 100
long 	nav_pitch			= 0;					// target pitch angle in degrees * 100
int 	nav_airspeed		= THROTTLE_CRUISE;		// target airspeed sensor value - THROTTLE_CRUISE = airspeed at cruising

// navigation control gains
// ------------------------
float head_P 	 			= HEAD_P; 				// Heading error proportional 
float head_I 	 			= HEAD_I;  				// Heading error integrator
float altitude_pitch_P 		= ALTITUDE_PITCH_P; 	// Altitude error proportional - controls the pitch with elevators 
float altitude_throttle_P 	= ALTITUDE_THROTTLE_P; 	// Altitude error proportional - controls the throttle
float integrators[]			= {0,0,0,0};			// PID Integrators


/***************************************************************************
 NMEA variables
 **************************************************************************/
#if GPS_PROTOCOL == 0 // This condition is used by the compiler only....
	// GPS Pointers
	char *token;
	char *search = ",";
	char *brkb, *pEnd;
	char gps_buffer[200]; //The traditional buffer.
#endif

/***************************************************************************
 SIRF variables
 **************************************************************************/
#define BUF_LEN 100
#if GPS_PROTOCOL == 1
	// The input buffer
	char gps_buffer[BUF_LEN]={0x24,0x50,0x53,0x52,0x46,0x31,0x30,0x30,0x2C,0x30,0x2C,0x35,0x37,0x36,0x30,0x30,0x2C,0x38,0x2C,0x31,0x2C,0x30,0x2A,0x33,0x37,0x0D,0x0A};	
	// Used to configure Sirf GPS
	const byte gps_ender[]={0xB0,0xB3};	
#endif
/***************************************************************************
 uBlox Variables
 **************************************************************************/
#if GPS_PROTOCOL == 2
	byte UBX_buffer[40];

	//uBlox Checksum
	byte ck_a 					= 0;
	byte ck_b 					= 0;
	long iTOW 					= 0; //GPS Millisecond Time of Week
	//long alt 					= 0; //Height above Ellipsoid 
	//long speed_3d 			= 0; //Speed (3 - D)	(not used)
	byte UBX_class				= 0;
	byte UBX_id					= 0;
	byte UBX_payload_length_hi	= 0;
	byte UBX_payload_length_lo	= 0;
	byte UBX_payload_counter	= 0;
	byte UBX_ck_a 				= 0;
	byte UBX_ck_b				= 0;
#endif


/***************************************************************************
 IMU Variables
 **************************************************************************/
#if GPS_PROTOCOL == 3
	byte IMU_buffer[20];
	byte payload_length	= 0;
	byte payload_counter	= 0;
	//IMU Checksum
	byte ck_a 		= 0;
	byte ck_b 		= 0;
	byte IMU_ck_a 		= 0;
	byte IMU_ck_b		= 0;
#endif



// used to consruct the GPS data from Bytes to ints and longs
// ----------------------------------------------------------
union long_union {
	int32_t dword;
	uint8_t	byte[4];
} longUnion;

union int_union {
	int16_t word;
	uint8_t	byte[2];
} intUnion;



// System Timers
// --------------
unsigned long fast_loopTimer	= 0;	// Time in miliseconds of main control loop
unsigned long slow_loopTimer	= 0;	// Time in miliseconds of main control loop
unsigned long deltaMiliSeconds 	= 0;	// Delta Time in miliseconds
unsigned long elapsedTime 		= 0;	// in miliseconds


// Waypoints
// ---------
long 	wp_distance			= 0;	// meters - distance between plane and next waypoint
long 	wp_totalDistance	= 0;	// meters - distance between old and next waypoint
float 	wp_area				= 0;	// Pre-calculation to speed mapping function
byte 	wp_total			= 0;	// # of waypoints
byte 	wp_index			= 0;	// Current WP index, -1 is RTL
float 	scaleLongDown		= 0;	// cosine of the Latitude used to scale longitude		
float 	scaleLongUp			= 0;	// used to reverse longtitude scaling

// 3D Location vectors
// -------------------
struct Location {
	long lat;
	long lng;
	long alt;
};

struct Location home 				= {0,0,0};		// home location
struct Location prev_WP 			= {0,0,0};		// last waypoint
struct Location current_loc 		= {0,0,0};		// current location
struct Location est_loc 			= {0,0,0};		// for estimation
struct Location next_WP 			= {0,0,0};		// next waypoint

// Sensors 
// --------
long  analog0				= 511;		// Thermopiles - Pitch
long  analog1				= 511;		// Thermopiles - Roll
long  analog2				= 511;		// Thermopiles - Z
float analog3				= 511;		// Airspeed Sensor - is a float to better handle filtering
float analog5				= 511;		// Battery Voltage
float battery_voltage 		= 0;
int ir_max					= 40;		// used to scale Thermopile output to 511
long roll_sensor			= 511;		// how much we're turning
long pitch_sensor			= 511;		// our angle of attack
int airspeed_offset			= 0;		// read the analog airspeed sensors to get this
long airspeed_current		= 0;		// airspeed 

// Debugging
// ---------
long est_turn_rate			= 0;
long actual_turn_rate		= 0;


// Basic Initialization
//---------------------
void setup() {
	#if GPS_PROTOCOL == 0
		Serial.begin(FIFTY_SEVEN_K_BAUD);
	#endif
	#if GPS_PROTOCOL == 1
		Serial.begin(FIFTY_SEVEN_K_BAUD);
	#endif
	#if GPS_PROTOCOL == 2
		Serial.begin(THIRTY_EIGHT_K_BAUD);
	#endif
	#if GPS_PROTOCOL == 3
		Serial.begin(THIRTY_EIGHT_K_BAUD);
	#endif
	
	init_ardupilot();
}


#if DEBUG == 0
void loop()
{
	// system Timers
	// ----------------------------------------
	deltaMiliSeconds 	= millis() - fast_loopTimer;
	if (deltaMiliSeconds < 20) {
		// force main loop to run at 50Hz
		return;
	}
	fast_loopTimer			= millis();

	// Uncomment to prove the loop timer
	// Serial.println(deltaMiliSeconds,DEC);
	
	// Read 3-position switch on radio (usually Ch5)
	// -------------------------------------------------
	read_control_switch();

	// Filters radio input - adjust filters in the radio.pde file
	// ----------------------------------------------------------
	read_radio();
	
	// check for throtle failsafe condition
	// ------------------------------------
	throttle_failsafe();

	// Read IR SENSORS
	// ---------------
	read_XY_analogs();


	// Output telemtry
	// ---------------
	if(millis() - slow_loopTimer > 100){
		slow_loopTimer = millis();

		// Read in the GPS position
		// ------------------------
		decode_gps();

		if (GPS_update & GPS_POSITION){
			print_position();
			// GPS event notification
			// ---------------
			gps_event();
		}
		
		if(invalid_location && current_loc.lat > 0){
			reset_location();
		}
		
		print_attitude();
		//print_radio();
		
		// reads Battery and Z sensor
		// --------------------------
		read_analogs();
		set_max_altitude_speed();
	}

	if (control_mode == STABILIZE){
		// Attitude Control Loop
		// -----------------------
		stabilize();

	} else if (control_mode == FLY_BY_WIRE){
		// Fake navigation Control Loop
		// ----------------------------
		nav_roll = ((ch1_in - ch1_trim) * 4500 * REVERSE_ROLL) /500;
		nav_roll = constrain(nav_roll, HEAD_MIN, HEAD_MAX); //3500

		// nav_pitch is the amount to adjust pitch due to altitude
		nav_pitch = ((ch2_in - ch2_trim) * 4500 * REVERSE_PITCH) /500;
		nav_pitch = constrain(nav_pitch, ALTITUDE_PITCH_MIN, ALTITUDE_PITCH_MAX);

		// nav_airspeed is the amount to adjust throttle due to altitude
		nav_airspeed = ((ch2_in - ch2_trim) * 1500) /1000;
		nav_airspeed = constrain(nav_airspeed, ALTITUDE_AIRSPEED_MIN, ALTITUDE_AIRSPEED_MAX);// -15 : 15
		
		// Attitude Control Loop
		// -----------------------
		stabilize_AP();
		
	}else if (control_mode >= AUTO){

		// Navigation Control Loop
		// -----------------------
		navigate();

		// Attitude Control Loop
		// -----------------------
		stabilize_AP_mix();
		
	}else if (control_mode == MANUAL){

		// set the outs for telemtry
		// -------------------------
		ch1_out = ch1_in;
		ch2_out = ch2_in;
		ch3_out = ch3_in;
	}

	// Clear GPS update manually
	// ------------------------
	GPS_update = GPS_NONE;
	
	// send the throttle value to the PWM out
	// --------------------------------------
	update_throttle();

	// Send an event notice of the main loop
	// -------------------------------------
	mainLoop_event();
}
#endif

// this is the debugging loop
// --------------------------
#if DEBUG == 1
void loop()
{
	// system Timers
	// ----------------------------------------
	deltaMiliSeconds 	= millis() - fast_loopTimer;
	if (deltaMiliSeconds < 20) {
		// force main loop to run at 50Hz
		return;
	}
	fast_loopTimer			= millis();
	
	elapsedTime += deltaMiliSeconds;
	
	// Read 3-position switch on radio (usually Ch5)
	// -------------------------------------------------
	read_control_switch_test();
	
	// Read IR SENSORS
	// ---------------
	read_XY_analogs_test();

	// This only fakes out reading the GPS
	// -----------------------------------
	readGPS_test();
	
	if (GPS_update == GPS_BOTH){
		if(invalid_location && current_loc.lat != 0){
			reset_location();
		}
		print_position();

		// GPS event notification
		// ---------------
		gps_event();
	}

	navigate();
	stabilize_AP();

	// Output full telemtry
	// --------------------
	if(millis() - slow_loopTimer > 100){
		print_attitude();
		slow_loopTimer = millis();
	}
		
	// send the throttle value to the PWM out
	// --------------------------------------
	update_throttle();

	// Send an event notice of the main loop
	// -------------------------------------
	mainLoop_event();
}
#endif



