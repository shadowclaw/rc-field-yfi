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
#include "WProgram.h"
void setup();
void loop();
void loop();
void init_gps(void);
void fast_init_gps(void);
void decode_gps(void);
void GPS_join_data(void);
void configure_gps(void);
void change_to_sirf_protocol(void);
void wait_for_GPS_fix(void);
void init_gps(void);
void fast_init_gps(void);
void wait_for_GPS_fix(void);
void decode_gps(void);
void IMU_join_data();
void IMU_join_data();
void checksum(byte data);
void wait_for_data(byte many);
int32_t join_4_bytes(byte Buffer[]);
void init_gps(void);
void fast_init_gps(void);
void wait_for_GPS_fix(void);
void decode_gps(void);
void init_gps(void);
void fast_init_gps(void);
void decode_gps(void);
void GPS_join_data();
int32_t join_4_bytes(byte Buffer[]);
void checksum(byte data);
void wait_for_GPS_fix(void);
void stabilize_AP_mix();
void stabilize_AP();
void stabilize();
void read_control_switch();
void reset_control_switch();
byte readSwitch(void);
void initControlSwitch();
float calc_attitude_roll(float error);
float smooth_attitude_roll(float error);
float calc_attitude_pitch(float error);
int calc_attitude_throttle(float error);
float calc_nav_roll(float error);
long calc_nav_pitch(long error);
long calc_nav_airspeed(long error);
float reset_I(float value);
void failsafe_event();
void switch_event(byte switchPosition);
void waypoint_event(byte event);
void gps_event(void);
void mainLoop_event(void);
void low_battery_event(void);
void navigate();
int get_altitude_above_home(void);
long getDistance(struct Location *loc1, struct Location *loc2);
long get_alt_distance(struct Location *loc1, struct Location *loc2);
float getArea(struct Location *loc1, struct Location *loc2);
long get_bearing2(struct Location *loc1, struct Location *loc2);
long get_bearing(struct Location *loc1, struct Location *loc2);
void print_radio();
void print_current_waypoint();
void print_control_mode(void);
void print_position();
void print_attitude();
void print_new_wp_info();
void print_position(void);
void print_new_wp_info();
void print_attitude(void);
void print_waypoints(byte wp_tot);
void read_radio();
void throttle_failsafe();
void init_radio();
void setup_throttle_trims();
void fast_init_radio();
void read_radio_limits();
void init_analogs(void);
void read_XY_analogs();
void read_analogs(void);
long getRoll(void);
long getPitch(void);
long x_axis(void);
long y_axis(void);
void zero_airspeed(void);
void demo_servos();
void set_servo_mux(boolean mode);
void set_degrees_mix();
void set_degrees();
void set_ch1_degrees(float deg);
void set_ch1_degrees_mix(float deg);
void set_ch2_degrees(float deg);
void set_ch2_degrees_mix(float deg);
void update_throttle();
void init_PWM();
void init_ardupilot();
void startup_air(void);
void startup_ground(void);
void saveLaunchParams(void);
void restoreLaunchParams(void);
void check_eeprom_defaults(void);
void read_eeprom_config(void);
void set_mode(byte mode);
void set_failsafe(boolean mode);
void set_max_altitude_speed(void);
void setGPSMux(void);
void setCommandMux(void);
void read_XY_analogs_test();
void read_control_switch_test();
void readGPS_test(void);
void init_test_location(void);
void save_wp_index();
void return_to_launch();
void reached_waypoint();
void load_waypoint();
void initialize_home();
struct Location set_loc_with_index(struct Location temp, int i);
struct Location get_loc_with_index(int i);
void readPoints();
void reset_location();
byte get_waypiont_mode(void);
void precalc_waypoint_distance(void);
void reset_crosstrack();
void reset_waypoint_index(void);
byte control_mode			= MANUAL;
boolean invalid_location 	= true;		// used to indicate we can't navigate witout good GPS data - the equations will choke

// Radio values
// ------------
int ch1_trim = 0;			// set by init sequence
int ch2_trim = 0;			// set by init sequence
int ch3_trim = CH3_TRIM;	// set by init sequence unless THROTTLE_IN == 0
int ch3_timer_trim = 0; 	// set by init sequence
int ch3_fs;					// set your failsafe throttle >50\u00b5s below ch3 trim

int ch1_in = 1500;						// store rudder position from radio
int ch2_in = 1500;						// store elevator position from radio
int ch3_in = CH3_TRIM;					// store thottle position from radio

int ch1_out = 1500;				// actual \u00b5s values to servos
int ch2_out = 1500;				// actual \u00b5s values to servos
int ch3_out = CH3_TRIM;			// actual \u00b5s values to servos

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
boolean print_telemetry		= false;

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

		if (print_telemetry){
			print_telemetry = false;
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

	}else if ((control_mode >= AUTO) && (control_mode != CONSTANTBANK)){

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

        else if (control_mode == CONSTANTBANK){
          
          nav_pitch=0;
          nav_roll=1000*REVERSE_ROLL;
          nav_airspeed=THROTTLE_CRUISE;
          stabilize_AP();
          
          
        }
	
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
	
	if (print_telemetry){
		print_telemetry = false;
		
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



#if GPS_PROTOCOL == 1

/****************************************************************
 Parsing stuff for SIRF binary protocol. 
 ****************************************************************/
void init_gps(void)
{
	Serial.println("init_gps");
	change_to_sirf_protocol();
	delay(500);//Waits fot the GPS to start_UP
	configure_gps();//Function to configure GPS, to output only the desired msg's
	wait_for_GPS_fix();
}

// If we are in the air:
void fast_init_gps(void)
{
	Serial.begin(FIFTY_SEVEN_K_BAUD); //Universal Sincronus Asyncronus Receiveing Transmiting 
	delay(100);//Waits fot the GPS to start_UP
	configure_gps();//Function to configure GPS, to output only the desired msg's
}


/****************************************************************
 ****************************************************************/
void decode_gps(void)
{
	static unsigned long GPS_timer = 0;
	static byte gps_counter = 0; //Another gps counter for the buffer
	static byte GPS_step = 0;
	static byte gps_ok = 0;//Counter to verify the reciving info
	const byte read_gps_header[]={0xA0,0xA2,0x00,0x5B,0x29};//Used to verify the payload msg header

	if(Serial.available() > 0)//Ok, let me see, the buffer is empty?
	{
		while(Serial.available() < 5){
		}
		switch(GPS_step) {
			case 0: //This case will verify the header, to know when the payload will begin 
				while(Serial.available() > 0)	//Loop if data available
				{
					if(Serial.read() == read_gps_header[gps_ok]){ //Checking if the head bytes are equal..
						//if yes increment 1
						gps_ok++; 
					}else{ 
						//Otherwise restart.
						gps_ok = 0; 
					}
					if(gps_ok >= 5) {
						//Ohh 5 bytes are correct, that means jump to the next step, and break the loop
						gps_ok = 0;
						GPS_step++;
						break;
					}
				}
				break; 
			case 1: //Will receive all the payload and join the received bytes... 
				while(Serial.available() < 92){
				}
				gps_counter = 0;
				memset(gps_buffer,0,sizeof(gps_buffer));
				
				while(Serial.available() > 0){
					//Read data and store it in the temp buffer
					byte b1 = Serial.read();
					byte b2 = gps_buffer[gps_counter-1];
					// gps_ender[]={0xB0,0xB3};	

					if((b1 == gps_ender[1]) && (b2 == gps_ender[0])){
						GPS_step = 0;
						gps_counter = 0;
						GPS_fix = gps_buffer[1];
						
						if(GPS_fix == VALID_GPS){
							// GPS signal is error free
							// ------------------------
							digitalWrite(12,HIGH);
							GPS_timer = millis();
							GPS_update = GPS_BOTH;
							gps_failure = false;
							print_telemetry = true;
							
							//Parse the data
							GPS_join_data(); 

						} else {
							// GPS has returned an error code
							// ------------------------------
							GPS_fix = BAD_GPS;
							GPS_update = GPS_NONE;
							//Serial.println("--GPS err");
							digitalWrite(12,LOW);
						}
						
						break;
					}else{
						gps_buffer[gps_counter] = b1;
						gps_counter++;
						
						if (gps_counter >= BUF_LEN){
							//Serial.println(" shit");
							Serial.flush();
							break;
						}
					}
				}
				break;
		}
	}
	if(millis() - GPS_timer > 2000){
		digitalWrite(12, LOW);	//If we don't receive any byte in two seconds turn off gps fix LED... 
		GPS_fix = BAD_GPS;
		GPS_update = GPS_NONE;
		
		if(millis() - GPS_timer > 10000){
			GPS_fix = FAILED_GPS;
			GPS_timer = millis();
			Serial.println("gpsFailure... ");
		}
	}
}

void GPS_join_data(void)
{
	// Read bytes and combine them with Unions
	// ---------------------------------------
	byte j = 22;
	longUnion.byte[3] 	= gps_buffer[j++];
	longUnion.byte[2] 	= gps_buffer[j++];
	longUnion.byte[1] 	= gps_buffer[j++];
	longUnion.byte[0] 	= gps_buffer[j++];
	current_loc.lat		= longUnion.dword;		//Y = lat * 10,000,000


	longUnion.byte[3] 	= gps_buffer[j++];
	longUnion.byte[2] 	= gps_buffer[j++];
	longUnion.byte[1] 	= gps_buffer[j++];
	longUnion.byte[0] 	= gps_buffer[j++];
	current_loc.lng 	= longUnion.dword;		// X = long * 10,000,000

	j = 34;
	longUnion.byte[3] 	= gps_buffer[j++];
	longUnion.byte[2] 	= gps_buffer[j++];
	longUnion.byte[1] 	= gps_buffer[j++];
	longUnion.byte[0] 	= gps_buffer[j++];	
	current_loc.alt 	= longUnion.dword;		// alt in meters * 100

	j = 39;
	intUnion.byte[1] 	= gps_buffer[j++];
	intUnion.byte[0] 	= gps_buffer[j++];
	ground_speed 		= intUnion.word;		// meters/second * 100
	

	if(ground_speed >= 50){
		//Only updates data if we are really moving... 
		intUnion.byte[1] 	= gps_buffer[j++];
		intUnion.byte[0] 	= gps_buffer[j++];
		ground_course 		= intUnion.word;		// degrees * 100
		ground_course 		= abs(ground_course);	//The GPS has a BUG sometimes give you the correct value but negative, weird!! 
	}else{
		//ground_speed 	= 0;
		//ground_course = 0;
	}

	j = 45;
	intUnion.byte[1] 	= gps_buffer[j++];
	intUnion.byte[0] 	= gps_buffer[j++];
	climb_rate 			= intUnion.word;	//m/s * 100

	// We can't navigate until the GPS has output data
	// We can't read GPS while in FBW. Therfore we need to avoid navigating with Old GPS data 
	// or we might send the plan into the ground
	// ------------------------
	if(current_loc.lat == 0){
		GPS_update = GPS_NONE;
		GPS_fix = BAD_GPS;
	}
	
	// clear buffer
	// -------------
	memset(gps_buffer,0,sizeof(gps_buffer));
}


/****************************************************************
 ****************************************************************/
void configure_gps(void)
{
	const byte gps_header[]={
		0xA0,0xA2,0x00,0x08,0xA6,0x00			};//Used to configure Sirf GPS
	const byte gps_payload[]={
		0x02,0x04,0x07,0x09,0x1B			};//Used to configure Sirf GPS
	const byte gps_checksum[]={
		0xA8,0xAA,0xAD,0xAF,0xC1			};//Used to configure Sirf GPS
	const byte cero = 0x00;//Used to configure Sirf GPS

	for(int z=0; z<2; z++)
	{
		for(int x=0; x<5; x++)//Print all messages to setup GPS
		{
			for(int y=0; y<6; y++)
			{
				Serial.print(byte(gps_header[y]));//Prints the msg header, is the same header for all msg..	
			} 
			Serial.print(byte(gps_payload[x]));//Prints the payload, is not the same for every msg
			for(int y=0; y<6; y++)
			{
				Serial.print(byte(cero)); //Prints 6 zeros
			} 
			Serial.print(byte(gps_checksum[x])); //Print the Checksum
			Serial.print(byte(gps_ender[0]));	//Print the Ender of the string, is same on all msg's. 
			Serial.print(byte(gps_ender[1]));	//ender	
		}
	}	
}

void change_to_sirf_protocol(void)
{
	Serial.begin(4800); //First try in 4800
	delay(300);
	for (byte x=0; x<=28; x++)
	{
		Serial.print(byte(gps_buffer[x]));//Sending special bytes declared at the beginning 
	}	
	delay(300);
	Serial.begin(9600); //Then try in 9600 
	delay(300);
	for (byte x=0; x<=28; x++)
	{
		Serial.print(byte(gps_buffer[x]));
	}
	Serial.begin(FIFTY_SEVEN_K_BAUD); //Universal Sincronus Asyncronus Receiveing Transmiting
}

/****************************************************************
 ****************************************************************/
void wait_for_GPS_fix(void)//Wait GPS fix...
{
	Serial.println(" ");
	Serial.println("Wait for GPS");
	GPS_update 	= GPS_NONE;
	GPS_fix 	= BAD_GPS;
	
	do{
		for(int c = 0; c <= 20; c++){
			decode_gps();
			digitalWrite(12,LOW);
			delay(25);
			digitalWrite(12,HIGH);
			delay(25);
		}
		if(GPS_fix == BAD_GPS){
			Serial.println("Still Waiting: No Valid Fix");
		}
	} while(GPS_fix == BAD_GPS && GPS_update == GPS_NONE);
}
#endif

#if GPS_PROTOCOL == 3

byte ck_a = 0;
byte ck_b = 0;

/****************************************************************
 * Here you have all the stuff for data reception from the IMU_GPS
 ****************************************************************/

/*	GPS_update bit flags -
	 - 0x01 bit = gps lat/lon data received
	 - 0x02 bit = gps alt and speed data received
	 - 0x04 bit = IMU data received
	 - 0x08 bit = PROBLEM - No IMU data last second!

#define GPS_NONE 0
#define GPS_POSITION 1
#define GPS_HEADING 2
#define GPS_BOTH 3
#define GPS_IMU 4
#define GPS_IMU_ERROR 8

*/

void init_gps(void)
{
	pinMode(12, OUTPUT);//Status led
	Serial.begin(THIRTY_EIGHT_K_BAUD); //Universal Sincronus Asyncronus Receiveing Transmiting 
	wait_for_GPS_fix();
}

void fast_init_gps(void)
{
	pinMode(12, OUTPUT);//Status led
	Serial.begin(THIRTY_EIGHT_K_BAUD); //Universal Sincronus Asyncronus Receiveing Transmiting 
}

/****************************************************************
 ****************************************************************/
void wait_for_GPS_fix(void)//Wait GPS fix...
{
	Serial.println(" ");
	Serial.println("Wait for GPS");
	do {
		decode_gps();
		digitalWrite(12,LOW);
		delay(25);
		digitalWrite(12,HIGH);
		delay(25);
	} while(GPS_fix != 0x07);//
	//} while(GPS_fix == BAD_GPS);

	digitalWrite(12,HIGH);

}

/****************************************************************
 * 
 ****************************************************************/
	
void decode_gps(void)
{
	static unsigned long IMU_timer=0; //used to set PROBLEM flag if no data is received. 
	static byte IMU_step = 0;
	boolean success = 0;
        int numc = 0;
        byte data;

/*
IMU Message format
Byte(s)		 Value
0-3		 Header "DIYd"
4                Payload length  (either 6 or 18 depending on if new gps data is available
5                Message ID = 2
6,7		 roll				Integer (degrees*100)
8,9		 pitch				Integer (degrees*100)
10,11		 yaw				Integer (degrees*100)
//  The following are not included in all messages
12-15		longitude			Integer (value*10**7)
16-19		latitude			Integer (value*10**7)
20,21		altitude			Integer (meters*10)
22,23		gps speed			Integer (M/S*100)

12,13 or 24,25  checksum
*/


	numc = Serial.available();
	if (numc > 0)
		for (int i=0;i<numc;i++)	// Process bytes received
		{
			data = Serial.read();
			switch(GPS_step)		 //Normally we start from zero. This is a state machine
			{
			case 0:	
				if(data == 0x44) 
					IMU_step++; //First byte of data packet header is correct, so jump to the next step
				break; 
			case 1:	
				if(data == 0x49)
					 IMU_step++;	//Second byte of data packet header is correct
				else 
					IMU_step=0;     //Second byte is not correct so restart to step zero and try again.		 
				break;

			case 2:	
				if(data == 0x59)
					 IMU_step++;	//Third byte of data packet header is correct
				else 
					IMU_step=0;     //Third byte is not correct so restart to step zero and try again.		 
				break;

			case 3:	
				if(data == 0x49)   //******** Need to get right hex code for header
					 IMU_step++;	//Fourth byte of data packet header is correct, Header complete
				else 
					IMU_step=0;     //Fourth byte is not correct so restart to step zero and try again.		 
				break;

			case 4:	
				payload_length = data;
                                checksum(payload_length);
			        IMU_step++;		 
				break;

			case 5:	
				if(data == 0x02) {  //Verify the message ID is correct
					 checksum(data);
                                         IMU_step++;	
                                }
				else 
					IMU_step=0;     //Whoa!  This is not the right message so restart to step zero and try again.		 
				break;
		 
			case 6:				 // Payload data read...
				if (payload_counter < payload_length){	// We stay in this state until we reach the payload_length
					IMU_buffer[payload_counter] = data;
					checksum(data);
					payload_counter++;
				}else{
					IMU_step++; 
				}
				break;
			case 7:
				IMU_ck_a=data;	 // First checksum byte
				GPS_step++;
				break;
			case 8:
				IMU_ck_b=data;	 // Second checksum byte
			 
		// We end the GPS read...
				if((ck_a=IMU_ck_a)&&(ck_b=IMU_ck_a)) {	 // Verify the received checksum with the generated checksum.. 
					IMU_join_data();
	                                success = 1;
				} else {
					success = 0;		//bad checksum
				} 						 
				// Variable initialization
				IMU_step = 0;
				payload_counter = 0;
				ck_a = 0;
				ck_b = 0;
				IMU_timer = millis(); //Restarting timer...
				break;
		        }
	        }		// End for...
	
	if(millis() - IMU_timer > 500){
		digitalWrite(12, LOW);	//If we don't receive any byte in a half second turn off gps fix LED... 
		GPS_fix = BAD_GPS;
                GPS_update = GPS_IMU_ERROR;
	}
}
  
 /****************************************************************
 * 
 ****************************************************************/
void IMU_join_data()
{
	int j=0;

	 //Storing IMU roll
	intUnion.byte[0] = IMU_buffer[j++];
	intUnion.byte[1] = IMU_buffer[j++];
	roll_sensor = intUnion.word;

	 //Storing IMU pitch
	intUnion.byte[0] = IMU_buffer[j++];
	intUnion.byte[1] = IMU_buffer[j++];
	pitch_sensor = intUnion.word;

	 //Storing IMU heading (yaw)
	intUnion.byte[0] = IMU_buffer[j++];
	intUnion.byte[1] = IMU_buffer[j++];
	ground_course = intUnion.word;

        if (payload_length = 6)
            GPS_update = GPS_IMU;
        else {				
            GPS_update = GPS_IMU || GPS_BOTH;
            GPS_fix = VALID_GPS;	
					 
	    current_loc.lng = join_4_bytes(&IMU_buffer[j]);
	    j += 4;
	    current_loc.lat = join_4_bytes(&IMU_buffer[j]);
	    j += 4;
	    //Storing GPS Height above the sea level
	    intUnion.byte[0] = IMU_buffer[j++];
	    intUnion.byte[1] = IMU_buffer[j++];
	    current_loc.alt = intUnion.word * 10; //  ***** Need to clarify here on the scaling   
	    //Storing Speed (3-D) 
	    intUnion.byte[0] = IMU_buffer[j++];
	    intUnion.byte[1] = IMU_buffer[j++];
	    ground_speed = (float)intUnion.word;

	    digitalWrite(12,HIGH);
}
						

/****************************************************************
 * 
 ****************************************************************/
void checksum(byte data)	 // 
{
	ck_a+=data;
	ck_b+=ck_a; 
}
/****************************************************************
 ****************************************************************/
void wait_for_data(byte many)
{
	while(Serial.available() <= many); 
}

 // Join 4 bytes into a long
 // -------------------------
int32_t join_4_bytes(byte Buffer[])
{
	longUnion.byte[0] = *Buffer;
	longUnion.byte[1] = *(Buffer+1);
	longUnion.byte[2] = *(Buffer+2);
	longUnion.byte[3] = *(Buffer+3);
	return(longUnion.dword);
}


#endif
#if GPS_PROTOCOL == 0
/****************************************************************
 Parsing stuff for NMEA
 ****************************************************************/
void init_gps(void)
{
	pinMode(12, OUTPUT);//Status led
	Serial.begin(9600);
	delay(1000);
	Serial.print(LOCOSYS_BAUD_RATE_38400);
	Serial.begin(THIRTY_EIGHT_K_BAUD);
	delay(500);
	Serial.print(LOCOSYS_REFRESH_RATE_250);
	delay(500);
	Serial.print(NMEA_OUTPUT_4HZ);
	delay(500);
	Serial.print(SBAS_OFF);

#if REMZIBI == 1
   init_remzibi_GPS(); 	
#endif

/* EM406 example init
	Serial.begin(4800); //Universal Sincronus Asyncronus Receiveing Transmiting 
	delay(1000);
	Serial.print(SIRF_BAUD_RATE_9600);
 
	Serial.begin(9600);
	delay(1000);
	
	Serial.print(GSV_OFF);
	Serial.print(GSA_OFF);
	
	#if USE_WAAS == 1
	Serial.print(WAAS_ON);
	#else
	Serial.print(WAAS_OFF);
	#endif*/
	
	wait_for_GPS_fix();
}
void fast_init_gps(void)
{
	pinMode(12, OUTPUT);//Status led
	//Serial.begin(9600); //Universal Sincronus Asyncronus Receiveing Transmiting 
	Serial.begin(THIRTY_EIGHT_K_BAUD);
}

void wait_for_GPS_fix(void)//Wait GPS fix...
{
	Serial.println(" ");
	Serial.println("Wait for GPS");
	do {
		decode_gps(); //Reading and parsing GPS data	
		digitalWrite(12,HIGH);
		delay(25);
		digitalWrite(12,LOW);
		delay(25);
	}
	while((GPS_update & GPS_POSITION != GPS_POSITION) & (GPS_update & GPS_HEADING != GPS_HEADING));
}

void decode_gps(void)
{
	const char head_rmc[]="GPRMC"; //GPS NMEA header to look for
	const char head_gga[]="GPGGA"; //GPS NMEA header to look for
	
	static unsigned long GPS_timer = 0; //used to turn off the LED if no data is received. 
	
	static byte unlock = 1; //some kind of event flag
	static byte checksum = 0; //the checksum generated
	static byte checksum_received = 0; //Checksum received
	static byte counter = 0; //general counter

	//Temporary variables for some tasks, specially used in the GPS parsing part (Look at the NMEA_Parser tab)
	unsigned long temp = 0;
	unsigned long temp2 = 0;
	unsigned long temp3 = 0;


	while(Serial.available() > 0)
	{
		if(unlock == 0)
		{
			gps_buffer[0] = Serial.read();//puts a byte in the buffer

			if(gps_buffer[0]=='$')//Verify if is the preamble $
			{
				counter 	= 0;
				checksum 	= 0;
				unlock		= 1;
			}
		} else {
			gps_buffer[counter] = Serial.read();

			if(gps_buffer[counter] == 0x0A)//Looks for \F
			{
				unlock = 0;

				if (strncmp (gps_buffer, head_rmc, 5) == 0)//looking for rmc head....
				{

					/*Generating and parsing received checksum, */
					for(int x=0; x<100; x++)
					{
						if(gps_buffer[x]=='*')
						{ 
							checksum_received = strtol(&gps_buffer[x + 1], NULL, 16);//Parsing received checksum...
							break; 
						}
						else
						{
							checksum ^= gps_buffer[x]; //XOR the received data... 
						}
					}

					if(checksum_received == checksum)//Checking checksum
					{
						/* Token will point to the data between comma "'", returns the data in the order received */
						/*THE GPRMC order is: UTC, UTC status , Lat, N/S indicator, Lon, E/W indicator, speed, course, date, mode, checksum*/
						token = strtok_r(gps_buffer, search, &brkb); //Contains the header GPRMC, not used

						token = strtok_r(NULL, search, &brkb); //UTC Time, not used
						//time=	atol (token);
						token = strtok_r(NULL, search, &brkb); //Valid UTC data? maybe not used... 


						//Longitude in degrees, decimal minutes. (ej. 4750.1234 degrees decimal minutes = 47.835390 decimal degrees)
						//Where 47 are degrees and 50 the minutes and .1234 the decimals of the minutes.
						//To convert to decimal degrees, devide the minutes by 60 (including decimals), 
						//Example: "50.1234/60=.835390", then add the degrees, ex: "47+.835390 = 47.835390" decimal degrees
						token = strtok_r(NULL, search, &brkb); //Contains Latitude in degrees decimal minutes... 

						//taking only degrees, and minutes without decimals, 
						//strtol stop parsing till reach the decimal point "."	result example 4750, eliminates .1234
						temp = strtol (token, &pEnd, 10);

						//takes only the decimals of the minutes
						//result example 1234. 
						temp2 = strtol (pEnd + 1, NULL, 10);

						//joining degrees, minutes, and the decimals of minute, now without the point...
						//Before was 4750.1234, now the result example is 47501234...
						temp3 = (temp * 10000) + (temp2);


						//modulo to leave only the decimal minutes, eliminating only the degrees.. 
						//Before was 47501234, the result example is 501234.
						temp3 = temp3 % 1000000;


						//Dividing to obtain only the de degrees, before was 4750 
						//The result example is 47 (4750/100 = 47)
						temp /= 100;

						//Joining everything and converting to float variable... 
						//First i convert the decimal minutes to degrees decimals stored in "temp3", example: 501234/600000 =.835390
						//Then i add the degrees stored in "temp" and add the result from the first step, example 47+.835390 = 47.835390 
						//The result is stored in "lat" variable... 
						//lat = temp + ((float)temp3 / 600000);
						current_loc.lat		= (temp * t7) + ((temp3 *100) / 6);

						token = strtok_r(NULL, search, &brkb); //lat, north or south?
						//If the char is equal to S (south), multiply the result by -1.. 
						if(*token == 'S'){
							current_loc.lat *= -1;
						}

						//This the same procedure use in lat, but now for Lon....
						token = strtok_r(NULL, search, &brkb);
						temp = strtol (token,&pEnd, 10); 
						temp2 = strtol (pEnd + 1, NULL, 10); 
						temp3 = (temp * 10000) + (temp2);
						temp3 = temp3%1000000; 
						temp/= 100;
						//lon = temp+((float)temp3/600000);
						current_loc.lng		= (temp * t7) + ((temp3 * 100) / 6);

						token = strtok_r(NULL, search, &brkb); //lon, east or west?
						if(*token == 'W'){
							current_loc.lng *= -1;
						}

						token = strtok_r(NULL, search, &brkb); //Speed overground?
						ground_speed = atoi(token) * 100;

						token = strtok_r(NULL, search, &brkb); //Course?
						ground_course = atoi(token) * 100;
						
						GPS_update |= GPS_POSITION; //Update the flag to indicate the new data has arrived. 

					}
					checksum = 0;
				}//End of the GPRMC parsing

				if (strncmp (gps_buffer, head_gga, 5) == 0)//now looking for GPGGA head....
				{
					/*Generating and parsing received checksum, */
					for(int x = 0; x<100; x++)
					{
						if(gps_buffer[x]=='*')
						{ 
							checksum_received = strtol(&gps_buffer[x + 1], NULL, 16);//Parsing received checksum...
							break; 
						}
						else
						{
							checksum^= gps_buffer[x]; //XOR the received data... 
						}
					}

					if(checksum_received== checksum)//Checking checksum
					{
						//strcpy(gps_GGA,gps_buffer);

						token = strtok_r(gps_buffer, search, &brkb);//GPGGA header, not used anymore
						token = strtok_r(NULL, search, &brkb);//UTC, not used!!
						token = strtok_r(NULL, search, &brkb);//lat, not used!!
						token = strtok_r(NULL, search, &brkb);//north/south, nope...
						token = strtok_r(NULL, search, &brkb);//lon, not used!!
						token = strtok_r(NULL, search, &brkb);//wets/east, nope
						token = strtok_r(NULL, search, &brkb);//Position fix, used!!
						GPS_fix = atoi(token); 
						if(GPS_fix >= 1){
							GPS_fix = VALID_GPS;
							print_telemetry = true;
						}else{
							GPS_fix = BAD_GPS;
						}
						token = strtok_r(NULL, search, &brkb); //sats in use!! Nein...
						token = strtok_r(NULL, search, &brkb);//HDOP, not needed
						token = strtok_r(NULL, search, &brkb);//ALTITUDE, is the only meaning of this string.. in meters of course. 
						//alt_MSL = atoi(token);
						//if(alt_MSL<0){
						//	alt_MSL = 0;
						//}
						current_loc.alt = abs(atoi(token)) * 100;
						
						if(GPS_fix== VALID_GPS) digitalWrite(12, HIGH); //Status LED...
						else digitalWrite(12, LOW);
						
						GPS_update |= GPS_HEADING; //Update the flag to indicate the new data has arrived.
					}
					checksum = 0; //Restarting the checksum
				}

				for(int a = 0; a<= counter; a++)//restarting the buffer
				{
					gps_buffer[a]= 0;
				} 
				counter = 0; //Restarting the counter
				GPS_timer = millis(); //Restarting timer...
			}
			else
			{
				counter++; //Incrementing counter
				if (counter >= 200)
				{
					//Serial.flush();
					counter = 0;
					checksum = 0;
					unlock = 0;
				}
			}
		}
	}
	
	if(millis() - GPS_timer > 2000){
		digitalWrite(12, LOW);	//If we don't receive any byte in two seconds turn off gps fix LED... 
		GPS_fix = BAD_GPS; 
	}
}
#endif

#if GPS_PROTOCOL == 2
/****************************************************************
 * Here you have all the parsing stuff for uBlox
 ****************************************************************/
//You have to disable all the other string, only leave this ones:

//NAV - POSLLH Geodetic Position Solution, PAGE 66 of datasheet
//NAV - VELNED Velocity Solution in NED, PAGE 71 of datasheet
//NAV - STATUS Receiver Navigation Status, PAGE 67 of datasheet

 /*
 GPS_fix Type 
 - 0x00 = no fix
 - 0x01 = dead reckonin
 - 0x02 = 2D - fix
 - 0x03 = 3D - fix
 - 0x04 = GPS + dead re
 - 0x05 = Time only fix
 - 0x06..0xff = reserved
 */

//Luckly uBlox has internal EEPROM so all the settings you change will remain forever.	Not like the SIRF modules! =P

void init_gps(void)
{
	pinMode(12, OUTPUT);//Status led
	Serial.begin(THIRTY_EIGHT_K_BAUD); //Universal Sincronus Asyncronus Receiveing Transmiting 
	wait_for_GPS_fix();
}

void fast_init_gps(void)
{
	pinMode(12, OUTPUT);//Status led
	Serial.begin(THIRTY_EIGHT_K_BAUD); //Universal Sincronus Asyncronus Receiveing Transmiting 
      wait_for_GPS_fix();
}

/****************************************************************
 * 
 ****************************************************************/
void decode_gps(void)
{
	static unsigned long GPS_timer = 0;
	static byte GPS_step = 0;
	byte data;
	int numc;
	
	numc = Serial.available();
	if (numc > 0){
		delay(3); // added delay to help with servo hangs
		for (int i=0; i<numc; i++){	// Process bytes received
			data = Serial.read();
			switch(GPS_step){		 //Normally we start from zero. This is a state machine
				case 0:	
					if(data==0xB5)	// UBX sync char 1
						GPS_step++;	 //ooh! first data packet is correct. Jump to the next step.
					break; 
				case 1:	
					if(data==0x62)	// UBX sync char 2
						GPS_step++;	 //ooh! The second data packet is correct, jump to the step 2
					else 
						GPS_step=0;	 //Nope, incorrect. Restart to step zero and try again.		 
					break;
				case 2:
					UBX_class = data;
					checksum(UBX_class);
					GPS_step++;
					break;
				case 3:
					UBX_id = data;
					checksum(UBX_id);
					GPS_step++;
					break;
				case 4:
					UBX_payload_length_hi = data;
					checksum(UBX_payload_length_hi);
					GPS_step++;
					break;
				case 5:
					UBX_payload_length_lo = data;
					checksum(UBX_payload_length_lo);
					GPS_step++;
					break;
				// revised version thx to Christopher Barnes
				case 6: // Payload data read...
					// We need to process the data byte before we check the number of bytes so far
					UBX_buffer[UBX_payload_counter] = data;
					checksum(data);
					UBX_payload_counter++;
					if (UBX_payload_counter < UBX_payload_length_hi) {
						// We stay in this state until we reach the payload_length
					} else {
						GPS_step++; // payload_length reached - next byte is checksum
					}
					break;
				case 7:
					UBX_ck_a=data;	 // First checksum byte
					GPS_step++;
					break;
				case 8:
					UBX_ck_b=data;	 // Second checksum byte
					// We end the GPS read...
					if((ck_a == UBX_ck_a) && (ck_b == UBX_ck_b)){ // Verify the received checksum with the generated checksum..
						GPS_join_data();							 // Parse the new GPS packet
						GPS_timer = millis(); //Restarting timer...
					}
					
					// Variable re-initialization
					GPS_step = 0;
					UBX_payload_counter = 0;
					ck_a = 0;
					ck_b = 0;
					break;
			}
		}
	}
	
	if((millis() - GPS_timer) > 2000){
		digitalWrite(12, LOW);	//If we don't receive any byte in two seconds turn off gps fix LED... 
		GPS_fix = BAD_GPS;
		GPS_update = GPS_NONE;

		if((millis() - GPS_timer) > 10000){
			GPS_fix = FAILED_GPS;
			GPS_timer = millis();
			Serial.println("gpsFailure... ");
			//reinit the GPS Modules
		}
	}
}

/****************************************************************
 * 
 ****************************************************************/
void GPS_join_data()
{
	int j; // our Byte Offset

	if(UBX_class == 0x01) 
	{
		//Checking the UBX ID
		switch(UBX_id){
		
		case 0x02: //ID NAV-POSLLH 
			if(GPS_fix == VALID_GPS){
				j=0;
				iTOW = join_4_bytes(&UBX_buffer[j]);
				
				j = 4;
				current_loc.lng = join_4_bytes(&UBX_buffer[j]);
	
				j = 8;
				current_loc.lat = join_4_bytes(&UBX_buffer[j]);
	
				j = 16;
				current_loc.alt = join_4_bytes(&UBX_buffer[j])/10; //alt_MSL
				
				GPS_update |= GPS_POSITION;
			}
			break;
			
		case 0x03://ID NAV-STATUS 
			//Serial.print("NAV-STATUS: ");
			//Serial.print("UBX_buffer ");
			//Serial.println(UBX_buffer[4],DEC);
			
			if(UBX_buffer[4] >= 0x03){
				GPS_fix = VALID_GPS; //valid position
				digitalWrite(12,HIGH);
				print_telemetry = true;
			} else {
				GPS_fix = BAD_GPS; //invalid position
				digitalWrite(12,LOW);
			}
			break;

		case 0x12:// ID NAV-VELNED 
			if(GPS_fix == VALID_GPS){
				/* not used
				j=16;
				speed_3d = join_4_bytes(&UBX_buffer[j]); // m/s
				*/
				j=20;
				ground_speed = join_4_bytes(&UBX_buffer[j]); // Ground speed 2D			
	
				if (ground_speed >= 120){
					j=24;
					ground_course = join_4_bytes(&UBX_buffer[j]) / 1000; // Heading 2D
				}
				
				GPS_update |= GPS_HEADING;
			}
			break; 
		}
	}
}

 // Join 4 bytes into a long
 // -------------------------
int32_t join_4_bytes(byte Buffer[])
{
	longUnion.byte[0] = *Buffer;
	longUnion.byte[1] = *(Buffer+1);
	longUnion.byte[2] = *(Buffer+2);
	longUnion.byte[3] = *(Buffer+3);
	return(longUnion.dword);
}

void checksum(byte data)
{
	ck_a += data;
	ck_b += ck_a; 
}


void wait_for_GPS_fix(void)//Wait GPS fix...
{
	Serial.println(" ");
	Serial.println("Wait for GPS");
	GPS_update 	= GPS_NONE;
	GPS_fix 	= BAD_GPS;
	
	do{
		for(int c=0; c<=20; c++){
			decode_gps();
			digitalWrite(12,LOW);
			delay(25);
			digitalWrite(12,HIGH);
			delay(25);
		}
		if(GPS_fix == BAD_GPS){
			Serial.println("Still Waiting: No Valid Fix");
		}
	} while(GPS_fix == BAD_GPS && GPS_update == GPS_NONE);
}
#endif

/****************************************************************
	Function that will integrate waypoints, radio, stabilization and failsafe
 ****************************************************************/
void stabilize_AP_mix()
{
	if(GPS_fix == FAILED_GPS){
		nav_roll = HEAD_MAX / 2;
	}

	float inf 	= (float)ch1_in - (float)ch1_trim;
	inf 		= abs(inf);									
	inf 		= min(inf, 400.0);							
	inf 		= ((400.0 - inf) /400.0);
	//servo_roll 		= (nav_roll - roll_sensor) * ROLL_P;
	servo_roll 		= calc_attitude_roll(nav_roll - roll_sensor);
	servo_roll 		= inf * servo_roll;	// scale down roll to mix in stick later

	inf 		= (float)ch2_in - (float)ch2_trim;				
	inf 		= abs(inf);									
	inf 		= min(inf, 400.0);							
	inf 		= ((400.0 - inf) /400.0);
	//servo_pitch 		= (nav_pitch - pitch_sensor) * PITCH_P;
	servo_pitch 		= calc_attitude_pitch(nav_pitch - pitch_sensor);
	servo_pitch 		= inf * servo_pitch;	// scale down pitch to mix in stick later

	// add in some pitch up while turning
	// ----------------------------------
	servo_pitch += abs(nav_roll * PITCH_COMP);
	
	// write out the servo PWM values
	// ------------------------------
	set_degrees_mix();

	
	// NOTE - Airspeed is stored and used in the program as an integer pressure value
	// Use the formula:  pressure = 0.1254 * speed * speed 
	// where speed is the airspeed in meters per second.
	// For example if you want airspeed_min to be 10 meters per second use a value of 13
	// 100 MPH = (about) 45 M/S = (about) 254 calculated pressure value

	// throttle control with airspeed compensation
	// -------------------------------------------

	#if AIRSPEED_SENSOR == 1
		servo_throttle = THROTTLE_CRUISE + calc_attitude_throttle(THROTTLE_CRUISE + nav_airspeed - airspeed_current); // function located in control_attitude.pde 
	#else
		// we don't have an airsensor
		// --------------------------
		servo_throttle = THROTTLE_CRUISE + calc_attitude_throttle(nav_airspeed); // function located in control_attitude.pde 
	#endif
}

//Function that controls rudder/aileron, elevator and throttle to produce desired attitude and airspeed.
void stabilize_AP()     
{
	if(GPS_fix == FAILED_GPS){       
		nav_roll = HEAD_MAX / 2;   		// If we have lost gps and have no ability to navigate we will circle at a gentle bank angle
	}									// This is a second form of failsafe, different from losing radio signal.

	servo_roll 		= calc_attitude_roll(nav_roll - roll_sensor);
	servo_pitch 	= calc_attitude_pitch(nav_pitch - pitch_sensor);
	
	// add in some pitch up while turning to keep the nose from dropping 
	// -----------------------------------------------------------------
	servo_pitch += abs(nav_roll * PITCH_COMP);
	
	// write out the servo PWM values
	// ------------------------------
	set_degrees();

	// throttle control with airspeed compensation
	// -------------------------------------------
	servo_throttle = THROTTLE_CRUISE + calc_attitude_throttle(THROTTLE_CRUISE + nav_airspeed - airspeed_current); // function located in control_attitude.pde 
}

//This function is like stabilize_AP(), but is only used for STABILIZE mode
void stabilize()
{
	// Setup proportional influence based on stick position
	// more stick = less stabilize
	// ---------------------------------------------------------------
	float inf 	= (float)ch1_in - (float)ch1_trim;				
	inf 		= abs(inf);									
	inf 		= min(inf, 200.0);							
	inf 		= ((200.0 - inf) /200.0);					
	//servo_roll 		= inf * ((float)roll_sensor * -ROLL_P);

	servo_roll 		= calc_attitude_roll(-roll_sensor);
	servo_roll 		= inf * servo_roll;// scale down roll to mix in stick later

	// Setup proportional influence based on stick position
	// more stick = less stabilize
	// ----------------------------------------------------
	inf 		= (float)ch2_in - (float)ch2_trim;			
	inf 		= abs(inf);
	inf 		= min(inf, 200.0);					
	inf 		= ((200.0 - inf) /200.0);
	//servo_pitch 		= inf * ((float)pitch_sensor * -PITCH_P);

	servo_pitch 		= calc_attitude_pitch(-pitch_sensor);
	servo_pitch 		= inf * servo_pitch;// scale down pitch to mix in stick later
	
	set_degrees_mix();
}



byte switchPosition 	= 0;
byte oldSwitchPosition 	= 0;

void read_control_switch()
{
	byte switchPosition = readSwitch();
	if (oldSwitchPosition != switchPosition){
		if(failsafe) {
			set_failsafe(false);
		}
		// we have moved the switch
		// tell user about it, it is up to them to deal with it!
		switch_event(switchPosition);
		oldSwitchPosition = switchPosition;

		// reset all the integrators
		// -------------------------
		//integrators[I_ROLL] = 0;
		//integrators[I_PITCH] = 0;
		integrators[I_THROTTLE] = 0;
		integrators[I_NAV_ROLL] = 0;
	}
}

void reset_control_switch()
{
	oldSwitchPosition = 0;
	read_control_switch();
}

byte readSwitch(void){
	if(digitalRead(4) == HIGH){
		if(digitalRead(5) == HIGH){

			// Middle Switch Position
			// ----------------------
			return 2;

		}else{
			// 3rd Switch Position
			// -------------------
			return 3;
		}
	}else{
		// 1st Switch Position
		// ----------------------
		return 1;
	}
}

void initControlSwitch()
{
	oldSwitchPosition = switchPosition = readSwitch();
}
/*
Attitude control functions:
These functions make sure the aircraft holds the desired pitch and roll.
Pitch and roll targets are set by the control_navigation routines.
*/

// input 		: degress
// range 		: +- 3500\u00b0
// output 		: degrees for plane roll
// range 		: +- 2500\u00b0
// Roll_Gain	: .65 (Easystar default)
// ROLL_P = .5	(half the input goes directly back out)
// ROLL_I = .2	(the rest goes out in X seconds)

float calc_attitude_roll(float error)
{
	// Integrator
	// ----------
	integrators[I_ROLL] +=  (error * integrators[I_ROLL] * (float)deltaMiliSeconds)/1000.0f;

	// Limit Integrator
	// ----------------	
	integrators[I_ROLL] = constrain(integrators[I_ROLL], -1000, 1000); // +-10 degrees

	// Sum the errors
	// --------------	
	error = (error * ROLL_P) + integrators[I_ROLL];
	error = constrain(error, ROLL_MIN, ROLL_MAX); // +-25\u00b0
	return smooth_attitude_roll(error);
}

float smooth_attitude_roll(float error)
{
	static float err_I;
		
	// Integrator
	// ----------
	err_I +=  (error * ROLL_Is * (float)deltaMiliSeconds)/1000.0f;
	
	// Limit Integrator
	// ----------------
	float lim = (1 - ROLL_Ps) * abs(error);
	err_I = constrain(err_I, -lim, lim);

	// Sum the errors
	// --------------
	error = (error * ROLL_Ps) + (err_I);
	return error;
}

float calc_attitude_pitch(float error)
{	
	// Integrator
	// ----------
	integrators[I_PITCH] +=  (error * PITCH_I * (float)deltaMiliSeconds)/1000.0f;
	
	// Limit Integrator
	// ----------------
	integrators[I_PITCH] = constrain(integrators[I_PITCH], -1000, 1000); // +-10 degrees
	
	// Sum the errors
	// --------------			
	error = (error * PITCH_P) + integrators[I_PITCH];
	return error;
}


int calc_attitude_throttle(float error)
{
	static float old_throttle_output;
	static float integrators[I_THROTTLE];
	
	integrators[I_THROTTLE] +=  (error * THROTTLE_I * deltaMiliSeconds)/1000.0;

	// Limit Integrator
	// ----------------
	integrators[I_THROTTLE] = constrain(integrators[I_THROTTLE], 0, THROTTLE_I_MAX);

	// Sum the errors
	// --------------			
	error = (error * THROTTLE_ABSOLUTE) + (THROTTLE_P * error) + integrators[I_THROTTLE];
	error = constrain(error, 0, (THROTTLE_MAX - THROTTLE_CRUISE));

	// smooth output
	// --------------
	old_throttle_output = (old_throttle_output*.90) + (error *.10);

	return old_throttle_output; //Returns the result
}
// These are the control functions for navigation 
// ----------------------------------------------



// input 		: degress
// range		: +- 360
// constrain	: +- 3500\u00b0 
// output 		: degrees for plane roll
// range 		: +- 2500\u00b0
// head_p calc = 2500 / 3500  	=  .714
// head_I calc = 1000 / 3500 	= -.285 / 3s = 0.095
float calc_nav_roll(float error)
{
	//static float err_I;
	integrators[I_NAV_ROLL] +=  (error * head_I * deltaMiliSeconds)/1000.0;
	
	// Limit Integrator
	// ----------------
	integrators[I_NAV_ROLL] = constrain(integrators[I_NAV_ROLL], HEAD_I_MIN, HEAD_I_MAX);

	// Sum the errors
	// --------------			
	error = (head_P * error) + integrators[I_NAV_ROLL];
	error = constrain(error, HEAD_MIN, HEAD_MAX);
	return (error * airspeed_scaler);
}


// input 	: meters
// range 	: +- 20m
// output 	: degrees for servos
// range 	: +- 15\u00b0
// p calc = 20 / 15 = 1.5
long calc_nav_pitch(long error)
{
	error = error * altitude_pitch_P;
	// 20 * 1.5 = 15;
	return constrain(error, ALTITUDE_PITCH_MIN, ALTITUDE_PITCH_MAX);
	//						-1500 (15\u00b0)		,	0 (0\u00b0) so we don't stall
}


// input 	: pressure sensor offset
// range 	: +- 20m
// output 	: Percent for throttle
// range 	: +- 15 Airspeed
// p calc = 20 / 15 = 1.33
long calc_nav_airspeed(long error)
{
	error = error * altitude_throttle_P;
	return constrain(error, ALTITUDE_AIRSPEED_MIN, ALTITUDE_AIRSPEED_MAX);// -15 : 15
}



// Zeros out I if we are entering AP for first time
// Keeps outdated data out of our calculations
float reset_I(float value)
{
  if(control_mode < AUTO)
    return 0; 
  else
    return value; 
}

/*
	This event will be called when the failsafe changes
	boolean failsafe reflects the current state
*/
void failsafe_event()
{
	if (failsafe == true){
		Serial.println("****FAILSAFE ON******");
		// This is an example of how to handle a failsafe.
		// Control modes are numbers. Autopilot (5) and higher are under 
		// computer navigation and don't need intervention.
		if (control_mode < AUTO){
			set_mode(RTL);
		}
	}else{
		Serial.println("****FAILSAFE OFF******");
	}
}

/*
	This event will be called when the switch changes
	It is up to the user how to react to a swicth change event
	options are: MANUAL, STABILIZE, FLY_BY_WIRE, AUTO, RTL, LOITER 
	see: defines.h
	
	The three switch postions can be handled by most radios.
	Adjust your seetings to make sure all three positions work.
	If you don't have a 3 postion switch, try a two position one 
	and note which case below activates in each position.
*/
void switch_event(byte switchPosition)
{
	switch(switchPosition)
	{
		case 1: // First position
		set_mode(POSITION_1);
		break;

		case 2: // middle position
		//set_mode(RTL);
		set_mode(POSITION_2);
		break;

		case 3: // last position
		set_mode(POSITION_3);
		break;
	}
}

void waypoint_event(byte event)
{
	switch(event)
	{
		case EVENT_WILL_REACH_WAYPOINT:
			// called just before wp_index is incemented
			Serial.print("Reached WP:");
			Serial.println(wp_index,DEC);
			break;
			
		case EVENT_SET_NEW_WAYPOINT_INDEX:
			// called just after wp_index is incemented
			Serial.print("Now going to WP:");
			Serial.println(wp_index,DEC);
			break;

		case EVENT_LOADED_WAYPOINT:
			Serial.print("Loaded WP index:");
			Serial.println(wp_index,DEC);
			print_current_waypoint();
			
			// custom loitering code
			/*
			if (wp_index == 2){
				set_mode(LOITER);
				elapsedTime = 0;
			}*/
			break;
			
		// called when the pattern to be flown is automatically restarted
		case EVENT_LOOP: 
			Serial.println("Looped WP Index");
			print_current_waypoint();
			break;			
			
	}
}

void gps_event(void)
{


}

// called after every single control loop
void mainLoop_event(void)
{
/*
	if (control_mode == LOITER){
		if (wp_index == 2 && elapsedTime > 120000 ){ // 2 minutes
			elapsedTime = 0;
			// our waypoints index is not altered during LOITER
			// All we need to do is reload the waypoint
			load_waypoint();
			// and return to Autopilot mode!
			set_mode(AUTO);
		}
	}
*/
}

void low_battery_event(void)
{
	Serial.println("Low Battery");
	set_mode(RTL);
}



		
float dlat,dlng;

void navigate()
{
	// do not navigate with corrupt data
	// ---------------------------------
	if (invalid_location)
	{
		nav_roll = 0;
		nav_pitch = 0;
		return;
	}
	
	
	if(GPS_update & GPS_HEADING)
	{
		GPS_update ^= GPS_HEADING;
		/* 
		this is what the GPS provides:
		ground_course	- 0 to 359 degrees *100
		ground_speed  	- m/s * 100
		climb_rate		- m/s * 100
		*/
		// guess the climb rate
		// --------------------
		#if DEBUG == 1
			if(pitch_sensor >= 0){
				climb_rate = (pitch_sensor * CLIMBRATE_UP * (long)deltaMiliSeconds) / 90000L;
			}else{
				climb_rate = (pitch_sensor * CLIMBRATE_DOWN * (long)deltaMiliSeconds) / 90000L;
			}
		#endif
		
		/*
		actual_turn_rate = old_ground_course - ground_course;
		
		if (actual_turn_rate <-18000)
			actual_turn_rate += 36000;
		if (actual_turn_rate >18000)
			actual_turn_rate -= 36000;
			
		old_ground_course = ground_course;
		*/
		
	}else{
			
		#if DEBUG == 1
		// Dead Reckon:
		
			// clamp the roll sensor so wildness doesn't ensue
			// ----------------------------------------------
			long roll_sensorClamp = constrain(roll_sensor, -4500, 4500);
			
			// run simulation to arrive at intermediate values
			// -----------------------------------------------
			est_turn_rate = (roll_sensorClamp * (long)TURNRATE * (long)deltaMiliSeconds) / 90000L;
			// 4500 * 130 * 1000 / 90000 = 6500
					
			// Integrate the turn rate guess - GPS will overwrite this val 
			// -----------------------------------------------------------
			ground_course += est_turn_rate;
	
			// Save turn rate for the print function
			// -------------------------------------
			est_turn_rate = -(roll_sensorClamp * (long)TURNRATE) / 90L;
			// 4500 * 130 / 90 = 6500
	
			// wrap ground_course values
			// -------------------------
			if (ground_course > 36000)	ground_course -= 36000;
			if (ground_course < 0) 		ground_course += 36000;
		#endif
	}
	
	if(GPS_update & GPS_POSITION)
	{
		GPS_update ^= GPS_POSITION;

		/*
		this is what the GPS provides:
		location		- [lat*10000000, long*10000000, alt*100]
		*/
		
		#if DEBUG == 1
			dlat 	= dlng = 0;
			est_loc = current_loc;
		#endif
	}else{
		#if DEBUG == 1
			// Dead Reckon:
			// Estimate the location of the aircraft
			// -------------------------------------
			float pb_Rad 		= (float)ground_course * .0001745;
			float dist 			= ((float)(ground_speed * deltaMiliSeconds)) / 1000;
			dlat 				+= cos(pb_Rad) * dist;
			dlng 				+= sin(pb_Rad) * dist;	
			current_loc.lat 	= est_loc.lat + dlat;	// latitude = Y part
			current_loc.lng 	= est_loc.lng + (dlng * scaleLongUp);	// Longitude = X part (scaled)
				// use climb_rate from IR Sensors
			current_loc.alt 	= est_loc.alt + climb_rate;
		#endif
	}
	

	// where we should be heading 
	// --------------------------
	target_bearing 	= get_bearing(&current_loc, &next_WP);

	// waypoint distance from plane
	// ----------------------------
	wp_distance = getDistance(&current_loc, &next_WP);

	// Course Error
	// ------------
	bearing_error = target_bearing - ground_course;
	// negative error = left turn
	// positive error = right turn

	
	if(control_mode == LOITER){
		float power;
		if (wp_distance < LOITER_RADIUS){
			power = (float)wp_distance / (float)LOITER_RADIUS;
			//Serial.print("inside power ");
			//Serial.println((power*100),DEC);
			bearing_error += 18000;
			bearing_error += power  * 9000;
		}else if (wp_distance < (LOITER_RADIUS*2)){
			power = (float)((LOITER_RADIUS*2) - wp_distance) / (float)LOITER_RADIUS;
			//Serial.print("outside power ");
			//Serial.println((power*100),DEC);
			bearing_error += power * -9000;
		}
	}

	// Wrap the values
	// ---------------
	if (bearing_error > 18000)	bearing_error -= 36000;
	if (bearing_error < -18000)	bearing_error += 36000;

	// Crosstrack Error
	// ----------------
	long crosstrack_error = (target_bearing - crosstrack_bearing);

	// Wrap the values
	// ---------------
	if (crosstrack_error > 18000)	crosstrack_error -= 36000;
	if (crosstrack_error < -18000)	crosstrack_error += 36000;

	
	// Add in Crosstrack Error
	// -----------------------
	if (abs(bearing_error) < 25) {
		//bearing_error += sin(radians(crosstrack_error/100)) * wp_distance * XTRACK_GAIN;
	} else if (abs(bearing_error) > 35) {
		// reset crosstrack_bearing
		//reset_crosstrack();
	}
	
	// Calculate the required roll of the plane
	// ----------------------------------------
	nav_roll = calc_nav_roll(bearing_error);
	//Serial.print("bearing_error");
	//Serial.print(bearing_error,DEC);
	//Serial.print("\tnav_roll");
	//Serial.println(nav_roll,DEC);
	// in degrees - positive = right turn; 35\u00b0 limit
		
	// Altitude error
	// --------------
	altitude_error 	= (next_WP.alt - current_loc.alt);
	// altitude_error = "how much to climb"
	// lower than WP =  positive number
	
	// Altitude converted to elevator pitch in degrees
	// ------------------------------------------------
	nav_pitch = calc_nav_pitch(altitude_error);
	// pitch down is negative


	// Throttle is in percent or airspeed value, depaending on your setup.
	// change AIRSPEED_SENSOR in your header file to reflect your setup.
	// 
	// throttle cruising speed is the % throttle you want to be at if things are level and no wind
	// Altitude based throttle control + 15as throttle adjustment
	// ---------------------------------------------------------
	nav_airspeed = calc_nav_airspeed(altitude_error);
		


	// Are we there yet?
	// wp_distance is in ACTUAL meters, not the *100 meters we get from the GPS
	// ------------------------------------------------------------------------
	if (control_mode == LOITER){
		// nothing to do really;
		
	}else if (wp_distance < 1){
		//
		// something went wrong!!!
		// if our waypoints are too large we can get an wrapped number
		// ----------------------------------------------------------------	
		Serial.print("wp_distance error, loc: ");
		Serial.print(current_loc.lat);
		Serial.print(", ");
		Serial.print(current_loc.lng);
		Serial.print("  next_WP ");
		Serial.print(next_WP.lat);
		Serial.print(", ");
		Serial.println(next_WP.lng);
		
	}else if(wp_distance < wp_radius) {  //10 meters
	
		//int alt_distance = get_alt_distance(&current_loc, &next_WP);
		//if(alt_distance < WP_ALTITUDE_RADIUS){
			waypoint_event(EVENT_WILL_REACH_WAYPOINT);
			reached_waypoint();
		//}
	}
}

/****************************************************************
Function that will read and store the current altitude when you switch to autopilot mode.
 ****************************************************************/

int get_altitude_above_home(void)
{
	// This is the altitude above the home location
	// The GPS gives us altitude at Sea Level
	// if you slope soar, you should see a negative number sometimes
	// -------------------------------------------------------------
	return current_loc.alt - home.alt;
}

long getDistance(struct Location *loc1, struct Location *loc2)
{
	float dlat 		= (float)loc2->lat - (float)loc1->lat;
	float dlong  	= ((float)loc2->lng - (float)loc1->lng) * scaleLongDown;
	return sqrt(sq(dlat) + sq(dlong)) * .01113195;
}

long get_alt_distance(struct Location *loc1, struct Location *loc2)
{
	return abs(loc1->alt - loc2->alt);
}

float getArea(struct Location *loc1, struct Location *loc2)
{
	return sq((float)(loc2->lat - loc1->lat)) + (sq((float)(loc2->lng - loc1->lng)) * scaleLongDown);
}

long get_bearing2(struct Location *loc1, struct Location *loc2)
{
	return 18000 + atan2((float)(loc1->lng - loc2->lng) * scaleLongDown, (float)(loc1->lat - loc2->lat)) * 5729.57795;
}

long get_bearing(struct Location *loc1, struct Location *loc2)
{
	long off_x = loc2->lng - loc1->lng;
	long off_y = (loc2->lat - loc1->lat) * scaleLongUp;
	long bearing =  9000 + atan2(-off_y, off_x) * 5729.57795;
	if (bearing < 0) bearing += 36000;
	return bearing;
}


/*void print_launch_params(void)
{
	Serial.print("wp_index = \t\t");
	Serial.println(wp_index,DEC);
	Serial.print("wp_total = \t\t");
	Serial.println(wp_total,DEC);
	Serial.print("ch1_trim = \t\t");
	Serial.println(ch1_trim,DEC);
	Serial.print("ch2_trim = \t\t");
	Serial.println(ch2_trim,DEC);
	Serial.print("ch3_trim = \t\t");
	Serial.println(ch3_trim,DEC);	
	Serial.print("ch1_min = \t");
	Serial.println(ch1_min,DEC);
	Serial.print("ch1_max = \t");
	Serial.println(ch1_max,DEC);
	Serial.print("ch2_min = \t");
	Serial.println(ch2_min,DEC);
	Serial.print("ch2_max = \t");
	Serial.println(ch2_max,DEC);
	Serial.print("Home Lat = \t\t");
	Serial.println(home.lat,DEC);
	Serial.print("Home Long = \t\t");
	Serial.println(home.lng,DEC);
	Serial.print("Home altitude = \t");
	Serial.println(home.alt,DEC);
}*/

void print_radio()
{
	Serial.print("R:");
	Serial.print(ch1_in);
	Serial.print("\tE:");
	Serial.print(ch2_in);
	Serial.print("\tT:");
	Serial.println(ch3_in);
}
void print_current_waypoint(){
		Serial.print("prev_WP: ");
		Serial.print("\t");
		Serial.print(prev_WP.lat,DEC);
		Serial.print("\t");
		Serial.print(prev_WP.lng,DEC);
		Serial.print("\t");
		Serial.println(prev_WP.alt,DEC);
		
		Serial.print("next_WP:");
		Serial.print("\t");
		Serial.print(next_WP.lat,DEC);
		Serial.print("\t");
		Serial.print(next_WP.lng,DEC);
		Serial.print("\t");
		Serial.println(next_WP.alt,DEC);
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
                case CONSTANTBANK:
                        Serial.println("##8| CONSTANT BANK");
                        break;
	}
}

#if GROUNDSTATION == 1
void print_position()
{
	//!!377659260|-1224329073|5543|79|-56|5543|0|7982
	Serial.print("!!");
	Serial.print(current_loc.lat,DEC);					// 0
	Serial.print("|");
	Serial.print(current_loc.lng,DEC);					// 1
	Serial.print("|");
	Serial.print(current_loc.alt,DEC);					// 2
	Serial.print("|");
	Serial.print(ground_speed,DEC);					// 3
	Serial.print("|");	
	Serial.print(airspeed_current,DEC);			// 4
	Serial.print("|");
	Serial.print(get_altitude_above_home(),DEC);	// 5
	Serial.print("|");
	Serial.print(climb_rate,DEC);					// 6
	Serial.print("|");
	Serial.print(wp_distance,DEC);					// 7
	Serial.print("|");
	Serial.println(nav_airspeed,DEC);					// 7
}

void print_attitude()
{
	/*
		0 roll - PWM output
		1 pitch - PWM output
		2 roll_sensor - from IR
		3 pitch_sensor - from IR 
		-- add Yaw

		++1786|1704|1000|-270|-1350|8815|10712|
	*/
	Serial.print("++");
	Serial.print(ch1_out,DEC);
	Serial.print("|");
	Serial.print(ch2_out,DEC);
	Serial.print("|");
	Serial.print(ch3_out,DEC);
	Serial.print("|");
	Serial.print(roll_sensor,DEC);
	Serial.print("|");
	Serial.print(pitch_sensor,DEC);
	Serial.print("|");
	Serial.print(ground_course,DEC);				// 9
	Serial.print("|");
	Serial.print(target_bearing,DEC);				// 10
	Serial.println("|");
}

// required by Groundstation to plot lateral tracking course 
void print_new_wp_info()
{
	Serial.print("??");
	Serial.print(wp_index,DEC);			//0
	Serial.print("|");
	Serial.print(prev_WP.lat,DEC);		//1
	Serial.print("|");
	Serial.print(prev_WP.lng,DEC);		//2
	Serial.print("|");
	Serial.print(prev_WP.alt,DEC);		//3
	Serial.print("|");
	Serial.print(next_WP.lat,DEC);		//4
	Serial.print("|");
	Serial.print(next_WP.lng,DEC);		//5
	Serial.print("|");
	Serial.print(next_WP.alt,DEC);		//6
	Serial.print("|");
	Serial.print(wp_totalDistance,DEC);	//7
	Serial.print("|");
	Serial.print(ch1_trim,DEC);			//8
	Serial.print("|");
	Serial.println(ch2_trim,DEC);			//9
}

#endif


#if GROUNDSTATION == 0
void print_position(void)
{
			Serial.print("!!!");
			Serial.print("LAT:");
			Serial.print(current_loc.lat/10,DEC);
			Serial.print(",LON:");
			Serial.print(current_loc.lng/10,DEC); //wp_current_lat
			Serial.print(",SPD:");
			Serial.print(ground_speed/100,DEC);		
			Serial.print(",CRT:");
			Serial.print(climb_rate,DEC);
			Serial.print(",ALT:");
			Serial.print(get_altitude_above_home()/100,DEC);
			Serial.print(",ALH:");
			Serial.print(next_WP.alt/100,DEC);
			Serial.print(",CRS:");
			Serial.print(ground_course/100,DEC);
			Serial.print(",BER:");
			Serial.print(target_bearing/100,DEC);
			Serial.print(",WPN:");
			Serial.print(wp_index,DEC);//Actually is the waypoint.
			Serial.print(",DST:");
			Serial.print(wp_distance,DEC);
			Serial.print(",BTV:");
			Serial.print(battery_voltage,DEC);
			Serial.print(",RSP:");
			Serial.print(servo_roll/100,DEC);
			Serial.println(",***");
}
void print_new_wp_info()
{

}
void print_attitude(void)
{
	Serial.print("QQQ");
	Serial.print("ASP:");
	Serial.print(airspeed_current,DEC);
	Serial.print(",THH:");
	Serial.print(servo_throttle,DEC);
	Serial.print (",RLL:");
	Serial.print(roll_sensor/100,DEC);
	Serial.print (",PCH:");
	Serial.print(pitch_sensor/100,DEC);
	Serial.println(",***");
}
#endif


void print_waypoints(byte wp_tot){
	
	Serial.print("wp_total: ");
	Serial.println(wp_tot, DEC);

	// create a location struct to hold the temp Waypoints for printing
	//Location tmp;
	struct Location tmp = get_loc_with_index(0);
	
	Serial.print("home: \t");
	Serial.print(tmp.lat, DEC);
	Serial.print("\t");
	Serial.print(tmp.lng, DEC);
	Serial.print("\t");
	Serial.println(tmp.alt,DEC);	

	for (int i = 1; i <= wp_tot; i++){
		tmp = get_loc_with_index(i);
		Serial.print("wp #");
		Serial.print(i);
		Serial.print("\t");
		Serial.print(tmp.lat, DEC);
		Serial.print("\t");
		Serial.print(tmp.lng, DEC);
		Serial.print("\t");
		Serial.println(tmp.alt,DEC);
	}
}


//Function that will read the radio data, limit servos and trigger a failsafe
// ----------------------------------------------------------------------------
byte failsafeCounter = 0;		// we wait a second to take over the throttle and send the plain circling

unsigned int timer1count	= 0;
unsigned int timer2count	= 0;
unsigned int timer3count	= 0;

unsigned int timer1diff		= 1500 * 2;
unsigned int timer2diff		= 1500 * 2;
unsigned int timer3diff		= CH3_TRIM * 2;

byte ch_read = 0;
boolean ch1_read = 0;
boolean ch2_read = 0;

void read_radio()
{
	//Filter Radio input
	if(timer1diff > 1700 && timer1diff < 4000){
		
		// if you change the filter value, remember timer1diff is double the servo value
		// so instead of .2, we use .1, 
		// .9 and .05 are valid for example
		ch1_in = (float)ch1_in *.8  + (float)timer1diff *.1;
	}
	
	if(timer2diff > 1700 && timer2diff < 4000){
		ch2_in = (float)ch2_in *.8  + (float)timer2diff *.1;
	}
	
	if(timer3diff > 1700 && timer3diff < 4000){
		ch3_in = (float)ch3_in *.95  + (float)timer3diff *.025;
	}

	#if THROTTLE_IN	== 1
		servo_throttle = (ch3_in - 1000)*.125;
	#endif
}

#if RADIO_TYPE == 0
ISR(PCINT2_vect) {
	int cnt = TCNT1;
	
	if(PIND & B00000100){ 		// ch 1 (pin 2) is high
		ch1_read = 1;
		timer1count = cnt;
	}else if (ch1_read == 1){	// ch 1 (pin 2) is Low
		ch1_read = 0;
		if (cnt < timer1count)   // Timer1 reset during the read of this pulse
		   timer1diff = (cnt + 40000 - timer1count);    // Timer1 TOP = 40000
		else
		  timer1diff = (cnt - timer1count);
	}
	
	if(PIND & B00001000){ 		// ch 1 (pin 2) is high
		ch2_read = 1;
		timer2count = cnt;
	}else if (ch2_read == 1){	// ch 1 (pin 2) is Low
		ch2_read = 0;
		if (cnt < timer2count)   // Timer1 reset during the read of this pulse
		   timer2diff = (cnt + 40000 - timer2count);    // Timer1 TOP = 40000
		else
		  timer2diff = (cnt - timer2count);
	}
}
ISR(PCINT0_vect)
{	
	int cnt = TCNT1;
	if(PINB & B00100000){
		timer3count = cnt;
	}else{	
		if (cnt < timer3count)   // Timer1 reset during the read of this pulse
		   timer3diff = (cnt + 40000 - timer3count);    // Timer1 TOP = 40000
		else
		  timer3diff = (cnt - timer3count);
	}
}
#endif

#if RADIO_TYPE == 1
ISR(PCINT2_vect) {
	int cnt = TCNT1;
	
	if ((PIND & B00000100) && (PIND & B00001000)) {  //LE detected
		// start a new read
		ch_read = 0;
		ch1_read = 1;
		timer1count = cnt;
	} else if((PIND & B00000100) && (!(PIND & B00001000))){ // Ch1 high,Ch2 Low
		if (cnt < timer1count)	// Timer1 reset during the read of this pulse
			timer2diff = (cnt + 40000 - timer1count);		//Timer1 TOP = 40000
		else
			timer2diff = (cnt - timer1count);
			ch_read = 2;
	}else if ((!(PIND & B00000100)) && (PIND & B00001000)){ // Ch1 low,Ch2 high
		if (cnt < timer1count)	// Timer1 reset during theread of this pulse
			timer1diff = (cnt + 40000 - timer1count);		//Timer1 TOP = 40000
		else
		timer1diff = (cnt - timer1count);
		ch_read = 1;  
	} else if ((!(PIND & B00000100)) && (!(PIND & B00001000))){ // Ch1low, Ch2 low
		if (ch_read == 0x00){
			if (cnt < timer1count){
				timer1diff = timer2diff = (cnt + 40000 -timer1count);
			}else{
				timer1diff = timer2diff = (cnt - timer1count);
			} 
		} else if (ch_read == 0x02) {
			if (cnt < timer1count){
				timer1diff = (cnt + 40000 - timer1count);
			}else{
				timer1diff = (cnt - timer1count);
			} 
		} else {
			if (cnt < timer1count){
				timer2diff = (cnt + 40000 - timer1count);
			}else{
				timer2diff = (cnt - timer1count);
			}
		}
	}
}
ISR(PCINT0_vect) {
	int cnt = TCNT1;
	if( ch1_read && (!(PINB & B00100000)) ){
		ch1_read=0;
		if (cnt < timer1count)   // Timer1 reset during the read of this pulse
		   timer3diff = (cnt + 40000 - timer1count);    // Timer1 TOP = 40000
		else
		  timer3diff = (cnt - timer1count);
	}
}
#endif

void throttle_failsafe(){
	#if THROTTLE_IN	== 1

	//check for failsafe and debounce funky reads
	// ------------------------------------------
	if (ch3_in < ch3_fs){
		// we detect a failsafe from radio 
		// throttle has dropped below the mark
		failsafeCounter++;
		if (failsafeCounter == 9)
			Serial.println("*** FAILSAFE DETECTED ***");
		if(failsafeCounter > 10) {
			set_failsafe(true);
			failsafeCounter = 10;
		}
		
		// set throttle cruise value
		servo_throttle = THROTTLE_CRUISE;
		
	}else if(failsafeCounter > 0){
		failsafeCounter--;
		if (failsafeCounter == 0)
			Serial.println("*FAILSAFE OFF*");
		if(failsafeCounter < 1) {
			set_failsafe(false);
			failsafeCounter = 0;
		}
	}
	#endif
}

void init_radio()
{
	// setup radio input timers
	// ------------------------
	fast_init_radio();
	
	// wait until we see the radio
	// ---------------------------
	while(ch1_in < 900 && ch2_in < 900){
		read_radio();
		delay(20);
	}

	// Warm up radio input filters
	// ---------------------------
	for(int c=0; c < 100; c++){
		delay(20);
		read_radio();
	}
	
	// Store the trim values
	// ---------------------
	ch1_trim = ch1_in;
	ch2_trim = ch2_in;
	ch3_trim = ch3_in;

	// Warn if we are out of range
	// ---------------------------
	if((ch1_trim < 1000) || (ch2_trim < 1000)){
		Serial.println("Radio Trim error");
	}
	
	// constrain out of range values
	// -----------------------------
	ch1_trim = constrain(ch1_trim, 950, 2050);
	ch2_trim = constrain(ch2_trim, 950, 2050);
	ch3_trim = constrain(ch3_trim, 950, 2050);

	// Store some trim of the throttle in \u00b5s/8 for the timers
	// to make the internal math simpler
	// ---------------------------------
	setup_throttle_trims();
	
	// Detect that we don't have the throttle to pin 13 connected 
	// ----------------------------------------------------------
	if(ch3_in < 500){
		Serial.print("No Thottle: ");
		Serial.println(ch3_in,DEC);
	}
}

void setup_throttle_trims()
{
	// only called from the ground
	#if REVERSE_THROTTLE == 0
		// auto save FS
		ch3_fs	 		= ch3_trim - 50;
		#if THROTTLE_IN == 1
			ch3_timer_trim	= (float)(ch3_trim - 1000) * .125;
		#else
			ch3_timer_trim = (float)(CH3_TRIM - 1000) * .125;
		#endif
	#else
		ch3_trim = 2000 - ch3_trim;
		ch3_timer_trim = (float)(ch3_trim - 1000) * .125;
	#endif
	
	ch3_timer_trim = constrain(ch3_timer_trim, 0, 125);
}

void fast_init_radio()
{
	#if THROTTLE_IN	== 1
		// enable in change interrupt on PB5 (digital pin 13)
		PCMSK0 = _BV(PCINT5);
	#endif
	
	// enable pin change interrupt on PD2,PD3 (digital pin 2,3)
	PCMSK2 = _BV(PCINT18) | _BV(PCINT19);

	// enable pin change interrupt 2 and 0
	PCICR = _BV(PCIE2);
	
	#if THROTTLE_IN	== 1
		// enable pin change interrupt 2 and 0
		PCICR |= _BV(PCIE0);
	#endif

	// fix for those without Throttle in while starting from Air
	ch3_timer_trim = (float)(CH3_TRIM - 1000) * .125;// set by init sequence
	ch3_timer_trim = constrain(ch3_timer_trim, 0, 125);
}

void read_radio_limits()
{
	// read the extremes and save the XY sensors - hold at an angle for 3 seonds to save
	// -------------------------------------------------
	Serial.println("Reading radio limits:");
	Serial.println("");
	Serial.println("Move sticks to: upper right and lower Left.");
	Serial.println("");
	#if ENABLE_Z_SENSOR == 0
		Serial.println("Calibrate XY IR sensor now.");
		Serial.println("");
	#endif
	Serial.println("To Continue, hold the stick in the corner for 2 seconds.");

	print_radio();
	// set initial servo limits for calibration routine
	// -------------------------------------------------
	ch1_min = ch1_trim - 150;
	ch1_max = ch1_trim + 150;

	ch2_min = ch2_trim - 150;
	ch2_max = ch2_trim + 150;

	// vars for the radio config routine
	// ---------------------------------
	int counter 	= 0;

	// Allows user to set stick limits and calibrate the IR
	// ----------------------------------------------------
	while(counter < 50){
		delay(40);
		read_XY_analogs();
		read_radio();

		// AutoSet servo limits
		// --------------------
		if (ch1_in > 1000 && ch1_in < 2000){
			ch1_min = min(ch1_in, ch1_min);
			ch1_max = max(ch1_in, ch1_max);
		}
		
		if (ch2_in > 1000 && ch2_in < 2000){
			ch2_min = min(ch2_in, ch2_min);
			ch2_max = max(ch2_in, ch2_max);
		}
		if(ch2_in < (ch2_min + 20) || ch2_in > (ch2_max -20)){
			Serial.print(".");
			counter++;
		}else{
			if (counter > 0)
				counter--;
		}
	}
	
	// contstrain min values
	// ---------------------
	ch1_min = constrain(ch1_min, 1000, 2000);
	ch1_max = constrain(ch1_max, 1000, 2000);
	ch2_min = constrain(ch2_min, 1000, 2000);
	ch2_max = constrain(ch2_max, 1000, 2000);
	
	Serial.println(" ");
}


/*
Control Surfaces Worksheet:

Elevator: ch2
down  = positive

	---/	1765		+100
		
	------  1500		
	
	----\	1041		-100


Rudder: ch1
Right turn = positive

   1220  1632  2043
	/     |     \
   /      |      \ 
	

*/


void init_analogs(void)
{
	zero_airspeed();
	read_XY_analogs();
	read_analogs();
}

void read_XY_analogs()
{
	analog0 = analogRead(0);
	analog1 = analogRead(1);	

#if ENABLE_Z_SENSOR == 0
	if (analog0 > 511){
		ir_max = max(analog0 - 511, ir_max);
		ir_max = max(40, ir_max);
	}
#endif
	roll_sensor  = getRoll() + ROLL_TRIM;
	pitch_sensor = getPitch() + PITCH_TRIM;

	// uncomment for testing
	//roll_sensor = pitch_sensor = 0;
	airspeed_scaler = 1;
	
	// Read Airspeed
	#if AIRSPEED_SENSOR == 1
		analog3 = ((float)analogRead(3) * .10) + (analog3 * .90);
		airspeed_current	= (int)analog3 - airspeed_offset;
		
		// uncomment for testing
		//airspeed_current = ch3_in - 1100;
		//airspeed_current = constrain(airspeed_current,0,60);
		
		//ground_course = (((float)(ch3_in - 1100))/800) * 36000;
		//ground_course = constrain(ground_course,0,36000);
		
		// scale down our roll based on airspeed
		// \u2014\u2014\u2014\u2014\u2014\u2014\u2014\u2014\u2014\u2014\u2014\u2014\u2014\u2014\u2014\u2014\u2014\u2014\u2014\u2014\u2014\u2014\u2014\u2014\u2014\u2014\u2014\u2014\u2014\u2014\u2014\u2014\u2014\u2014\u2014\u2014\u2014
		float test 	= (float)airspeed_current;
		test 		= constrain(test, AIRSPEED_MIN_TURN_SPEED, AIRSPEED_MAX_TURN_SPEED);
		test 		= test - AIRSPEED_MIN_TURN_SPEED;
		float test2 = AIRSPEED_MAX_TURN_SPEED - AIRSPEED_MIN_TURN_SPEED;
		airspeed_scaler = AIRSPEED_TURN_P + (test/test2) * AIRSPEED_TURN_P;
			
	#endif

//6-4
//#define AIRSPEED_TURN_P .5 // the amount we scale down our turns when going slow; 1 = no effect
//6-5
//#define AIRSPEED_MAX_TURN_SPEED 25 // turn full on above this airspeed, set less than or equal to THROTTLE_CRUISE 
//6-6
//#define AIRSPEED_MIN_TURN_SPEED 12 // used to limit the effect 

}


void read_analogs(void)
{
#if ENABLE_Z_SENSOR == 1	
	analog2 = ((float)analogRead(2) * 0.01) + ((float)analog2 * .99);
	//Checks if the roll is less than 10 degrees to read z sensor
	if(abs(roll_sensor) <= 1000){
		// fix
		ir_max = abs(511 - analog2);
		ir_max = max(40, ir_max);
	}
#endif

#if BATTERY_EVENT == 1
	analog5 = ((float)analogRead(5)*.01) + ((float)analog5*.99);
	battery_voltage = BATTERY_VOLTAGE(analog5);
	if(battery_voltage < INTPUT_VOLTAGE)
		low_battery_event();
#endif
}



// returns the sensor values as degrees of roll
//   0 ----- 511  ---- 1023    IR Sensor
// -90\u00b0       0         90\u00b0	    degree output * 100
// sensors are limited to +- 60\u00b0 (6000 when you multply by 100)
long getRoll(void)
{
	#if XY_SENSOR_LOCATION ==1
	return constrain((x_axis() + y_axis()) / 2, -6000, 6000);
	#endif
	
	#if XY_SENSOR_LOCATION ==0
	return constrain((-x_axis() - y_axis()) / 2, -6000, 6000);
	#endif

	#if XY_SENSOR_LOCATION ==3
	return constrain((-x_axis() - y_axis()) / 2, -6000, 6000);
	#endif
	
	#if XY_SENSOR_LOCATION ==2
	return constrain((x_axis() + y_axis()) / 2, -6000, 6000);
	#endif
}

long getPitch(void)
{
  #if XY_SENSOR_LOCATION ==1
  return constrain((-x_axis() + y_axis()) / 2, -6000, 6000);
  #endif
  
  #if XY_SENSOR_LOCATION ==0
  return constrain((x_axis() - y_axis()) / 2, -6000, 6000);
  #endif

  #if XY_SENSOR_LOCATION ==3
  return constrain((-x_axis() + y_axis()) / 2, -6000, 6000);
  #endif
  
  #if XY_SENSOR_LOCATION ==2
  return constrain((x_axis() - y_axis()) / 2, -6000, 6000);
  #endif

}

long x_axis(void)// roll
{			
	return ((analog1 - 511l) * 9000l) / ir_max;
}

long y_axis(void)// pitch
{			
	return ((analog0 - 511l) * 9000l) / ir_max;
}


void zero_airspeed(void)
{
	analog3 = analogRead(3);
	for(int c=0; c < 80; c++){
		analog3 = (analog3 * .90) + ((float)analogRead(3) * .10);	
	}
	airspeed_offset = analog3;
}



/*

IR sesor looks at the difference of the two readings and gives a value the same = 511
differnce is +=40\u00b0
(temp dif / 40) * 511
if the top sees the ground its the dif is positive or negative


Analog 0 = Pitch sensor
Analog 0 = Roll sensor - unmarked
   	
	  					   ^ GROUND
		285			 	 713
					 	 P
			 \			/
			  \		  /
				\	/
				511
				/	\
			  /		 \
			/		  \
		 P
	  300				707
	 			||||
				||||
				||||
			 cable			 
				
				
 */


// swing the servos around to show them we're alive
// ------------------------------------------------
void demo_servos()
{
	set_servo_mux(true);
	OCR1A = 1600 * 2;
	OCR1B = 1600 * 2;
	delay(400);
	OCR1A = 1400 * 2;
	OCR1B = 1400 * 2;
	delay(200);
	OCR1A = 1500 * 2;
	OCR1B = 1500 * 2;
	set_servo_mux(false);
}

void set_servo_mux(boolean mode)
{
	while(TCNT1 < 20000){};
	if (mode){
		//take over the MUX
		pinMode(4, OUTPUT);
		digitalWrite(4, HIGH);
	}else{
		//release the MUX to allow Manual Control
		digitalWrite(4, LOW); 
		pinMode(4, INPUT);
	}
}
// wants +- 4500\u00b0
void set_degrees_mix()
{

#if MIXING_MODE == 0
	set_ch1_degrees_mix(servo_roll); // 45 \u00b0 = right turn (unless reversed)
	set_ch2_degrees_mix(servo_pitch);
#endif

  /*V tail mode*/ // needs serious help!
#if MIXING_MODE == 1
	set_ch1_degrees_mix((REVERSE_PITCH * servo_pitch) + (REVERSE_ROLL * servo_roll));	
	set_ch2_degrees_mix((REVERSE_PITCH * servo_pitch) - (REVERSE_ROLL * servo_roll));
#endif
}

// wants +- 4500\u00b0
void set_degrees()
{

#if MIXING_MODE == 0
	set_ch1_degrees(servo_roll);
	set_ch2_degrees(servo_pitch);
#endif

  /*V tail mode*/ // needs serious help!
#if MIXING_MODE == 1
	set_ch1_degrees((REVERSE_PITCH * servo_pitch) + (REVERSE_ROLL * servo_roll));	
	set_ch2_degrees((REVERSE_PITCH * servo_pitch) - (REVERSE_ROLL * servo_roll));
#endif

}

// requires +- 4500\u00b0
void set_ch1_degrees(float deg){
	// 			1520 + 3500 *.11111f = 1520 + 388.885 = right turn;
	ch1_out = ch1_trim + ((float)REVERSE_ROLL * deg * .11111f);
	ch1_out = constrain(ch1_out, 	ch1_min, 	ch1_max);
	ch1_out = constrain(ch1_out, 	1000, 	2000);
	OCR1A = ch1_out * 2;	//OCR1A is the channel 1 pulse width in half microseconds
}

// requires +- 4500\u00b0
void set_ch1_degrees_mix(float deg){
	ch1_out = ch1_in + ((float)REVERSE_ROLL * deg * .11111f);
	ch1_out = constrain(ch1_out, 	ch1_min, 	ch1_max);
	ch1_out = constrain(ch1_out, 	1000, 	2000);
	OCR1A = ch1_out * 2;
}

// requires +- 4500\u00b0
void set_ch2_degrees(float deg){
	ch2_out = ch2_trim + ((float)REVERSE_PITCH * deg * .11111f);
	ch2_out = constrain(ch2_out, 	ch2_min, 	ch2_max);
	ch2_out = constrain(ch2_out, 	1000, 	2000);
	OCR1B = ch2_out * 2;
}
// requires +- 4500\u00b0
void set_ch2_degrees_mix(float deg){
	ch2_out = ch2_in + ((float)REVERSE_PITCH * deg * .11111f);
	ch2_out = constrain(ch2_out, 	ch2_min, 	ch2_max);
	ch2_out = constrain(ch2_out, 	1000, 	2000);
	OCR1B = ch2_out * 2;
}

// sets the throttle timer value based on throttle percent
// -------------------------------------------------------
void update_throttle()
{

#if THROTTLE_OUT == 0
	servo_throttle = 0;
#else
	if (control_mode > STABILIZE){
		servo_throttle = constrain(servo_throttle, 0, THROTTLE_MAX);
	}
#endif

#if REVERSE_THROTTLE == 1
	// 250 - 116.25 +120 = 
	int temp = 250 - servo_throttle + ch3_timer_trim;
	temp = constrain(temp, 137, 250);
	OCR2A = temp;
#endif

#if REVERSE_THROTTLE == 0
	int temp = servo_throttle + 125 + ch3_timer_trim;
	temp = constrain(temp, 137, 250);
	OCR2A = temp;
#endif

	// output for the groundstation
	// ----------------------------
	ch3_out = temp * 8;
}

// Throttle Timer Interrupt
// ------------------------
ISR(TIMER1_CAPT_vect) // Timer/Counter1 Capture Event
{
	//This is a timer 1 interrupts, executed every 20us 
	TCNT2 = 0; //restarting the counter of timer 2
	PORTB |= 0x01; //Putting the pin high!
}

ISR(TIMER2_COMPA_vect) // Timer/Counter2 Compare Match A
{
	// called when TCNT2 == 125:250 which equals 1000 - 2000\u00b5s
	// the counter will increment 1 every 8\u00b5s
	PORTB &= 0xFE;//Putting the pin low
}

void init_PWM()
{
	// Servo setup
	// -----------
	
	// Timer 1
	TCCR1A = ((1<<WGM11) | (1<<COM1B1) | (1<<COM1A1)); //Fast PWM: ICR1=TOP, OCR1x=BOTTOM,TOV1=TOP
	TCCR1B = (1<<WGM13) | (1<<WGM12) | (1<<CS11); // Clock scaler = 8, 2,000,000 counts per second
	OCR1A = 3000;	// Rudder  - multiply your value * 2; for example 3000 = 1500 = 45\u00b0; 4000 = 2000 = 90\u00b0
	OCR1B = 3000; 	// Elevator
	ICR1 = 40000; 	//50hz freq...Datasheet says	(system_freq/prescaler)/target frequency. So (16000000hz/8)/50hz = 40000,

	// throttle;
	//Setting up the Timer 2 - 8 bit timer
	TCCR2A 	= _BV(WGM21); //CTC mode 
	TCCR2B 	= _BV(CS20)|_BV(CS22); //prescaler 128, at 16mhz (128/16) = 8, the counter will increment 1 every 8us
	OCR2A 	= 138; //1500us/8; The top, when the counter reaches the value definied here will execute the interrupt, 187 is the servo centered... 
	TIMSK1 |= _BV(ICIE1); 	// Timer/Counter1, Input Capture Interrupt Enable //PB0 - output throttle
	TIMSK2 	= _BV(OCIE2A);	// Timer/Counter2 Compare Match A
}

/*****************************************************************************

setup the system
- are we in the air or on the ground
- is the throttle at idle? - configure by reading the serial, could be in failsafe or higher value
-

*****************************************************************************/
void init_ardupilot()
{
	Serial.println("Init Ardupilot 2.5.0");
	
 	// ATMEGA ADC
 	// PC0 - ADC0 	- 23 - X sensor
 	// PC1 - ADC1 	- 24 - Y sensor
 	// PC2 - ADC2 	- 25 - Z sensor
 	// PC3 - ADC3 	- 26 - Pressure sensor
 	// PC4 - ADC4 	- 27 - 
 	// PC5 - ADC5 	- 28 - Battery Voltage
 	
 	// ATMEGA
	// PORTD
	// p0				// PD0 - RXD  		- Serial RX 	
	// p1				// PD1 - TXD  		- Serial TX 
	pinMode(2,INPUT);	// PD2 - INT0 		- Rudder in							- INPUT Rudder
	pinMode(3,INPUT);	// PD3 - INT1 		- Elevator in 						- INPUT Elevator
	pinMode(4,INPUT);	// PD4 - XCK/T0 	- MUX pin							- Connected to Pin 2 on ATtiny
	pinMode(5,INPUT);	// PD5 - T0			- Mode pin							- Connected to Pin 6 on ATtiny   - Select on MUX
	pinMode(6,INPUT);	// PD6 - T1			- Remove Before Fly grounded Pin	
	pinMode(7,OUTPUT);	// PD7 - AIN0		- Remove Before Fly input pin		- 
	// PORTB
	pinMode(8,OUTPUT); 	// PB0 - AIN1		- Servo throttle					- OUTPUT THROTTLE
	pinMode(9,OUTPUT);	// PB1 - OC1A		- Elevator PWM out					- Elevator PWM out
	pinMode(10,OUTPUT);	// PB2 - OC1B		- Rudder PWM out					- Rudder PWM out
	pinMode(11,OUTPUT); // PB3 - MOSI/OC2	- GPS status						- 
	pinMode(12,OUTPUT); // PB4 - MISO		- Blue LED pin  - GPS Lock			- GPS Lock
	pinMode(13,INPUT); //  PB5 - SCK		- Yellow LED pin   				- INPUT Throttle

	digitalWrite(6,HIGH);
	
	// Enable GPS
	// ----------------
	setGPSMux();

	// setup control switch
	// ----------------
	initControlSwitch();
		
	// setup PWM timers
	// ----------------
	init_PWM();

	// setup Analog input
	// ----------------
	init_analogs();

	// read EEPROM set by config tool
	// ------------------------------
	read_eeprom_config();

	// read EEPROM to check for first time use
	// ---------------------------------------
	check_eeprom_defaults();

#if DEBUG == 1
	Serial.println("Debug mode enabled");
	ch1_in = 1500;
	ch2_in = 1500;
	ch3_in = 1000;
	startup_air();
#endif

#if DEBUG == 0
	if(digitalRead(6) == LOW){
		startup_ground();
	}else{
		startup_air();
	}
#endif
}

void startup_air(void)
{
	Serial.println("Startup: Air");
	// Makes the servos wiggle
	// -----------------------
	demo_servos();

	// just the basics
	// ----------------
	fast_init_radio();

	// set MUX to GPS
	// --------------
	setGPSMux();
			
#if DEBUG == 1
	// saves some data as the home location
	init_test_location();
#else
	fast_init_gps();// this has given me problems such as the 406 dropping out of Binary mode
#endif

	// load launch settings from EEPROM
	// --------------------------------
	restoreLaunchParams();

	// Output waypints for confirmation
	// --------------------------------
	print_waypoints(wp_total);

	// set the correct flight mode
	// ---------------------------
#if DEBUG == 0
	reset_control_switch();
#endif
}

void startup_ground(void)
{
	Serial.println("Startup: Ground");
	
	// set MUX to GPS
	// --------------
	setGPSMux();

	// prime the radio and set the trims
	// ---------------------------------
	init_radio();
	print_radio();

	// Makes the servos wiggle
	// -----------------------
	demo_servos();
	
#if DEBUG == 0
	// GPS initialization routines
	// this will read in the current location
	// --------------------------------------
	init_gps();	
#else
	// This will setup a fake home location based on 
	// an offset from the first wapypoint
	// ----------------------------------
	init_test_location();
#endif

	// Save location from GPS
	// ----------------------
	initialize_home();
	
	// Load the first waypoint
	// ----------------------
	load_waypoint();

	// Makes the servos wiggle
	// -----------------------
	demo_servos();

	// Output waypints for confirmation
	// --------------------------------
	print_waypoints(wp_total);
	
#if SET_RADIO_LIMITS == 1
	read_radio_limits();
#endif

	// we're done, tell the user with servo twitch
	// -------------------------------------------
	demo_servos();
	delay(1000);
	demo_servos();
	
	// Loop till we remove the safety flag.. 
	// ------------------------------------
	while(digitalRead(6) == LOW){ //Loops until we remove the safetly flag
		Serial.println("Remove Safety Flag");
		demo_servos(); //Testing servos
		decode_gps();  //Decoding GPS
		for (byte n = 0; n < 50; n++){
			read_XY_analogs(); //Reading Analogs		
			delay(20);
		}
	}
	
	// update Home location - to increase accuracy
	// -------------------------------------------
	initialize_home();

	// update analogs
	// --------------
	init_analogs();

	// Save the settings for in-air restart
	// ------------------------------------
	saveLaunchParams();

	// set the correct flight mode
	// ---------------------------
	reset_control_switch();
	
	Serial.println(" ");
	Serial.println("Ready to FLY. ");
}

// if we are starting on the ground
// --------------------------------
void saveLaunchParams(void)
{	
	// Sensor settings
	eeprom_busy_wait();
	eeprom_write_word((uint16_t *)	0x01, airspeed_offset);		eeprom_busy_wait();
	eeprom_write_word((uint16_t *)	0x3E4, ir_max);				eeprom_busy_wait();

	// Radio settings
	eeprom_write_word((uint16_t *)	0x3D6, ch1_trim);			eeprom_busy_wait();
	eeprom_write_word((uint16_t *)	0x3D8, ch2_trim);			eeprom_busy_wait();
	eeprom_write_word((uint16_t *)	0x3DA, ch3_trim);			eeprom_busy_wait();

	eeprom_write_word((uint16_t *)	0x3DC, ch1_min);			eeprom_busy_wait();
	eeprom_write_word((uint16_t *)	0x3DE, ch1_max);			eeprom_busy_wait();
	eeprom_write_word((uint16_t *)	0x3E0, ch2_min);			eeprom_busy_wait();
	eeprom_write_word((uint16_t *)	0x3E2, ch2_max);			eeprom_busy_wait();	
}

// if we are restarting in the air
// -------------------------------
void restoreLaunchParams(void)
{
	// set 0 or home as last index by default
	// --------------------------------------
	wp_index 	= 0;


#if REMEMBER_LAST_WAYPOINT_MODE == 1
	eeprom_busy_wait();
	wp_index 	= eeprom_read_byte((uint8_t *) 0x0A);	// or return current waypoint
#endif

	eeprom_busy_wait();
	wp_total 			= eeprom_read_byte((uint8_t *)	0x09);	eeprom_busy_wait();	//includes home
	airspeed_offset 	= eeprom_read_word((uint16_t *)	0x01);	eeprom_busy_wait(); 
	ir_max				= eeprom_read_word((uint16_t *)	0x3E4);	eeprom_busy_wait(); 

	ch1_trim 			= eeprom_read_word((uint16_t *)	0x3D6);	eeprom_busy_wait();
	ch2_trim 			= eeprom_read_word((uint16_t *)	0x3D8);	eeprom_busy_wait();
	ch3_trim 			= eeprom_read_word((uint16_t *)	0x3DA);	eeprom_busy_wait();

	ch1_min 	= eeprom_read_word((uint16_t *)			0x3DC);	eeprom_busy_wait();
	ch1_max 	= eeprom_read_word((uint16_t *)			0x3DE);	eeprom_busy_wait();
	ch2_min 	= eeprom_read_word((uint16_t *)			0x3E0);	eeprom_busy_wait();
	ch2_max 	= eeprom_read_word((uint16_t *)			0x3E2);	eeprom_busy_wait();

	// lets fix broken values
	// ----------------------
	ch1_trim = constrain(ch1_trim, 	950, 	2050);
	ch2_trim = constrain(ch2_trim, 	950, 	2050);
	ch3_trim = constrain(ch3_trim, 	950, 	2050);

	ch1_min = constrain(ch1_min, 	950, 	2050);
	ch1_max = constrain(ch1_max, 	950, 	2050);
	ch2_min = constrain(ch2_min, 	950, 	2050);
	ch2_max = constrain(ch2_max, 	950, 	2050);
	
	ir_max	 = constrain(ir_max, 150, 512);
	airspeed_offset = constrain(airspeed_offset, 0, 512);
	
	// load home latitude, long, alt
	// -----------------------------
	home = get_loc_with_index(0);
	
	// load next WP
	// ------------
	//next_WP = get_loc_with_index(wp_index);
	load_waypoint();

	// don't trust our location data until we read the GPS
	// ----------------------------------------------------
	invalid_location = true;
}

void check_eeprom_defaults(void)
{
	int test = eeprom_read_word((uint16_t *)	0x3D6);	eeprom_busy_wait();
	if (test < 100){
		eeprom_busy_wait();
		eeprom_write_byte((uint8_t *)	0x3E7, 0);		eeprom_busy_wait();	// 0 = abs, 1 = relative
		eeprom_write_byte((uint8_t *)	0x3E6, 0);		eeprom_busy_wait(); // 0 = return home after 1 trip
		eeprom_write_word((uint16_t *)	0x01,  250);	eeprom_busy_wait();	// air_speed_offset
		eeprom_write_word((uint16_t *)	0x3E4, 200);	eeprom_busy_wait();	// ir_max 
	
		eeprom_write_word((uint16_t *)	0x3D6, 1500);	eeprom_busy_wait();	// ch1_trim
		eeprom_write_word((uint16_t *)	0x3D8, 1500);	eeprom_busy_wait();	// ch2_trim
		eeprom_write_word((uint16_t *)	0x3DA, 1024);	eeprom_busy_wait(); // ch3_trim
	
		eeprom_write_word((uint16_t *)	0x3DC, 1100);	eeprom_busy_wait(); // ch1_min
		eeprom_write_word((uint16_t *)	0x3DE, 1900);	eeprom_busy_wait(); // ch1_max
		eeprom_write_word((uint16_t *)	0x3E0, 1100);	eeprom_busy_wait(); // ch2_min
		eeprom_write_word((uint16_t *)	0x3E2, 1900);	eeprom_busy_wait(); // ch2_max
	}
}

void read_eeprom_config(void)
{
	// Read out wp_total
	// -----------------
	eeprom_busy_wait();
	wp_total = eeprom_read_byte((uint8_t *)0x09);
	Serial.print("wp_total ");
	Serial.println(wp_total,DEC);

	// Read out user options
	// ----------------------
	eeprom_busy_wait();
	config_tool_options = eeprom_read_byte((uint8_t *)	0x00);	
	Serial.print("options ");
	Serial.println(config_tool_options,BIN);

	// Read out user options
	// ----------------------
	eeprom_busy_wait();
	wp_radius = eeprom_read_byte((uint8_t *)	0x0B);
	Serial.print("wp_radius ");
	Serial.println(wp_radius,DEC);
}

void set_mode(byte mode)
{
	control_mode = mode;
	
	// this may not be needed if we always have GPS now
	// ------------------------------------------------
	//invalid_location = true; 
		
	switch(control_mode)
	{
		case MANUAL:
		break;

		case AUTO:
		break;

		case STABILIZE:
		break;
		
		case FLY_BY_WIRE:
		break;

		case RTL:
		return_to_launch();
		break;
		
		case LOITER:
		break;
	}
	
	// output control mode to the ground station
	print_control_mode();
}

void set_failsafe(boolean mode)
{
	// only act on changes
	// -------------------
	if(failsafe != mode){

		// store the value so we don't trip the gate twice
		// -----------------------------------------------
		failsafe = mode;

		if (failsafe == false){
			// We're back in radio contact
			// ---------------------------

			// re-read the switch so we can return to our preferred mode
			reset_control_switch();
			
			// Release hardware MUX
			// ---------------------
			digitalWrite(4, LOW);
			pinMode(4, INPUT);
			//Serial.println("pin 4 low");
			
		}else{
			// We've lost radio contact
			// ---------------------------
			
			// Are we in Manual or Stabilization?
			// ----------------------------------
			if(control_mode < AUTO){
			
				if (failsafe == true){
					// Override hardware MUX
					// ---------------------
					pinMode(4, OUTPUT);
					digitalWrite(4, HIGH);

					// Come home
					// ---------
					set_mode(RTL);
				}else{
					// Release hardware MUX
					// ---------------------
					digitalWrite(4, LOW);
					pinMode(4, INPUT);
				}
			}
		}
		// Let the user know what's up so they can override the behavior
		// -------------------------------------------------------------
		failsafe_event();
	}
}


void set_max_altitude_speed(void)
{
	if(get_altitude_above_home() > max_altitude) {
	  max_altitude = get_altitude_above_home();
	  eeprom_busy_wait();
	  eeprom_write_word((unsigned int*)0x05, (max_altitude/100));
	}
	
	if(ground_speed > max_speed){
	  max_speed = ground_speed;
	  eeprom_busy_wait();
	  eeprom_write_word((unsigned int*)0x07,(max_speed/100));
	}
}

// This hack is to control the V2 shield so we can read the serial from 
// the XBEE radios - which is not implemented yet
void setGPSMux(void)
{
	#if SHIELD_VERSION < 1
		digitalWrite(7, LOW); //Remove Before Fly Pull Up resistor
    #else
		digitalWrite(7, HIGH); //Remove Before Fly Pull Up resistor
	#endif
}

void setCommandMux(void)
{
	#if SHIELD_VERSION < 1
		digitalWrite(7, HIGH); //Remove Before Fly Pull Up resistor
    #else
		digitalWrite(7, LOW); //Remove Before Fly Pull Up resistor
	#endif
}

/* 
ailerons
EEPROM memory map


0 0x00		byte configuration - Bitmap of groundstation toggles - still needs to be defined
1 0x01 		int air_speed_offset
2 0x02 		..
3 0x03 		byte rolltrim
4 0x04 		byte pitchtrim
5 0x05 		uint max_altitude
6 0x06 		..
7 0x07 		uint max_airspeed
8 0x08 		..
9 0x09 		byte wp_total
10 0x0A		byte wp_index
11 0x0B		byte radius
12 0x0C		int home altitude
13 0x0D		..	
14 0x0E		long home latitude
15 0x0F		..
16 0x10		..
17 0x11		..
18 0x12		long home longitude
19 0x13		..
20 0x14		..
21 0x15		..	
22 0x16		Alt_hold_Home
23 0x17 	..	
24 0x18 	long waypoint 1 latitude	
25 0x19 	..	
26 0x1A 	..	
27 0x1B 	..	
28 0x1C 	long waypoint 1 longitude
29 0x1D 	..
30 0x1E 	..
31 0x1F 	..
32 0x20 	int  waypoint 1 altitude
33 0x21 	..
34 0x22 	

...

0x3CE	off_lat
0x3CF	..
0x3D0	..
0x3D1	..
0x3D2	off_lng
0x3D3	..
0x3D4	..
0x3D5	..
0x3D6	int ch1_TRIM
0x3D7	..
0x3D8	int ch2_TRIM
0x3D9	..
0x3DA	int ch3_TRIM
0x3DB	..
0x3DC	int ch1_min
0x3DD	..
0x3DE	int ch1_max
0x3DF	..
0x3E0	int ch2_min
0x3E1	..
0x3E2	int ch2_max
0x3E3	..
0x3E4	int ir_max
0x3E5	..
0x3E6	byte loop_waypoints
0x3E7	byte waypoint_mode



















*/
// For Testing set Debug in header to 1 
// -------------------------------------

void read_XY_analogs_test()
{
	
	roll_sensor = ((float)roll_sensor *.8f) + ((float)nav_roll *.2f);
	pitch_sensor = ((float)pitch_sensor *.8f) + ((float)nav_pitch *.2f);
	//roll_sensor = nav_roll;
	//pitch_sensor = nav_pitch;
	//airspeed_current = THROTTLE_CRUISE;
	
#if THROTTLE_OUT == 0
	servo_throttle = 40;
#endif
	airspeed_scaler = 1;
	airspeed_current = THROTTLE_CRUISE + ((float)airspeed_current *.9f) + ((float)servo_throttle *.1f);
	airspeed_current = constrain(airspeed_current, 0, THROTTLE_MAX);
}

void read_control_switch_test()
{
	// takle over the MUX
	pinMode(4, OUTPUT);
	digitalWrite(4, HIGH);
	//control_mode = AUTO;
}

void readGPS_test(void)
{
	static unsigned long GPS_timer = 0;
	
	//testing 1hz simulation
	if((millis() - GPS_timer) > 1000) {
		GPS_timer = millis();
		ground_speed = (airspeed_current * 1340) / 30;

		//ground_speed			= 1340;
		GPS_fix 				= VALID_GPS;
		GPS_update 				= GPS_BOTH;	
		print_telemetry			= true;
	}
}

// for testing
void init_test_location(void)
{
	Serial.println("debugging - Init Location");
	wp_index = 1;
	save_wp_index();
	
	home = get_loc_with_index(0);
	//if (home.lat == 0){
	
		home = get_loc_with_index(1);
		home.lat += 9500;
		home.lng += 9500;
		set_loc_with_index(home, 0);
	//}
	// save home location
	//home.lat =   377176960;
	//home.lng = -1223811830;
	//home.alt = 0;
	//set_loc_with_index(home, 0);
	
	// Set a location to be at when we come online
	//current_loc.lat =   377162980;
	//current_loc.lng = -1223798250;
	//current_loc.alt = 10000;//8000;
	current_loc 	= home;
	ground_course	= 0;
	control_mode 	= AUTO;
}
void save_wp_index()
{
	eeprom_busy_wait();
	eeprom_write_byte((uint8_t *)0x0A, wp_index);
}

void return_to_launch()
{
	// home is WP 0
	// ------------
	wp_index = 0;

	// Save current waypoint index
	// ---------------------------
	save_wp_index();

	// Loads WP from Memory
	// --------------------
	load_waypoint();

	// Altitude to hold over home
	// Set by configuration tool
	// -------------------------
	if (config_tool_options & HOLD_ALT_ABOVE_HOME){
 		next_WP.alt = current_loc.alt;
 	}else{
		eeprom_busy_wait();
		int hold_alt_above_home = eeprom_read_word((uint16_t *)	0x16);
		next_WP.alt = home.alt + ((long)hold_alt_above_home * 100);
	}
}

void reached_waypoint()
{
	if (control_mode == RTL){
		set_mode(LOITER);
		
	}else if (control_mode == AUTO){
		// load the next waypoint
		// ----------------------
		wp_index++;
		// are we looping or returning home?
		if(wp_index > wp_total){
			eeprom_busy_wait();
			byte loop_waypoints = eeprom_read_byte((uint8_t *)	0x3E6);
			
			if (loop_waypoints > 0){
				Serial.println("Looping, going to waypoint 1");
				waypoint_event(EVENT_LOOP);

				wp_index = 1; // first real waypoint
				waypoint_event(EVENT_SET_NEW_WAYPOINT_INDEX);
				loop_waypoints--;
				waypoint_event(EVENT_LOOP);
				
				// save loop_waypoints counter
				eeprom_busy_wait();
				eeprom_write_byte((uint8_t * )0x3E6, loop_waypoints);
				
			}else {
				Serial.println("Done, Heading home");
				//set_mode(RTL);
                                reset_waypoint_index();
				// nothing more to do here
				return;
			}
		}else{
			Serial.print("moving on to waypoint: ");
			Serial.println(wp_index,DEC);
		}
				
		// Save current waypoint index to EEPROM
		// -------------------------------------
		save_wp_index();
		
		// notify user of new index selection
		// -------------------------------------
		waypoint_event(EVENT_SET_NEW_WAYPOINT_INDEX);
		
		// load next WP
		// ------------
		load_waypoint();
	}
}

// run this whenever the wp_index changes
// -------------------------------------
void load_waypoint()
{
	// copy the current WP into the OldWP slot
	// ---------------------------------------
	prev_WP = current_loc;

	// Load the next_WP slot
	// ---------------------
	next_WP = get_loc_with_index(wp_index);

	// offset the altitude relative to home position
	// ---------------------------------------------
	next_WP.alt += home.alt;

	// let them know we have loaded the WP
	// -----------------------------------
	waypoint_event(EVENT_LOADED_WAYPOINT);
	
	// do this whenever Old and new WP's change
	// ---------------------------------------------
	precalc_waypoint_distance();
}


// run this at setup on the ground
// -------------------------------
void initialize_home()
{
	// Copy our current location to home
	// ---------------------------------
	home = current_loc;
	set_loc_with_index(home, 0);

	prev_WP = home;
	
	// Set the current WP index to 1 (first WP)
	// ----------------------------------------
	wp_index = 1;
}

struct Location set_loc_with_index(struct Location temp, int i)
{
	temp.lat = temp.lat/10;
	temp.lng = temp.lng/10;		// lat and long stored as * 1,000,000
	temp.alt = temp.alt/100; 	// altitude is stored as meters 
	
	if (i == 0){

		// Save Home location to EEPROM
		// ----------------------------
		eeprom_busy_wait();
		eeprom_write_dword((uint32_t *)0x0E, temp.lat);
		eeprom_busy_wait();
		eeprom_write_dword((uint32_t *)0x12, temp.lng);
		eeprom_busy_wait();
		eeprom_write_word((uint16_t *)0x0C, (int)temp.alt);
	
	}else{
	
		// this is not being used right now
		// --------------------------------
		long mem_position = (long)(WP_START_BYTE + (i-1) * WP_10_BYTES);
    	eeprom_busy_wait();		
		eeprom_write_dword((uint32_t *)	mem_position, temp.lat);
		eeprom_busy_wait();
		mem_position += 4;
		eeprom_write_dword((uint32_t *)	mem_position, temp.lng);
		eeprom_busy_wait();
		mem_position += 4;
		eeprom_write_word((uint16_t *)	mem_position, (int)temp.alt);
	}
}


struct Location get_loc_with_index(int i)
{
	struct Location temp;
	long mem_position;

	// Find out proper location in memory by using the start_byte position + the index
	// --------------------------------------------------------------------------------
	if (i == 0) {
		// read home position 
		eeprom_busy_wait();
		temp.lat = (long)eeprom_read_dword((uint32_t*)0x0E);eeprom_busy_wait();
		temp.lng = (long)eeprom_read_dword((uint32_t*)0x12);eeprom_busy_wait();
		temp.alt = 0;
		temp.alt = (long)eeprom_read_word((uint16_t*)0x0C);

		temp.lat *= 10;
		temp.lng *= 10;
		temp.alt *= 100;
		return temp;
		
	}else{
		// read WP position 
		mem_position = (long)(WP_START_BYTE + (i-1) * WP_10_BYTES);
		eeprom_busy_wait();
		temp.lat = (long)eeprom_read_dword((uint32_t*)mem_position);
		mem_position += 4;
		eeprom_busy_wait();
		temp.lng = (long)eeprom_read_dword((uint32_t*)mem_position);
		mem_position += 4;
		temp.alt = 0;
		eeprom_busy_wait();
		temp.alt = (long)eeprom_read_word((uint16_t*)mem_position);
		
		temp.lat *= 10;
		temp.lng *= 10;
		temp.alt *= 100;
		/*
		if(wp_mode == REL_WP || abs(temp.lat) < 10000 ){
			wp_mode = REL_WP;
			temp.lat += home.lat;
			temp.lng += home.lng;
		}
		*/
		
		return temp;
	}
}

void readPoints()
{
    for (byte i = 0; i < wp_total; i++){
    
    	struct Location tmp = get_loc_with_index(i);
    			
		Serial.print("waypoint #");
		Serial.print(i);
		Serial.print("\t");
		Serial.print(tmp.lat,DEC);
		Serial.print("\t");
		Serial.print(tmp.lng,DEC);
		Serial.print("\t");
		Serial.println(tmp.alt,DEC);
	}
}

// reset Current Location to be the originating point
// This allows us to start navigating from an arbitrary point if 
// the system is in RTL
// --------------------
void reset_location()
{
	invalid_location = false;
	
	if (control_mode == LOITER){
		next_WP = current_loc;
		prev_WP = current_loc;
	}else{
		// copy location into previous waypoint
		prev_WP = current_loc;
	}	
	// do our precalcs
	precalc_waypoint_distance();
}

byte get_waypiont_mode(void)
{
	eeprom_busy_wait();
	return eeprom_read_byte((uint8_t*)0x3E7);
}

// Precalc for navigation algorithms
// called on each WP change
// ---------------------------------
void precalc_waypoint_distance(void)
{
	// this is handy for the groundstation
	wp_totalDistance 	= getDistance(&prev_WP, &next_WP);

	// this is used to offset the shrinking longitude as we go towards the poles	
	float rads = (abs(next_WP.lat)/t7) * 0.0174532925;
	//377,173,810 / 10,000,000 = 37.717381 * 0.0174532925 = 0.658292482926943		
	scaleLongDown = cos(rads);
	scaleLongUp = 1.0f/cos(rads);

	// set a new crosstrack bearing
	// ----------------------------
	reset_crosstrack();
	
	// output the new WP information to the Ground Station
	// ---------------------------------------------------
	print_new_wp_info();
}

void reset_crosstrack()
{
	crosstrack_bearing 	= get_bearing(&current_loc, &next_WP);
}

// utility to reset WP index to 0 se we can restart mission
void reset_waypoint_index(void){
	Serial.println("reset_waypoint_index");
	wp_index = 1;	// first WP
	load_waypoint();
}



int main(void)
{
	init();

	setup();
    
	for (;;)
		loop();
        
	return 0;
}

