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
	invalid_location = true; 
		
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
