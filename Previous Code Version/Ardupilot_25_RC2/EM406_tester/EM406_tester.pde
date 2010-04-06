#include <avr/io.h>
#include <math.h>
#include "defines.h"

//To use the header file in your library, use brackets:
//#include <easystar.h>

//To use the header file in your local folder, use quotes:
#include "easystar_25.h"


// System Timers
// --------------
unsigned long fast_loopTimer	= 0;	// Time in miliseconds of main control loop
unsigned long deltaMiliSeconds 	= 0;	// Delta Time in miliseconds

long 	ground_course 		= 0;			// degrees * 100 dir of plane 
// GPS variables
// -------------
byte 	GPS_fix				= BAD_GPS;		// This variable store the status of the GPS
boolean gps_failure			= false;		// not currently used
float 	ground_speed 		= 0;			// meters/second
float 	climb_rate 			= 0;			// meters/second
byte 	GPS_update			= GPS_NONE;		// do we have GPS data to work with?
const float t7				= 10000000.0;	// used to scale GPS values for EEPROM storage



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

// 3D Location vectors
// -------------------
struct Location {
	long lat;
	long lng;
	long alt;
};

struct Location current_loc 		= {0,0,0};		// current location

// Basic Initialization
//---------------------
void setup() {
	pinMode(7,OUTPUT);	// PD7 - AIN0		- Remove Before Fly input pin		- 
	pinMode(12,OUTPUT); // PB4 - MISO		- Blue LED pin  - GPS Lock			- GPS Lock

	#if SHIELD_VERSION == 0 // v1 Shield
	digitalWrite(7, LOW);
	#endif
	
	#if SHIELD_VERSION == 1 
	// v2 Shield
	digitalWrite(7, HIGH);
	#endif

	Serial.begin(57600);
	init_gps();
}


void loop()
{
	// system Timers
	// ----------------------------------------
	deltaMiliSeconds 	= millis() - fast_loopTimer;
	if (deltaMiliSeconds < 100) {
		// force main loop to run at 50Hz
		return;
	}
	fast_loopTimer			= millis();

	
	// Read in the GPS position
	// ------------------------
	decode_gps();
	Serial.print(".");
	
	if (GPS_update & GPS_POSITION){
		Serial.println(" ");
	
		GPS_update = GPS_NONE;
		print_position();
	}	
}


void print_position(void)
{
	//!!377659260|-1224329073|5543|79|-56|5543|0|7982
	Serial.print("lat/lng * 10^7:");
	Serial.print(current_loc.lat,DEC);
	Serial.print(", ");
	Serial.print(current_loc.lng,DEC);
	Serial.print(", alt: ");
	Serial.print(current_loc.alt/100,DEC);
	Serial.print("m, gs: ");
	Serial.print(ground_speed/100,DEC);
	Serial.println("m/s");
}