#include <avr/io.h>
#include <math.h>
#include "defines.h"

//To use the header file in your library, use brackets:
//#include <easystar.h>

//To use the header file in your local folder, use quotes:
#include "easystar_25.h"


// System Timers
// --------------
unsigned long fast_loopTimer			= 0;	// Time in miliseconds of main control loop
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
 * uBlox Variables
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

// used to consruct the GPS data from Bytes to ints and longs
// ----------------------------------------------------------
union long_union {
  int32_t dword;
  uint8_t	byte[4];
} 
longUnion;

union int_union {
  int16_t word;
  uint8_t	byte[2];
} 
intUnion;

// 3D Location vectors
// -------------------
struct Location {
  long lat;
  long lng;
  long alt;
};

struct Location current_loc 		= {
  0,0,0};		// current location

// Basic Initialization
//---------------------
void setup() 
{
  Serial.begin(57600);
  Serial1.begin(115200);
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

