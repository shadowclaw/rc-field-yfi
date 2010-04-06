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
		home.lat -= 9500;
		home.lng -= 9500;
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
