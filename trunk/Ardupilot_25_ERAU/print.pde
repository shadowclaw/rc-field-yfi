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


