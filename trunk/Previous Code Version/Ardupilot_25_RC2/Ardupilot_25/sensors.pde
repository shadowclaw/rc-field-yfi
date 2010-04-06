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
	// roll_sensor = pitch_sensor = 0;
	airspeed_scaler = 1;
	
	// Read Airspeed
	#if AIRSPEED_SENSOR == 1
		analog3 = ((float)analogRead(3) * .10) + (analog3 * .90);
		airspeed_current	= (int)analog3 - airspeed_offset;
				
		// scale down our roll based on airspeed
		// —————————————————————————————————————
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
// -90°       0         90°	    degree output * 100
// sensors are limited to +- 60° (6000 when you multply by 100)
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
differnce is +=40°
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


