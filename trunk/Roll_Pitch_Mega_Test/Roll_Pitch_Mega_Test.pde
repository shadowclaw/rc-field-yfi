//test file for roll and pitch on the mega

#define ENABLE_Z_SENSOR 1  // 0 = no Z sensor, 1 = use Z sensor (no Z requires field calibration with each flight)
//2-2
#define XY_SENSOR_LOCATION 0 	//XY Thermopiles Sensor placement
//Mounted right side up: 		0 = cable in front, 1 = cable behind
//Mounted upside down: 			2 = cable in front, 3 = cable behind
//2-3
#define PITCH_TRIM 0 //(Degrees +- 5) allows you to offset bad IR placement
//2-4
#define ROLL_TRIM 0 // (Degrees +- 5) allows you to offset bad IR placement

int ir_max				= 300;		// used to scale Thermopile output to 511
long roll_sensor			= 0;		// how much we're turning in degrees * 100
long pitch_sensor			= 0;	

void read_XY_analogs()
{
	analog11 = analogRead(11);
	analog9 = analogRead(9);

#if ENABLE_Z_SENSOR == 0
	if (analog11 > 511){
		ir_max = max((abs(511 - analog11) * IR_MAX_FIX), ir_max);
		ir_max = constrain(ir_max, 40, 600);
	}
#endif

	roll_sensor  = getRoll() + ROLL_TRIM;
	pitch_sensor = getPitch() + PITCH_TRIM;

	// uncomment for testing
	//roll_sensor = pitch_sensor = 0;
	
	// Read Airspeed
	#if AIRSPEED_SENSOR == 1
		analog5 = ((float)analogRead(5) * .10) + (analog5 * .90);
		airspeed_current	= (int)analog5 - airspeed_offset;
	#endif
}

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
	return ((analog9 - 511l) * 9000l) / ir_max;
	//      611 - 511 
	//         100 * 9000 / 100 = 90°  low = underestimate  = 36 looks like 90 = flat plane or bouncy plane
	//         100 * 9000 / 250 = 36°   				    = 36 looks like 36
	//		   100 * 9000 / 500 = 18°  high = over estimate = 36 looks like 18 = crash plane
}

long y_axis(void)// pitch
{
	return ((analog11 - 511l) * 9000l) / ir_max;
}
