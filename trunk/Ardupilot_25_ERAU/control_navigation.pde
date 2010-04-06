// These are the control functions for navigation 
// ----------------------------------------------



// input 		: degress
// range		: +- 360
// constrain	: +- 3500° 
// output 		: degrees for plane roll
// range 		: +- 2500°
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
// range 	: +- 15°
// p calc = 20 / 15 = 1.5
long calc_nav_pitch(long error)
{
	error = error * altitude_pitch_P;
	// 20 * 1.5 = 15;
	return constrain(error, ALTITUDE_PITCH_MIN, ALTITUDE_PITCH_MAX);
	//						-1500 (15°)		,	0 (0°) so we don't stall
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

