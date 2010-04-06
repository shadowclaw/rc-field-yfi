/*
Attitude control functions:
These functions make sure the aircraft holds the desired pitch and roll.
Pitch and roll targets are set by the control_navigation routines.
*/

// input 		: degress
// range 		: +- 3500°
// output 		: degrees for plane roll
// range 		: +- 2500°
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
	error = constrain(error, ROLL_MIN, ROLL_MAX); // +-25°
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
