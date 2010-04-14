//test file for roll and pitch on the mega

#define ENABLE_Z_SENSOR 1  // 0 = no Z sensor, 1 = use Z sensor (no Z requires field calibration with each flight)
//2-2
#define XY_SENSOR_LOCATION 0 	//XY Thermopiles Sensor placement
//Mounted right side up: 		0 = cable in front, 1 = cable behind
//Mounted upside down: 			2 = cable in front, 3 = cable behind
//2-3
#define PITCH_TRIM 0 //(Degrees +- 5) allows you to offset bad IR placement degrees*100
//2-4
#define ROLL_TRIM 0 // (Degrees +- 5) allows you to offset bad IR placement degrees*100
#define IR_MAX_FIX .88

boolean MEGA = 1; //1=mega   0=ardupilot

int ir_max				= 300;		// used to scale Thermopile output to 511
int roll_sensor			= 0;		// how much we're turning in degrees * 100
int pitch_sensor			= 0;	

int  analog11				= 511;		// Thermopiles - Pitch
int  analog9				= 511;		// Thermopiles - Roll
int  analog7				= 511;		// Thermopiles - Z
float analog5				= 511;		// Airspeed Sensor - is a float to better handle filtering

void setup()
{
  Serial.begin(115200);
}

void loop()
{
  read_XY_analogs();
  delay(100);
  
  Serial.print("Roll: ");
  Serial.print(roll_sensor);
  Serial.print("      Pitch: ");
  Serial.print(pitch_sensor);
  
  Serial.print("      11: ");
  Serial.print(analog11);
  Serial.print("      9: ");
  Serial.print(analog9);
  
  Serial.print("      Z: ");
  analog7=analogRead(2);
  Serial.println(analog7);
  
  /*Serial.println(analog9);
  Serial.println(analog9-511);
  Serial.println((analog9-511)*30);
  Serial.println(x_axis());
  Serial.println(analog11);
  Serial.println(analog11-511);
  Serial.println((analog11-511)*30);
  Serial.println(y_axis());
  Serial.println((-x_axis() - y_axis()) / 2);
  Serial.println();
  */
}

void read_XY_analogs()
{
    if (MEGA)
    {
        analog11 = analogRead(11);
	analog9 = analogRead(9);
    }
    else
    {
        analog11 = analogRead(0);
        analog9 = analogRead(1);
    }
    
    roll_sensor  = getRoll() + ROLL_TRIM;
    pitch_sensor = getPitch() + PITCH_TRIM;

//#if ENABLE_Z_SENSOR == 1
	//Checks if the roll is less than 10 degrees to read z sensor
	/*if(abs(roll_sensor) <= 1000 && abs(pitch_sensor) <= 1000){
		long analogZ;
                if (MEGA) analogZ=analogRead(7);
                else analogZ=analogRead(2);
                analog7 = ((float)analogZ * 0.95) + ((float)analog7 * .05);
                ir_max = abs(511 - analog7) * IR_MAX_FIX;
		ir_max = constrain(ir_max, 40, 600);
	}*/
//#endif

	
	// uncomment for testing
	//roll_sensor = pitch_sensor = 0;
	
	// Read Airspeed
	/*#if AIRSPEED_SENSOR == 1
		analog5 = ((float)analogRead(5) * .10) + (analog5 * .90);
		airspeed_current	= (int)analog5 - airspeed_offset;
	#endif*/
}

int getRoll(void)
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


int getPitch(void)
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

int x_axis(void)// roll
{
        int a=511;
        int b=30;
        return ((analog9 - a) * b);
        //return ((analog9 - 51l) * 9000) / ir_max;
	//      611 - 511 
	//         100 * 9000 / 100 = 90°  low = underestimate  = 36 looks like 90 = flat plane or bouncy plane
	//         100 * 9000 / 250 = 36°   				    = 36 looks like 36
	//		   100 * 9000 / 500 = 18°  high = over estimate = 36 looks like 18 = crash plane
}

int y_axis(void)// pitch
{
	int a=511;
        int b=30;
        return ((analog11 - a) * b);
        //return ((analog11 - 51l) * 9000) / ir_max;
}
