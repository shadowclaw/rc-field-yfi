long analog0				= 511;		// Thermopiles
long analog1				= 511;		// Thermopiles
long analog2				= 511;		// Thermopiles
long analog3				= 511;		// Airspeed Sensor
long analog4				= 511;		// Airspeed Sensor
long analog5				= 511;		// Airspeed Sensor


void setup() {
	Serial.begin(38400);
}


void loop() 
{
	analog0 = analogRead(0);
	analog1 = analogRead(1);
	analog2 = analogRead(2);
	analog3 = analogRead(3);
	analog4 = analogRead(4);
	analog5 = analogRead(5);
		
	Serial.print("A0: "); 
	Serial.print(analog0);
	Serial.print(" A1: "); 
	Serial.print(analog1);
	Serial.print(" A2: "); 
	Serial.print(analog2);
	Serial.print(" A3: "); 
	Serial.print(analog3);
	Serial.print(" A4: "); 
	Serial.print(analog4);
	Serial.print(" A5: "); 
	Serial.println(analog5);
	delay(50);
}
