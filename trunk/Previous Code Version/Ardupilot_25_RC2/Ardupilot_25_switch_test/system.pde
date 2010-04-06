/*****************************************************************************

setup the system
- are we in the air or on the ground
- is the throttle at idle? - configure by reading the serial, could be in failsafe or higher value
-

*****************************************************************************/
void init_ardupilot()
{
	Serial.println("Init Ardupilot Control Switch Tester 2.5.0");

 	// ATMEGA ADC
 	// PC0 - ADC0 	- 23 - X sensor
 	// PC1 - ADC1 	- 24 - Y sensor
 	// PC2 - ADC2 	- 25 - Z sensor
 	// PC3 - ADC3 	- 26 - Pressure sensor
 	// PC4 - ADC4 	- 27 - 
 	// PC5 - ADC5 	- 28 - Battery Voltage
 	
 	// ATMEGA
	// PORTD
	// p0				// PD0 - RXD  		- Serial RX 	
	// p1				// PD1 - TXD  		- Serial TX 
	pinMode(2,INPUT);	// PD2 - INT0 		- Rudder in							- INPUT Rudder
	pinMode(3,INPUT);	// PD3 - INT1 		- Elevator in 						- INPUT Elevator
	pinMode(4,INPUT);	// PD4 - XCK/T0 	- MUX pin							- Connected to Pin 2 on ATtiny
	pinMode(5,INPUT);	// PD5 - T0			- Mode pin							- Connected to Pin 6 on ATtiny   - Select on MUX
	pinMode(6,INPUT);	// PD6 - T1			- Remove Before Fly grounded Pin	
	pinMode(7,OUTPUT);	// PD7 - AIN0		- Remove Before Fly input pin		- 
	// PORTB
	pinMode(8,OUTPUT); 	// PB0 - AIN1		- Servo throttle					- OUTPUT THROTTLE
	pinMode(9,OUTPUT);	// PB1 - OC1A		- Elevator PWM out					- Elevator PWM out
	pinMode(10,OUTPUT);	// PB2 - OC1B		- Rudder PWM out					- Rudder PWM out
	pinMode(11,OUTPUT); // PB3 - MOSI/OC2	- GPS status						- 
	pinMode(12,OUTPUT); // PB4 - MISO		- Blue LED pin  - GPS Lock			- GPS Lock
	pinMode(13,INPUT); //  PB5 - SCK		- Yellow LED pin   				- INPUT Throttle

	// setup control switch
	// ----------------
	initControlSwitch();
	reset_control_switch();
}

void set_mode(byte mode)
{
	control_mode = mode;
		
	switch(control_mode)
	{
		case MANUAL:
		break;

		case AUTO:
		break;

		case STABILIZE:
		break;
		
		case FLY_BY_WIRE:
		break;

		case RTL:
		break;
		
		case LOITER:
		break;
	}
	
	// output control mode to the ground station
	print_control_mode();
}

