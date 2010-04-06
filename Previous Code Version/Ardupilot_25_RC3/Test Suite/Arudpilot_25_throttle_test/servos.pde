// sets the throttle timer value based on throttle percent
// -------------------------------------------------------
void update_throttle()
{

#if THROTTLE_OUT == 0
	servo_throttle = 0;
#else
	//if (control_mode > STABILIZE){
		servo_throttle = constrain(servo_throttle, 0, THROTTLE_MAX);
	//}
#endif

#if REVERSE_THROTTLE == 1
	// 250 - 116.25 +120 = 
	int temp = 250 - servo_throttle + ch3_timer_trim;
	temp = constrain(temp, 137, 250);
	OCR2A = temp;
#endif

#if REVERSE_THROTTLE == 0
	int temp = servo_throttle + 125 + ch3_timer_trim;
	temp = constrain(temp, 137, 250);
	OCR2A = temp;
#endif

	// output for the groundstation
	// ----------------------------
	ch3_out = temp * 8;
}

// Throttle Timer Interrupt
// ------------------------
ISR(TIMER1_CAPT_vect) // Timer/Counter1 Capture Event
{
	//This is a timer 1 interrupts, executed every 20us 
	TCNT2 = 0; //restarting the counter of timer 2
	PORTB |= 0x01; //Putting the pin high!
}

ISR(TIMER2_COMPA_vect) // Timer/Counter2 Compare Match A
{
	// called when TCNT2 == 125:250 which equals 1000 - 2000µs
	// the counter will increment 1 every 8µs
	PORTB &= 0xFE;//Putting the pin low
}

void init_PWM()
{
	// Servo setup
	// -----------
	
	// Timer 1
	TCCR1A = ((1<<WGM11) | (1<<COM1B1) | (1<<COM1A1)); //Fast PWM: ICR1=TOP, OCR1x=BOTTOM,TOV1=TOP
	TCCR1B = (1<<WGM13) | (1<<WGM12) | (1<<CS11); // Clock scaler = 8, 2,000,000 counts per second
	OCR1A = 3000;	// Rudder  - multiply your value * 2; for example 3000 = 1500 = 45°; 4000 = 2000 = 90°
	OCR1B = 3000; 	// Elevator
	ICR1 = 40000; 	//50hz freq...Datasheet says	(system_freq/prescaler)/target frequency. So (16000000hz/8)/50hz = 40000,

	// throttle;
	//Setting up the Timer 2 - 8 bit timer
	TCCR2A 	= _BV(WGM21); //CTC mode 
	TCCR2B 	= _BV(CS20)|_BV(CS22); //prescaler 128, at 16mhz (128/16) = 8, the counter will increment 1 every 8us
	OCR2A 	= 138; //1500us/8; The top, when the counter reaches the value definied here will execute the interrupt, 187 is the servo centered... 
	TIMSK1 |= _BV(ICIE1); 	// Timer/Counter1, Input Capture Interrupt Enable //PB0 - output throttle
	TIMSK2 	= _BV(OCIE2A);	// Timer/Counter2 Compare Match A
}

