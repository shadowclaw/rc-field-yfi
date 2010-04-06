//Function that will read the radio data, limit servos and trigger a failsafe
// ----------------------------------------------------------------------------
byte failsafeCounter = 0;		// we wait a second to take over the throttle and send the plain circling

unsigned int timer1count	= 0;
unsigned int timer2count	= 0;
unsigned int timer3count	= 0;

unsigned int timer1diff		= 1500 * 2;
unsigned int timer2diff		= 1500 * 2;
unsigned int timer3diff		= CH3_TRIM * 2;

byte ch_read = 0;
boolean ch1_read = 0;
boolean ch2_read = 0;

void read_radio()
{
	//Filter Radio input
	if(timer1diff > 1700 && timer1diff < 4000){
		
		// if you change the filter value, remember timer1diff is double the servo value
		// so instead of .2, we use .1, 
		// .9 and .05 are valid for example
		ch1_in = (float)ch1_in *.8  + (float)timer1diff *.1;
	}
	
	if(timer2diff > 1700 && timer2diff < 4000){
		ch2_in = (float)ch2_in *.8  + (float)timer2diff *.1;
	}
	
	if(timer3diff > 1700 && timer3diff < 4000){
		ch3_in = (float)ch3_in *.95  + (float)timer3diff *.025;
	}

	#if THROTTLE_IN	== 1
		servo_throttle = (ch3_in - 1000)*.125;
	#endif
}

#if RADIO_TYPE == 0
ISR(PCINT2_vect) {
	int cnt = TCNT1;
	
	if(PIND & B00000100){ 		// ch 1 (pin 2) is high
		ch1_read = 1;
		timer1count = cnt;
	}else if (ch1_read == 1){	// ch 1 (pin 2) is Low
		ch1_read = 0;
		if (cnt < timer1count)   // Timer1 reset during the read of this pulse
		   timer1diff = (cnt + 40000 - timer1count);    // Timer1 TOP = 40000
		else
		  timer1diff = (cnt - timer1count);
	}
	
	if(PIND & B00001000){ 		// ch 1 (pin 2) is high
		ch2_read = 1;
		timer2count = cnt;
	}else if (ch2_read == 1){	// ch 1 (pin 2) is Low
		ch2_read = 0;
		if (cnt < timer2count)   // Timer1 reset during the read of this pulse
		   timer2diff = (cnt + 40000 - timer2count);    // Timer1 TOP = 40000
		else
		  timer2diff = (cnt - timer2count);
	}
}
ISR(PCINT0_vect)
{	
	int cnt = TCNT1;
	if(PINB & B00100000){
		timer3count = cnt;
	}else{	
		if (cnt < timer3count)   // Timer1 reset during the read of this pulse
		   timer3diff = (cnt + 40000 - timer3count);    // Timer1 TOP = 40000
		else
		  timer3diff = (cnt - timer3count);
	}
}
#endif

#if RADIO_TYPE == 1
ISR(PCINT2_vect) {
	int cnt = TCNT1;
	
	if ((PIND & B00000100) && (PIND & B00001000)) {  //LE detected
		// start a new read
		ch_read = 0;
		ch1_read = 1;
		timer1count = cnt;
	} else if((PIND & B00000100) && (!(PIND & B00001000))){ // Ch1 high,Ch2 Low
		if (cnt < timer1count)	// Timer1 reset during the read of this pulse
			timer2diff = (cnt + 40000 - timer1count);		//Timer1 TOP = 40000
		else
			timer2diff = (cnt - timer1count);
			ch_read = 2;
	}else if ((!(PIND & B00000100)) && (PIND & B00001000)){ // Ch1 low,Ch2 high
		if (cnt < timer1count)	// Timer1 reset during theread of this pulse
			timer1diff = (cnt + 40000 - timer1count);		//Timer1 TOP = 40000
		else
		timer1diff = (cnt - timer1count);
		ch_read = 1;  
	} else if ((!(PIND & B00000100)) && (!(PIND & B00001000))){ // Ch1low, Ch2 low
		if (ch_read == 0x00){
			if (cnt < timer1count){
				timer1diff = timer2diff = (cnt + 40000 -timer1count);
			}else{
				timer1diff = timer2diff = (cnt - timer1count);
			} 
		} else if (ch_read == 0x02) {
			if (cnt < timer1count){
				timer1diff = (cnt + 40000 - timer1count);
			}else{
				timer1diff = (cnt - timer1count);
			} 
		} else {
			if (cnt < timer1count){
				timer2diff = (cnt + 40000 - timer1count);
			}else{
				timer2diff = (cnt - timer1count);
			}
		}
	}
}
ISR(PCINT0_vect) {
	int cnt = TCNT1;
	if( ch1_read && (!(PINB & B00100000)) ){
		ch1_read=0;
		if (cnt < timer1count)   // Timer1 reset during the read of this pulse
		   timer3diff = (cnt + 40000 - timer1count);    // Timer1 TOP = 40000
		else
		  timer3diff = (cnt - timer1count);
	}
}
#endif

void throttle_failsafe(){
	#if THROTTLE_IN	== 1

	//check for failsafe and debounce funky reads
	// ------------------------------------------
	if (ch3_in < ch3_fs){
		// we detect a failsafe from radio 
		// throttle has dropped below the mark
		failsafeCounter++;
		if (failsafeCounter == 9)
			Serial.println("*** FAILSAFE DETECTED ***");
		if(failsafeCounter > 10) {
			set_failsafe(true);
			failsafeCounter = 10;
		}
		
		// set throttle cruise value
		servo_throttle = THROTTLE_CRUISE;
		
	}else if(failsafeCounter > 0){
		failsafeCounter--;
		if (failsafeCounter == 0)
			Serial.println("*FAILSAFE OFF*");
		if(failsafeCounter < 1) {
			set_failsafe(false);
			failsafeCounter = 0;
		}
	}
	#endif
}

void init_radio()
{
	// setup radio input timers
	// ------------------------
	fast_init_radio();
	
	// wait until we see the radio
	// ---------------------------
	while(ch1_in < 900 && ch2_in < 900){
		read_radio();
		delay(20);
	}

	// Warm up radio input filters
	// ---------------------------
	for(int c=0; c < 100; c++){
		delay(20);
		read_radio();
	}
	
	// Store the trim values
	// ---------------------
	ch1_trim = ch1_in;
	ch2_trim = ch2_in;
	ch3_trim = ch3_in;

	// Warn if we are out of range
	// ---------------------------
	if((ch1_trim < 1000) || (ch2_trim < 1000)){
		Serial.println("Radio Trim error");
	}
	
	// constrain out of range values
	// -----------------------------
	ch1_trim = constrain(ch1_trim, 950, 2050);
	ch2_trim = constrain(ch2_trim, 950, 2050);
	ch3_trim = constrain(ch3_trim, 950, 2050);

	// Store some trim of the throttle in µs/8 for the timers
	// to make the internal math simpler
	// ---------------------------------
	setup_throttle_trims();
	
	// Detect that we don't have the throttle to pin 13 connected 
	// ----------------------------------------------------------
	if(ch3_in < 500){
		Serial.print("No Thottle: ");
		Serial.println(ch3_in,DEC);
	}
}

void setup_throttle_trims()
{
	// only called from the ground
	#if REVERSE_THROTTLE == 0
		// auto save FS
		ch3_fs	 		= ch3_trim - 50;
		#if THROTTLE_IN == 1
			ch3_timer_trim	= (float)(ch3_trim - 1000) * .125;
		#else
			ch3_timer_trim = (float)(CH3_TRIM - 1000) * .125;
		#endif
	#else
		ch3_trim = 2000 - ch3_trim;
		ch3_timer_trim = (float)(ch3_trim - 1000) * .125;
	#endif
	
	ch3_timer_trim = constrain(ch3_timer_trim, 0, 125);
}

void fast_init_radio()
{
	#if THROTTLE_IN	== 1
		// enable in change interrupt on PB5 (digital pin 13)
		PCMSK0 = _BV(PCINT5);
	#endif
	
	// enable pin change interrupt on PD2,PD3 (digital pin 2,3)
	PCMSK2 = _BV(PCINT18) | _BV(PCINT19);

	// enable pin change interrupt 2 and 0
	PCICR = _BV(PCIE2);
	
	#if THROTTLE_IN	== 1
		// enable pin change interrupt 2 and 0
		PCICR |= _BV(PCIE0);
	#endif

	// fix for those without Throttle in while starting from Air
	ch3_timer_trim = (float)(CH3_TRIM - 1000) * .125;// set by init sequence
	ch3_timer_trim = constrain(ch3_timer_trim, 0, 125);
}

void read_radio_limits()
{
	// read the extremes and save the XY sensors - hold at an angle for 3 seonds to save
	// -------------------------------------------------
	Serial.println("Reading radio limits:");
	Serial.println("");
	Serial.println("Move sticks to: upper right and lower Left.");
	Serial.println("");
	#if ENABLE_Z_SENSOR == 0
		Serial.println("Calibrate XY IR sensor now.");
		Serial.println("");
	#endif
	Serial.println("To Continue, hold the stick in the corner for 2 seconds.");

	print_radio();
	// set initial servo limits for calibration routine
	// -------------------------------------------------
	ch1_min = ch1_trim - 150;
	ch1_max = ch1_trim + 150;

	ch2_min = ch2_trim - 150;
	ch2_max = ch2_trim + 150;

	// vars for the radio config routine
	// ---------------------------------
	int counter 	= 0;

	// Allows user to set stick limits and calibrate the IR
	// ----------------------------------------------------
	while(counter < 50){
		delay(40);
		read_XY_analogs();
		read_radio();

		// AutoSet servo limits
		// --------------------
		if (ch1_in > 1000 && ch1_in < 2000){
			ch1_min = min(ch1_in, ch1_min);
			ch1_max = max(ch1_in, ch1_max);
		}
		
		if (ch2_in > 1000 && ch2_in < 2000){
			ch2_min = min(ch2_in, ch2_min);
			ch2_max = max(ch2_in, ch2_max);
		}
		if(ch2_in < (ch2_min + 20) || ch2_in > (ch2_max -20)){
			Serial.print(".");
			counter++;
		}else{
			if (counter > 0)
				counter--;
		}
	}
	
	// contstrain min values
	// ---------------------
	ch1_min = constrain(ch1_min, 1000, 2000);
	ch1_max = constrain(ch1_max, 1000, 2000);
	ch2_min = constrain(ch2_min, 1000, 2000);
	ch2_max = constrain(ch2_max, 1000, 2000);
	
	Serial.println(" ");
}


/*
Control Surfaces Worksheet:

Elevator: ch2
down  = positive

	---/	1765		+100
		
	------  1500		
	
	----\	1041		-100


Rudder: ch1
Right turn = positive

   1220  1632  2043
	/     |     \
   /      |      \ 
	

*/


