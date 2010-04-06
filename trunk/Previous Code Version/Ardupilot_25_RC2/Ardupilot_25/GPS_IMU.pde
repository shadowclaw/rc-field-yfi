#if GPS_PROTOCOL == 3

byte ck_a = 0;
byte ck_b = 0;

/****************************************************************
 * Here you have all the stuff for data reception from the IMU_GPS
 ****************************************************************/

/*	GPS_update bit flags -
	 - 0x01 bit = gps lat/lon data received
	 - 0x02 bit = gps alt and speed data received
	 - 0x04 bit = IMU data received
	 - 0x08 bit = PROBLEM - No IMU data last second!

#define GPS_NONE 0
#define GPS_POSITION 1
#define GPS_HEADING 2
#define GPS_BOTH 3
#define GPS_IMU 4
#define GPS_IMU_ERROR 8

*/

void init_gps(void)
{
	pinMode(12, OUTPUT);//Status led
	Serial.begin(THIRTY_EIGHT_K_BAUD); //Universal Sincronus Asyncronus Receiveing Transmiting 
	wait_for_GPS_fix();
}

void fast_init_gps(void)
{
	pinMode(12, OUTPUT);//Status led
	Serial.begin(THIRTY_EIGHT_K_BAUD); //Universal Sincronus Asyncronus Receiveing Transmiting 
}

/****************************************************************
 ****************************************************************/
void wait_for_GPS_fix(void)//Wait GPS fix...
{
	Serial.println(" ");
	Serial.println("Wait for GPS");
	do {
		decode_gps();
		digitalWrite(12,LOW);
		delay(25);
		digitalWrite(12,HIGH);
		delay(25);
	} while(GPS_fix != 0x07);//
	//} while(GPS_fix == BAD_GPS);

	digitalWrite(12,HIGH);

}

/****************************************************************
 * 
 ****************************************************************/
	
void decode_gps(void)
{
	static unsigned long IMU_timer=0; //used to set PROBLEM flag if no data is received. 
	static byte IMU_step = 0;
	boolean success = 0;
        int numc = 0;
        byte data;

/*
IMU Message format
Byte(s)		 Value
0-3		 Header "DIYd"
4                Payload length  (either 6 or 18 depending on if new gps data is available
5                Message ID = 2
6,7		 roll				Integer (degrees*100)
8,9		 pitch				Integer (degrees*100)
10,11		 yaw				Integer (degrees*100)
//  The following are not included in all messages
12-15		longitude			Integer (value*10**7)
16-19		latitude			Integer (value*10**7)
20,21		altitude			Integer (meters*10)
22,23		gps speed			Integer (M/S*100)

12,13 or 24,25  checksum
*/


	numc = Serial.available();
	if (numc > 0)
		for (int i=0;i<numc;i++)	// Process bytes received
		{
			data = Serial.read();
			switch(GPS_step)		 //Normally we start from zero. This is a state machine
			{
			case 0:	
				if(data == 0x44) 
					IMU_step++; //First byte of data packet header is correct, so jump to the next step
				break; 
			case 1:	
				if(data == 0x49)
					 IMU_step++;	//Second byte of data packet header is correct
				else 
					IMU_step=0;     //Second byte is not correct so restart to step zero and try again.		 
				break;

			case 2:	
				if(data == 0x59)
					 IMU_step++;	//Third byte of data packet header is correct
				else 
					IMU_step=0;     //Third byte is not correct so restart to step zero and try again.		 
				break;

			case 3:	
				if(data == 0x49)   //******** Need to get right hex code for header
					 IMU_step++;	//Fourth byte of data packet header is correct, Header complete
				else 
					IMU_step=0;     //Fourth byte is not correct so restart to step zero and try again.		 
				break;

			case 4:	
				payload_length = data;
                                checksum(payload_length);
			        IMU_step++;		 
				break;

			case 5:	
				if(data == 0x02) {  //Verify the message ID is correct
					 checksum(data);
                                         IMU_step++;	
                                }
				else 
					IMU_step=0;     //Whoa!  This is not the right message so restart to step zero and try again.		 
				break;
		 
			case 6:				 // Payload data read...
				if (payload_counter < payload_length){	// We stay in this state until we reach the payload_length
					IMU_buffer[payload_counter] = data;
					checksum(data);
					payload_counter++;
				}else{
					IMU_step++; 
				}
				break;
			case 7:
				IMU_ck_a=data;	 // First checksum byte
				GPS_step++;
				break;
			case 8:
				IMU_ck_b=data;	 // Second checksum byte
			 
		// We end the GPS read...
				if((ck_a=IMU_ck_a)&&(ck_b=IMU_ck_a)) {	 // Verify the received checksum with the generated checksum.. 
					IMU_join_data();
	                                success = 1;
				} else {
					success = 0;		//bad checksum
				} 						 
				// Variable initialization
				IMU_step = 0;
				payload_counter = 0;
				ck_a = 0;
				ck_b = 0;
				IMU_timer = millis(); //Restarting timer...
				break;
		        }
	        }		// End for...
	
	if(millis() - IMU_timer > 500){
		digitalWrite(12, LOW);	//If we don't receive any byte in a half second turn off gps fix LED... 
		GPS_fix = BAD_GPS;
                GPS_update = GPS_IMU_ERROR;
	}
}
  
 /****************************************************************
 * 
 ****************************************************************/
void IMU_join_data()
{
	int j=0;

	 //Storing IMU roll
	intUnion.byte[0] = IMU_buffer[j++];
	intUnion.byte[1] = IMU_buffer[j++];
	roll_sensor = intUnion.word;

	 //Storing IMU pitch
	intUnion.byte[0] = IMU_buffer[j++];
	intUnion.byte[1] = IMU_buffer[j++];
	pitch_sensor = intUnion.word;

	 //Storing IMU heading (yaw)
	intUnion.byte[0] = IMU_buffer[j++];
	intUnion.byte[1] = IMU_buffer[j++];
	ground_course = intUnion.word;

        if (payload_length = 6)
            GPS_update = GPS_IMU;
        else {				
            GPS_update = GPS_IMU || GPS_BOTH;
            GPS_fix = VALID_GPS;	
					 
	    current_loc.lng = join_4_bytes(&IMU_buffer[j]);
	    j += 4;
	    current_loc.lat = join_4_bytes(&IMU_buffer[j]);
	    j += 4;
	    //Storing GPS Height above the sea level
	    intUnion.byte[0] = IMU_buffer[j++];
	    intUnion.byte[1] = IMU_buffer[j++];
	    current_loc.alt = intUnion.word * 10; //  ***** Need to clarify here on the scaling   
	    //Storing Speed (3-D) 
	    intUnion.byte[0] = IMU_buffer[j++];
	    intUnion.byte[1] = IMU_buffer[j++];
	    ground_speed = (float)intUnion.word;

	    digitalWrite(12,HIGH);
}
						

/****************************************************************
 * 
 ****************************************************************/
void checksum(byte data)	 // 
{
	ck_a+=data;
	ck_b+=ck_a; 
}
/****************************************************************
 ****************************************************************/
void wait_for_data(byte many)
{
	while(Serial.available() <= many); 
}

 // Join 4 bytes into a long
 // -------------------------
int32_t join_4_bytes(byte Buffer[])
{
	longUnion.byte[0] = *Buffer;
	longUnion.byte[1] = *(Buffer+1);
	longUnion.byte[2] = *(Buffer+2);
	longUnion.byte[3] = *(Buffer+3);
	return(longUnion.dword);
}


#endif
