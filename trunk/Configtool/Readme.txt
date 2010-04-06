Please note, this project is part of Remzibi's OSD. There are some drivers and controls that are required to run the GPS Emulator.
You must download and install the Remzibi OSD Configuration software first. http://remzibi.happykillmore.com

Changes that must be made to the ArduPilot Source to make the GPS Emulator work:

easystar.h
----------
//1-7
#define GPS_PROTOCOL 0

//4-1
#define ALTITUDE_ERROR_MAX 10 //

//4-4
#define ALTITUDE_ERROR_PITCH_MAX 14 //Limits, EasyStar climb by itself, you don't need to up the elevator (you may stall)... 


ArduPilot_Easy_Star_V23
-----------------------
void loop()//Main Loop

**** Change print_data(); to print_data_emulator();


GPS_NMEA.pde Tab
----------------
void fast_init_gps(void)
{
  pinMode(12, OUTPUT);//Status led
  Serial.begin(9600); //Universal Sincronus Asyncronus Receiveing Transmiting 
  //Serial.begin(38400);
}


//Suggested changes below (to help with 5Hz buffer overruns)
if(gps_buffer[0]=='$')//Verify if is the preamble $
{
   counter = 0; //Add this
   checksum = 0; //Add this
   unlock=1; 
}


else
{
   counter++; //Incrementing counter
   if (counter >= 200) //Add this
   { //Add this
     Serial.flush(); //Add this
     counter = 0;  //Add this
     checksum = 0; //Add this
     unlock = 0;
   }
}



System.pde Tab (Add the following)
----------------------------------
void print_data_emulator(void)
{
  static unsigned long timer1=0;
  static byte counter;
  
  if(millis()-timer1 > ATTITUDE_RATE_OUTPUT)
  {   
    digitalWrite(13,HIGH);
    Serial.print("!!!");
    Serial.print ("STT:");
    Serial.print((int)Tx_Switch_Status());
    Serial.print (",WPN:");
    Serial.print((int)last_waypoint);//Actually is the waypoint.
    Serial.print (",DST:");
    Serial.print(wp_distance);
    Serial.print (",RER:");
    Serial.print((int)roll_set_point);
    Serial.print (",PSET:");
    Serial.print(pitch_set_point);
    Serial.println(",***");
    timer1=millis(); 
    digitalWrite(13,LOW);
  }
}




