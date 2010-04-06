//Test code for the inflight waypoint updating
//Chris Kirby

//checksum does not include the CHK message

byte buffer[6];
byte packet[164];
byte packet_index=0;

unsigned int adler_a=0;
unsigned int adler_b=0;

char rts[] = "XXXRTS";
char wpt_msg[] = "XXX";
char wpt_end[] = "END";
char wpt_top[] = "TOP";
char check[] = "CHK";

byte flag=0;
byte list_flag=0;
byte number_waypoints=0;

void setup()
{
  Serial.begin(115200);
}

byte packet_case=0;
//values of packet_case
//0 = wait for a RTS and send and ACK
//1 = receive packet / wait for CHK then calc checksum
//2 = receive the checksum / compare and reply with GWL or BCR
//3 = parse the packet into the EEPROM lists
void loop()
{
  switch (packet_case)
  {
    case 0:
      while (Serial.available()>5)
     {
         for(byte i=0; i<6; i++)
         {
             buffer[i]=Serial.read();
         }
         flag=0;
         for(byte i=0;i<6;i++)
         {
            flag += (rts[i]==buffer[i]);
         }
         
         if (flag==6)
         {
           Serial.print("XXXACK");
           packet_index=0;
           packet_case=1;
         }
     }
      break;
    
    case 1:
     while (Serial.available()>0)
     {
       packet[packet_index]=Serial.read();
       buffer[0]=buffer[1];
       buffer[1]=buffer[2];
       buffer[2]=packet[packet_index];
       
       flag=0;
       for(byte i=0;i<3;i++)
         {
            flag += (check[i]==buffer[i]);
         }
       
       if (flag==3)
       {
         calc_checksum();
         packet_case=2;
         packet_index++;
         flag=0;
         break;
       }
         
       packet_index++;
       
     }
      break;
      
    case 2:
      if (Serial.available())
      {
        packet[packet_index]=Serial.read();
        flag++;
        packet_index++;
      }
      
      if (flag==4)
      {
        if ((highByte(adler_b)==packet[packet_index-4])&(lowByte(adler_b)==packet[packet_index-3])&
            (highByte(adler_a)==packet[packet_index-2])&(lowByte(adler_a)==packet[packet_index-1]))
        {
           Serial.print("XXXGWL");
           packet_case=3;
        }
        
        else
        {
          Serial.print("XXXBCR");
          packet_case=1;
          packet_index=0;
        }
      }
        
      
      break;
    
    case 3:
      
      /*Serial.println();
      Serial.print("THE PACKET:");
      for (byte i=0; i<packet_index;i++)
      {
        Serial.print(packet[i]);
      }
      Serial.println();*/
      
      //Parse list change flag 
      flag=0;
      for(byte i=0;i<3;i++)
      {
        flag += (wpt_end[i]==packet[i+3]);
      }
      if (flag==3) list_flag=1;
      else
      {
        flag=0;
        for(byte i=0;i<3;i++)
        {
          flag += (wpt_top[i]==packet[i+3]);
        }
        if (flag ==3) list_flag=2;
        else list_flag=0;
      }
      
      //Parse number of waypoints in list
      number_waypoints=packet[7];
      
      //parse waypoints
      long latitude=0;
      long longitude=0;
      int altitude=0;
      
      for(int i=0 ; i<number_waypoints;i++)
      {
        int j=i*10;
        latitude=(long(packet[j+7])<<24)+(long(packet[j+8])<<16)+(long(packet[j+9])<<8)+long(packet[j+10]);
        longitude=(long(packet[j+11])<<24)+(long(packet[j+12])<<16)+(long(packet[j+13])<<8)+long(packet[j+14]);
        altitude=(int(packet[j+15])<<8)+(packet[j+16]);
        Serial.print("1:");
        Serial.println((packet[j+11]),HEX);
        Serial.print("2:");
        Serial.println((packet[j+12]),HEX);
        Serial.print("3:");
        Serial.println((packet[j+13]),HEX);
        Serial.print("4:");
        Serial.println((packet[j+14]),HEX);
             
        
        
        Serial.print("LATITUDE:");
        Serial.println(latitude,DEC);
        Serial.print("LONGITUDE:");
        Serial.println(longitude,DEC);
        Serial.print("ALTITUDE:");
        Serial.println(altitude,DEC);
        //write to EEPROM
      }
      
      packet_case=0;
      break;
  } 
  
  
}

void calc_checksum()
{
  adler_a=1;
  adler_b=0;
  for(byte i=0;i<=(packet_index-3);i++)
  {
    adler_a=(adler_a+packet[i])%65521;
    adler_b=(adler_b+adler_a)%65521;
  }  
}
