/*****************************************
 * Like Average.. 
 * analog0 = roll infrared.
 * analog1 = pitch infrared. 
 * analog2 = z sensor.
 * analog3 = pressure sensor. 
 *****************************************/
 //Activating the ADC interrupts. 
void Analog_Init(void)
{
 ADCSRA|=(1<<ADIE)|(1<<ADEN);
 ADCSRA|= (1<<ADSC);
}
//
int Anal_Read(uint8_t pin)
{
  return analog_buffer[pin];
}
//
void Analog_Reference(uint8_t mode)
{
	analog_reference = mode;
}
//ADC interrupt vector, this piece of
//is executed everytime a convertion is done. 
ISR(ADC_vect)
{
  ADC_flag=0x01;
}
 
/***********************************/ 
void catch_analogs(void)
{ 
  if(ADC_flag==1)
  {
   uint8_t low, high;
   ADC_flag=0; //Restarting flag
   low = ADCL;
   high = ADCH;
   //analog_buffer[MuxSel]=(high << 8) | low;
   switch(MuxSel)
   {
    case 0: analog0=((high << 8)) | low; break; 
    case 1: analog1=((high << 8)) | low; break; 
    case 2: analog2=(((high << 8) | low)*.99)+((float)analog2*.01); sensor_z(); break;
    case 3: analog3=(((high << 8) | low)*.95)+((float)analog3*.05); break;
    case 4: break;
    case 5: analog5=(((high << 8) | low)*.99)+((float)analog5*.01); Batt_Volt=Battery_Voltage(analog5); break; 
   }
   
   MuxSel++;
   if(MuxSel >=6) MuxSel=0;
   ADMUX = (analog_reference << 6) | (MuxSel & 0x07);
   ADCSRA|= (1<<ADSC);// start the conversion
  //analog3= (float)((float)analog_buffer[3]*.90) + ((float)analog_buffer[3]*.10);
  }
}
/**/
void zero_airspeed(void)
{
   air_speed_offset=analog3;//air_speed_bias_f;
}
/**/
 int airSpeed(void)
{
 //return constrain(Pressure_MTS((float)(analog3-air_speed_offset))-2,0,500); //COnverting to m/s = sqrt(((((5000mV/1023adc)*(analog3-air_speed_bias))/1 Volt/kP)*2)/1.225 Newtons/m*m)
 return ((int)analog3-air_speed_offset);
}

/*****************************************
 * COnverts the infrared values to degrees, is not very well tuned but it works
 * Is limited to 60 degrees, we dont need more, if the aircraft exceeds 60 degrees it 
 * will try to came back anyway... 
 *****************************************/
 
 int y_axis(void)
 {
   return (int)(((analog0-511)*90)/max_ir);
 }
 
  int x_axis(void)
 {
   return (int)(((analog1-511)*90)/max_ir);
 }
 
int get_roll(void)
{
 
  #if REVERSE_X_SENSOR ==1
  return constrain((x_axis()+y_axis())/2,-60,60); 
  #endif 
  #if REVERSE_X_SENSOR ==0
  return constrain((-x_axis()-y_axis())/2,-60,60);
  #endif  
  //return constrain(y_axis(),-60,60); 
}

int get_pitch(void)
{
  #if REVERSE_X_SENSOR ==1
  return constrain((-x_axis()+y_axis())/2,-60,60);
  #endif
  #if REVERSE_X_SENSOR ==0
  return constrain((+x_axis()-y_axis())/2,-60,60);
  #endif
 //return -constrain(x_axis(),-60,60);   
}

void sensor_z(void)
{
       if(abs(get_roll())<=10)//Checks if the roll is less than 10 degrees to read z sensor
      {
        max_ir=abs(511-analog2);
      } 
}
