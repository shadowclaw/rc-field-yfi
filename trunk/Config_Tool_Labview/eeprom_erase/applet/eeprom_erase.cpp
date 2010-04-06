#include <EEPROM.h>

#include "WProgram.h"
void setup();
void loop();
void setup()
{
  pinMode(13,OUTPUT);
  digitalWrite(13,HIGH);
  
  for(int i=0;i<512;i++)
  {
    EEPROM.write(i,0);
  }
  
  digitalWrite(13,LOW);
}

void loop()
{
  
}

int main(void)
{
	init();

	setup();
    
	for (;;)
		loop();
        
	return 0;
}

