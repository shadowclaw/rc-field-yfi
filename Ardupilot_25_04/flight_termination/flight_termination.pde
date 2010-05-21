#include <Servo.h>

Servo rudder_out;
Servo elevator_out;
Servo throttle_out;

long rudder_value = 1000;
long elevator_value = 1000;
long throttle_value = 1000;
long radio_signal_value = LOW;

int rudder_in = 2;
int elevator_in = 3;
int throttle_in = 4;

int radio_signal = 5;

void setup()
{
  pinMode(rudder_in,INPUT);
  pinMode(elevator_in,INPUT);
  pinMode(throttle_in,INPUT);
  pinMode(radio_signal,INPUT);
  
  rudder_out.attach(7);
  elevator_out.attach(8);
  throttle_out.attach(9);  
  
  Serial.begin(57600);
}

void loop()
{
  rudder_value = pulseIn(rudder_in,HIGH);
  elevator_value = pulseIn(elevator_in,HIGH);
  throttle_value = pulseIn(throttle_in,HIGH);
  
  radio_signal_value = digitalRead(radio_signal);
  
  
  if(radio_signal_value)
  {
   rudder_out.writeMicroseconds(rudder_value);
   elevator_out.writeMicroseconds(elevator_value); 
   throttle_out.writeMicroseconds(throttle_value);
  }
  
  else
  {
    rudder_out.write(0); //Need to change
    elevator_out.write(180);
    throttle_out.write(0);
  }
}
