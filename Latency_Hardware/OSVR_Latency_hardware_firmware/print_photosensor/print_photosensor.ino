// OSRVR print photosensor firmware source code.
// Author: Russell Taylor working for Sensics.com through Reliasolve.com
// LICENSE: Apache 2.0

// This prints the analog input from the photosensor at a regular interval.

// This code uses information from the AnalogReadSerial example that
// came with the Arduino.

const int LED_PIN = 13;

//*****************************************************
void setup() 
//*****************************************************
{
  Serial.begin(9600);

  // initialize digital pin 13 as an output.
  pinMode(LED_PIN, OUTPUT);  
 }

//*****************************************************
void loop() 
//*****************************************************
{
  Serial.print(analogRead(A0));
  Serial.print("\n");
  delay(500);  
}//end loop

