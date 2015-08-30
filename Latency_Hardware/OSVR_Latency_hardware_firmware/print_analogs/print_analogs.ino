// OSRVR print photosensor firmware source code.
// Copyright 2015
// Author: Russell Taylor working for Sensics.com through Reliasolve.com
// LICENSE: Apache 2.0

// This prints the analog input from a set of analogs at a regular interval.

// This code uses information from the AnalogReadSerial example that
// came with the Arduino.

const int LED_PIN = 13;
const int numAnalogs = 4;

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
  Serial.print(analogRead(0));
  for (int i = 1; i < numAnalogs; i++) {
    Serial.print(", ");
    Serial.print(analogRead(i));
  }
  Serial.print("\n");
  delay(100);  
}//end loop

