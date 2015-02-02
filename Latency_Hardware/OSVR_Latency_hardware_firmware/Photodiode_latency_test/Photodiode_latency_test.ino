// OSRVR latency-testing hardware firmware source code.
// Author: Russell Taylor working for Sensics.com through Reliasolve.com
// LICENSE: XXX

// This is a separate program that uses the on-board LED combined with
// the analog input to check how long it takes the Arduino to detect
// a change in brightness.  This turns the LED on and off and waits until
// it detects a brightness change, so is a conservative estimate of the
// latency (the latency may be less, but it cannot be more).

// This code uses information from the Blink example modified by Scott
// Fitzgerald that came with the Arduino.
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
  // The first time through the loop, we do a calibration to find out
  // what the threshold for turning the LED on and off should be.  We
  // turn it off, wait for it to go off, and then read it.  Then back
  // on and wait, then back off.
  static bool calibrated = false;
  static int threshold = -1;
  if (!calibrated) {
    digitalWrite(LED_PIN, LOW);
    delay(400);
    int dark_value = analogRead(A0);

    digitalWrite(LED_PIN, HIGH);
    delay(400);
    int bright_value = analogRead(A0);
    
    digitalWrite(LED_PIN, LOW);
    delay(400);
    
    if (bright_value - dark_value < 100) {
      Serial.print("Not enough brightness difference, reposition photosensor to see LED\n");
    } else {
      threshold = (dark_value + bright_value) / 2;
      Serial.print("Calibrated: threshold = ");
      Serial.print(threshold);
      Serial.print(" (dark = ");
      Serial.print(dark_value);
      Serial.print(", bright = ");
      Serial.print(bright_value);
      Serial.print(")\n");
      delay(400);
      calibrated = true;
    }
  }
  
  // Once we have calibrated, measure how long
  // it takes to do an analog read and go around
  // the loop.  This lets us know how much of the
  // latency in the photodiode measurement is due
  // to housekeeping.
  static bool loop_delay_measured = false;
  if (calibrated && !loop_delay_measured) {
    static bool first_time = true;
    static unsigned long start;
    if (first_time) {
      start = micros();
      int unused = analogRead(A0);
      first_time = false;
    } else {
      unsigned long now = micros();
      unsigned long latency = now - start;
      Serial.print("Loop delay (microseconds) = ");
      Serial.print(latency);
      Serial.print("\n\n");
      delay(400);
      loop_delay_measured = true;
    }
  }
  
  // We run a finite-state machine to test the latency between when the LED is
  // turned on and when the photosensor changes its value to pass the threshold.
  // We test both the rising and falling brightness condition.
  if (loop_delay_measured) {
    static enum { S_SET_ON, S_MEASURE_ON, S_SET_OFF, S_MEASURE_OFF }
      state = S_SET_ON;
    
    // micros() will wrap around after about 7 hours of running.
    // It has an accuracy of 4 microseconds.        
    static unsigned long start = 0;
    
    switch (state) {
      case S_SET_ON:
        // Turn on the LED and record when we did.
        start = micros();
        digitalWrite(LED_PIN, HIGH);
        state = S_MEASURE_ON;
        break;
      
      case S_MEASURE_ON:
        // Wait until the LED passes threshold, then report how long it
        // took and wait a bit for the printing to happen.
        if (analogRead(A0) > threshold) {
          unsigned long now = micros();
          unsigned long latency = now - start;
          Serial.print("On delay (microseconds) = ");
          Serial.print(latency);
          Serial.print("\n");
          delay(500);
          state = S_SET_OFF;
        }
        break;
        
      case S_SET_OFF:
        // Turn off the LED and record when we did.
        digitalWrite(LED_PIN, LOW);
        start = micros();
        state = S_MEASURE_OFF;
        break;
      
      case S_MEASURE_OFF:
        // Wait until the LED passes threshold, then report how long it
        // took and wait it bit for the printing to happen.
        if (analogRead(A0) < threshold) {
          unsigned long now = micros();
          unsigned long latency = now - start;
          Serial.print("Off delay (microseconds) = ");
          Serial.print(latency);
          Serial.print("\n");
          delay(500);
          state = S_SET_ON;
        }
        break;
    }
  }
  
}//end loop

