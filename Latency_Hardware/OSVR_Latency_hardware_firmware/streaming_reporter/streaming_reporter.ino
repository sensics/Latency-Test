// OSRVR print photosensor firmware source code.
// Copyright 2015
// Author: Russell Taylor working for Sensics.com through Reliasolve.com
// LICENSE: Apache 2.0

// This program is meant to be paired with the VRPN.org device
// vrpn_Sensics_streaming_arduino device, which will send the appropriate
// commands to control it and read back its results.

// It prints the analog input from a set of analogs as quickly as possible,
// with the number of analogs read from the serial line at program start.
// It also lets the program request the insertion of additional markers,
// which it does by sending the marker number on the serial stream.

// INPUT: An initial ASCII number followed by a carriage return indicating
// how many analogs to be read.  The number must be between 1 and 8.
//   Optional: numeric marker commands, each followed by a carriage return
// indicating a host-side event to be correlated with the analog data.  These
// are inserted into the output stream and returned.

// OUTPUT: Streaming lines.  Each line consists of:
//   (1) A list of comma-separated ASCII values starting with Analog0
// and continuing up to the number of analogs requests (Analog1, Analog2,
// ... Analog[N-1].
//   (2) An optional list of numeric event markers that were sent across
// the serial line that were received since the last report.

// Initialize to an invalid value
int numAnalogs = 0;

//*****************************************************
void setup() 
//*****************************************************
{
  Serial.begin(115200);
}

//*****************************************************
void readAndParseInput()
//*****************************************************
{
  // Nothing to do if no available characters.
  if (Serial.available() == 0) {
    return;
  }
  
  // If we don't have the number of analogs specified validly,
  // set it.
  if (numAnalogs <= 0) {
    numAnalogs = Serial.parseInt();
  }
  
  // We already have our analogs specified, so this is
  // a marker request.  Add it to the array of outstanding
  // requests.
  // XXX
}

//*****************************************************
void loop() 
//*****************************************************
{
  readAndParseInput();
  if (numAnalogs > 0) {
    Serial.print(analogRead(0));
    for (int i = 1; i < numAnalogs; i++) {
      Serial.print(",");
      Serial.print(analogRead(i));
    }
    Serial.print("\n");
  }
}//end loop

