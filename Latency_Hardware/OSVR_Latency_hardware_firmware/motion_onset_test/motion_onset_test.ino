// OSRVR latency-testing hardware firmware source code.
// Code for "motion onset" test.
// Author: Russell Taylor working for Sensics.com through ReliaSolve.com
// LICENSE: Apache License 2.0.

// This code is based on the "Accelerometer_Gyro_Shield_test" example from
// Robogaia industries that ships along with their 6-axis Gyro/Accelerometer
// shield.  That source is available from Robogaia.com.

// This code also uses information from the Blink example modified by Scott
// Fitzgerald that came with the Arduino.
// This code uses information from the AnalogReadSerial example that
// came with the Arduino.

//some other examples for MPU6000 code
//http://arduino.cc/en/Tutorial/BarometricPressureSensor
//http://code.google.com/p/ardu-imu/downloads/detail?name=ArduIMU_1.9.zip&can=2&q=

/*
 Circuit:
 MPU6000 sensor attached to the SPI communications circuit using pins  10 - 13:
 NOTE: Pin 13 is also the pin that controls the on-board LED, so it cannot be
       used for LED indication when SPI communication is happening.
 CSB: pin 10
 MOSI: pin 11
 MISO: pin 12
 SCK: pin 13
 */

// the sensor communicates using SPI, so include the library:
#include <SPI.h>

//MPU 6000 register addresses
const int ACCEL_XOUT_H = 0x3B;    //59  R ACCEL_XOUT[15:8]
const int ACCEL_XOUT_L = 0x3C;    //60  R ACCEL_XOUT[7:0]
const int ACCEL_YOUT_H = 0x3D;    //61  R ACCEL_YOUT[15:8]
const int ACCEL_YOUT_L = 0x3E;    //62  R ACCEL_YOUT[7:0]
const int ACCEL_ZOUT_H = 0x3F;    //63  R ACCEL_ZOUT[15:8]
const int ACCEL_ZOUT_L = 0x40;    //64  R ACCEL_ZOUT[7:0]
const int TEMP_OUT_H   = 0x41;    //65  R TEMP_OUT[15:8]
const int TEMP_OUT_L   = 0x42;    //66  R TEMP_OUT[7:0]
const int GYRO_XOUT_H  = 0x43;    //67  R GYRO_XOUT[15:8]
const int GYRO_XOUT_L  = 0x44;    //68  R GYRO_XOUT[7:0]
const int GYRO_YOUT_H  = 0x45;    //69  R GYRO_YOUT[15:8]
const int GYRO_YOUT_L  = 0x46;    //70  R GYRO_YOUT[7:0]
const int GYRO_ZOUT_H  = 0x47;    //71  R GYRO_ZOUT[15:8]
const int GYRO_ZOUT_L  = 0x48;    //72  RGYRO_ZOUT[7:0]

const int READ_FLAG =0x80;   //128 has to be added to the address register

// MPU 6000 registers
#define MPUREG_WHOAMI 0x75 //
#define	MPUREG_SMPLRT_DIV 0x19 //
#define MPUREG_CONFIG 0x1A //
#define MPUREG_GYRO_CONFIG 0x1B
#define MPUREG_ACCEL_CONFIG 0x1C
#define MPUREG_INT_PIN_CFG 0x37
#define	MPUREG_INT_ENABLE 0x38 
#define MPUREG_ACCEL_XOUT_H 0x3B //
#define MPUREG_ACCEL_XOUT_L 0x3C //
#define MPUREG_ACCEL_YOUT_H 0x3D //
#define MPUREG_ACCEL_YOUT_L 0x3E //
#define MPUREG_ACCEL_ZOUT_H 0x3F //
#define MPUREG_ACCEL_ZOUT_L 0x40 //
#define MPUREG_TEMP_OUT_H 0x41//
#define MPUREG_TEMP_OUT_L 0x42//
#define MPUREG_GYRO_XOUT_H 0x43 // 
#define	MPUREG_GYRO_XOUT_L 0x44 //
#define MPUREG_GYRO_YOUT_H 0x45 //
#define	MPUREG_GYRO_YOUT_L 0x46 //
#define MPUREG_GYRO_ZOUT_H 0x47 //
#define	MPUREG_GYRO_ZOUT_L 0x48 //
#define MPUREG_USER_CTRL 0x6A //
#define	MPUREG_PWR_MGMT_1 0x6B //
#define	MPUREG_PWR_MGMT_2 0x6C //

// Configuration bits  MPU 6000
#define BIT_SLEEP 0x40
#define BIT_H_RESET 0x80
#define BITS_CLKSEL 0x07
#define MPU_CLK_SEL_PLLGYROX 0x01
#define MPU_CLK_SEL_PLLGYROZ 0x03
#define MPU_EXT_SYNC_GYROX 0x02
#define BITS_FS_250DPS              0x00
#define BITS_FS_500DPS              0x08
#define BITS_FS_1000DPS             0x10
#define BITS_FS_2000DPS             0x18
#define BITS_FS_2G                  0x00
#define BITS_FS_4G                  0x08
#define BITS_FS_8G                  0x10
#define BITS_FS_16G                 0x18
#define BITS_FS_MASK                0x18
#define BITS_DLPF_CFG_256HZ_NOLPF2  0x00
#define BITS_DLPF_CFG_188HZ         0x01
#define BITS_DLPF_CFG_98HZ          0x02
#define BITS_DLPF_CFG_42HZ          0x03
#define BITS_DLPF_CFG_20HZ          0x04
#define BITS_DLPF_CFG_10HZ          0x05
#define BITS_DLPF_CFG_5HZ           0x06
#define BITS_DLPF_CFG_2100HZ_NOLPF  0x07
#define BITS_DLPF_CFG_MASK          0x07
#define	BIT_INT_ANYRD_2CLEAR	    0x10
#define	BIT_RAW_RDY_EN		    0x01
#define	BIT_I2C_IF_DIS              0x10

#undef VERBOSE
#undef VERBOSE2

// pins used for the connection with the sensor
// the other you need are controlled by the SPI library):
const int chipSelectPin = 10;

// Thresholds
const int GYRO_THRESHOLD = 300;
const int ACCEL_CHANGE_THRESHOLD = 500;
const int BRIGHTNESS_CHANGE_THRESHOLD = 3;
const unsigned long TIMEOUT_USEC = 1000000L;

// Keeps track of delays so we can do an average.
const int NUM_DELAYS = 16;
unsigned long delays[NUM_DELAYS];
static int count = 0, odd_count = 0, even_count = 0;        

//*****************************************************
void setup() 
//*****************************************************
{
  Serial.begin(9600);
  Serial.print("OSVR_latency_hardware_firmware onset test v03.00.01\n");
  Serial.print(" Mount the photosensor rigidly on the screen.\n");
  Serial.print(" Move the inertial sensor along with the tracking hardware.\n");
  Serial.print(" Make the app change the brightness in front of the photosensor.\n");
  Serial.print(" Latencies reported in microseconds, 1-second timeout\n");
  
  MPU6000_Init();
  delay(100);
}

//*****************************************************
void loop() 
//*****************************************************
{
  // We run a finite-state machine that cycles through cases of waiting for
  // a period of non motion, detecting a sudden motion, waiting until there
  // is a change in brightness, and reporting the time passed in microseconds
  // between the motion and the brightness change.
  static enum { S_CALM, S_MOTION, S_BRIGHTNESS } state = S_CALM;
  static unsigned long start;
  static int initial_brightness;
  
  switch (state) {
    case S_CALM:
      {
        // Wait for a period of at least 100 cycles where there is no motion above
        // the motion threshold.
        static int calm_cycles = 0;
        if (moving2()) {
          calm_cycles = 0;
        } else if (++calm_cycles >= 50) {
            state = S_MOTION;
            calm_cycles = 0;
        } else {
          // Make sure we stay calm for a while (half a second)
          delay(10);
        }
      }
      break;

    case S_MOTION:
      {
        // Wait for a sudden motion.  When we find it, record the time in microseconds
        // so we can compare it to when the brightness changes.  Also record the brightness
        // so we can look for changes.
        if (moving2()) {
          start = micros();
          initial_brightness = analogRead(A0);
          state = S_BRIGHTNESS;
#ifdef VERBOSE
          Serial.print("Moving\n");
#endif
        }
      }
      break;

    case S_BRIGHTNESS:
      {
        // Wait for a change in brightness compared to the original value that
        // passes a threshold.  When we get it, report the latency.
        // If it takes too long, then we time out and start over.
        int brightness = analogRead(A0);
        unsigned long now = micros();
        
        // Keep track of how many values we got for odd and even rows and
        // compute a running average when we get a full complement for each.
        // Ignore timeout values
        if (abs(brightness - initial_brightness) > BRIGHTNESS_CHANGE_THRESHOLD) {
          // Print the result for this time
          Serial.print(now - start);
          Serial.print("\n");
          if (count % 2 == 0) {
            odd_count++;  // This is the first one (zero indexed) or off by twos
          } else {
            even_count++;
          }
          delays[count++] = now - start;
          state = S_CALM;
        } else if (now - start > TIMEOUT_USEC) {
          Serial.print("Timeout: no brightness change after motion, restarting\n");
          // We don't increment the counter and we set the reading to 0 so we ignore it.
          delays[count++] = 0;
          state = S_CALM;
        }

        // See if it is time to print the average result.          
        if (count == NUM_DELAYS) {
          unsigned long even_average = 0;
          unsigned long odd_average = 0;
          for (int i = 0; i < NUM_DELAYS/2; i++) {
            odd_average += delays[2*i];
            even_average += delays[2*i+1];
          }
          
          odd_average /= odd_count;
          Serial.print("Average of last ");
          Serial.print(NUM_DELAYS/2);
          Serial.print(" odd counts (ignoring timeouts) = ");
          Serial.print(odd_average);
          Serial.print("\n");
          even_average /= even_count;
          Serial.print("Average of last ");
          Serial.print(NUM_DELAYS/2);
          Serial.print(" even counts (ignoring timeouts) = ");
          Serial.print(even_average);
          Serial.print("\n");
          
          count = 0;
          odd_count = 0;
          even_count = 0;
        }
      }
      break;
   
    default:
      Serial.print("Error: Unrecognized state; restarting\n");
      state = S_CALM;
      break;
  }
}


// Determine whether we're undergoing acceleration or rotation.
// Does this by comparing the gyros to a threshold and
// comparing accelerations to see if they are changing.
//*****************************************************
inline bool moving2(void)
//*****************************************************
{
  bool ret = false;
  
  int accX, accY, accZ;
  int gyroX, gyroY, gyroZ;
  accX = getXAccelerometerCounts();
  accY = getYAccelerometerCounts();
  accZ = getZAccelerometerCounts();
  gyroX = getXGyroCounts();
  gyroY = getYGyroCounts();
  gyroZ = getZGyroCounts();
  
#ifdef VERBOSE2
  /* Debugging info to help figure out what the thresholds should be */
  Serial.print(" "); Serial.print(accX);
  Serial.print(" "); Serial.print(accY);
  Serial.print(" "); Serial.print(accZ);
  Serial.print(", "); Serial.print(gyroX);
  Serial.print(" "); Serial.print(gyroY);
  Serial.print(" "); Serial.print(gyroZ);
  Serial.print(", "); Serial.print(analogRead(A0));
  Serial.print("\n");
  delay(1000);
#endif
  
  // If the gyros are above threshold, then we're moving.
  if (gyroX >= GYRO_THRESHOLD) {
#ifdef VERBOSE
    Serial.print("gX\n");
#endif
    ret = true;
  }
  if (gyroY >= GYRO_THRESHOLD) {
#ifdef VERBOSE
    Serial.print("gY\n");
#endif
    ret = true;
  }
  if (gyroZ >= GYRO_THRESHOLD) {
#ifdef VERBOSE
    Serial.print("gZ\n");
#endif
    ret = true;
  }
  
  // See if the accelerations are sufficiently different from the
  // ones we read last time.  If so, we're moving.
  static int prevX = 0;
  static int prevY = 0;
  static int prevZ = 0;
  if ( abs(accX - prevX) >= ACCEL_CHANGE_THRESHOLD ) {
#ifdef VERBOSE
    Serial.print("aX\n");
#endif
    ret = true;
  }
  if ( abs(accY - prevY) >= ACCEL_CHANGE_THRESHOLD ) {
#ifdef VERBOSE
    Serial.print("aY\n");
#endif
    ret = true;
  }
  if ( abs(accZ - prevZ) >= ACCEL_CHANGE_THRESHOLD ) {
#ifdef VERBOSE
    Serial.print("aZ\n");
#endif
    ret = true;
  }
  prevX = accX;
  prevY = accY;
  prevZ = accZ;

  return ret;  
}

//*****************************************************
  // We only read the high-order bytes for two reasons:
  // 1) Speed.
  // 2) To avoid getting a measurement whose upper and lower bytes are
  // inconsisent.
  // XXX However, this routine did not work.  It is not clear why it
  // did not, but switching back to using the original read routines
  // through the moving2() call.
void getAccelerometerAndGyroHighCounts(byte &accX, byte &accY, byte &accZ,
                                       byte &gyroX, byte &gyroY, byte &gyroZ)
//*****************************************************
{
    accX = readRegister(ACCEL_XOUT_H);
    accY = readRegister(ACCEL_YOUT_H);
    accZ = readRegister(ACCEL_ZOUT_H);
    gyroX = readRegister(GYRO_XOUT_H);
    gyroY = readRegister(GYRO_YOUT_H);
    gyroZ = readRegister(GYRO_ZOUT_H);
}

//*****************************************************
int getXAccelerometerCounts(void)
//*****************************************************
{
    int tempData_HI,tempData_LO;
     //Read the  data
    tempData_HI = readRegister(ACCEL_XOUT_H);
    tempData_LO = readRegister(ACCEL_XOUT_L );
    
    return ((tempData_HI << 8) | tempData_LO);
    
}//end func

//*****************************************************  
int getYAccelerometerCounts(void)
//*****************************************************
{
    int tempData_HI,tempData_LO;
    
    tempData_HI = readRegister(ACCEL_YOUT_H);
    tempData_LO = readRegister(ACCEL_YOUT_L );
    
    return ((tempData_HI << 8) + tempData_LO);
}//end func
  
//*****************************************************  
int getZAccelerometerCounts(void)
//*****************************************************
{
    int tempData_HI,tempData_LO;
    
    tempData_HI = readRegister(ACCEL_ZOUT_H );
    tempData_LO = readRegister(ACCEL_ZOUT_L );
    
    return ((tempData_HI << 8) + tempData_LO);
    
}//end func

//*****************************************************
int getXGyroCounts(void)
//*****************************************************
{
     int tempData_HI,tempData_LO;
     
     tempData_HI = readRegister(GYRO_XOUT_H );
     tempData_LO = readRegister(GYRO_XOUT_L );
     
     return ((tempData_HI << 8) + tempData_LO);
}//end func

//*****************************************************
int getYGyroCounts(void)
//*****************************************************
{
     int tempData_HI,tempData_LO;
     
     tempData_HI = readRegister(GYRO_YOUT_H);
     tempData_LO = readRegister(GYRO_YOUT_L );
     
     return ((tempData_HI << 8) + tempData_LO);
}//end func  

//*****************************************************
int getZGyroCounts(void)
//*****************************************************
{
     int tempData_HI,tempData_LO;
     
     tempData_HI = readRegister(GYRO_ZOUT_H);
     tempData_LO = readRegister(GYRO_ZOUT_L);
     
     return ((tempData_HI << 8) + tempData_LO);
}//end func 


// Determine whether we're undergoing acceleration or rotation.
// Does this by comparing the gyros to a threshold and
// comparing accelerations to see if they are changing.
//*****************************************************
inline bool moving(void)
//*****************************************************
{
  byte accX, accY, accZ;
  byte gyroX, gyroY, gyroZ;
  getAccelerometerAndGyroHighCounts(accX, accY, accZ, gyroX, gyroY, gyroZ);
  
  // If the gyros are above threshold, then we're moving.
  if (gyroX >= GYRO_THRESHOLD) { return true; }
  if (gyroY >= GYRO_THRESHOLD) { return true; }
  if (gyroZ >= GYRO_THRESHOLD) { return true; }
  
  // See if the accelerations are sufficiently different from the
  // ones we read last time.  If so, we're moving.
  static int prevX = 0;
  static int prevY = 0;
  static int prevZ = 0;
  bool ret = false;
  if ( abs(accX - prevX) >= ACCEL_CHANGE_THRESHOLD ) {
    ret = true;
  }
  if ( abs(accY - prevY) >= ACCEL_CHANGE_THRESHOLD ) {
    ret = true;
  }
  if ( abs(accZ - prevZ) >= ACCEL_CHANGE_THRESHOLD ) {
    ret = true;
  }
  prevX = accX;
  prevY = accY;
  prevZ = accZ;

  return ret;  
}

//Read from register
//*****************************************************
unsigned int readRegister(byte thisRegister) 
//*****************************************************
{
  unsigned int result = 0;   // result to return
  byte addr = thisRegister + 0x80;
 // Serial.print(thisRegister, BIN);
 // byte dataToSend = thisRegister ;
  //Serial.println(thisRegister, BIN);
  // take the chip select low to select the device:
  digitalWrite(chipSelectPin, LOW);
  // send the device the register you want to read:
  SPI.transfer(addr);
  // send a value of 0 to read the first byte returned:
  result = SPI.transfer(0x00);
  // take the chip select high to de-select:
  digitalWrite(chipSelectPin, HIGH);
  // return the result:
  return(result);
}


//Write to register
//*****************************************************
void writeRegister(byte thisRegister, byte thisValue) 
//*****************************************************
{
  thisRegister = thisRegister ;
  // now combine the register address and the command into one byte:
  byte dataToSend = thisRegister;

  // take the chip select low to select the device:
  digitalWrite(chipSelectPin, LOW);

  SPI.transfer(dataToSend); //Send register location
  SPI.transfer(thisValue);  //Send value to record into register

  // take the chip select high to de-select:
  digitalWrite(chipSelectPin, HIGH);
}//


// MPU6000 Initialization and configuration
//*************************************************
void MPU6000_Init(void)
//*************************************************
{
    // MPU6000 chip select setup
    pinMode(chipSelectPin, OUTPUT);
    digitalWrite(chipSelectPin, HIGH);
    
    // SPI initialization
    SPI.begin();
    SPI.setClockDivider(SPI_CLOCK_DIV16);      // SPI at 1Mhz (on 16Mhz clock)
    delay(10);
    
    // Chip reset
    writeRegister(MPUREG_PWR_MGMT_1, BIT_H_RESET);
    delay(100);

    // Wake up device and select GyroZ clock (better performance)
    writeRegister(MPUREG_PWR_MGMT_1, MPU_CLK_SEL_PLLGYROZ);
    delay(1);

    // Disable I2C bus (recommended on datasheet)
    writeRegister(MPUREG_USER_CTRL, BIT_I2C_IF_DIS);
    delay(1);

    // Set the sampling rate.  We do this by setting the divisor on the rate,
    // which has a base update rate of 1kHz.  The number we use here is added
    // to 1 and then used as a divisor: Rate = 1Khz / (value + 1).  So a value
    // of 0 is 1 kHz, a value of 4 is 200Hz, and a value of 19 is 50Hz.
    // We want to sample as rapidly as possible, so that we have the minimum
    // latency between motion and its detection.  Even at 1kHz, we're going to
    // have some fraction of a second of latency on our reads.
    writeRegister(MPUREG_SMPLRT_DIV,0);
    delay(1);

    // Set the filter pass frequency on the low-pass filter to the maximum
    // (no filter, corresponding to a 2.1Khz cutoff).
    writeRegister(MPUREG_CONFIG, BITS_DLPF_CFG_2100HZ_NOLPF);
    delay(1);
    
    // Set the measurement scale on the gyros and the accelerometers.
    // From the data sheet: For precision tracking of both fast and slow
    // motions, the parts feature a user-programmable gyroscope full-scale range
    // of ±250, ±500, ±1000, and ±2000°/sec (dps) and a user-programmable accelerometer
    // full-scale range of ±2g, ±4g, ±8g, and ±16g.
    // We set the sensitivity to match the example app, which got good full-scale results.
    writeRegister(MPUREG_GYRO_CONFIG,BITS_FS_2000DPS);
    delay(1);
    writeRegister(MPUREG_ACCEL_CONFIG,BITS_FS_4G);
    delay(1);   

    // Oscillator set
    writeRegister(MPUREG_PWR_MGMT_1,MPU_CLK_SEL_PLLGYROZ);
    delay(1);
}

