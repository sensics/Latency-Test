// OSRVR latency-testing hardware firmware source code.
// Code for "motion turn-around" test.
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

// Debugging definitions.
#undef VERBOSE
#undef VERBOSE2
#define PRINT_TRACE

// pins used for the connection with the sensor
// the other you need are controlled by the SPI library):
const int chipSelectPin = 10;

// Thresholds
const int ACCEL_CHANGE_THRESHOLD = 500;
const int GYRO_THRESHOLD = 300;
const int GYRO_MIN_SPEED_THRESHOLD = 2000;
const int BRIGHTNESS_THRESHOLD = 10;
const int GYRO_CALIBRATION_THRESHOLD = 1000;
const int BRIGHTNESS_CALIBRATION_THRESHOLD = 10;
const unsigned long TIMEOUT_USEC = 2000000L;
const unsigned long CALIBRATE_USEC = 1000000L;

// Arrays to hold measurements surrounding the reversal for debugging
#ifdef PRINT_TRACE
const int TRACE_SIZE = 250;
const int TRACE_SKIP = 5;
static int trace_count = 0;
static int trace_skip_count = 0;
char trace_gyroY[TRACE_SIZE];
unsigned char trace_bright[TRACE_SIZE];
#endif

//*****************************************************
void setup() 
//*****************************************************
{
  Serial.begin(9600);
  Serial.print("OSVR_latency_hardware_firmware turnaround test v01.01.00\n");
  Serial.print(" Mount the photosensor rigidly on the eyepiece or screen.\n");
  Serial.print(" Rotate the inertial sensor along with the tracking hardware.\n");
  Serial.print(" Make the app vary brightness darker in one direction and lighter in the other.\n");
  Serial.print(" Latencies reported in microseconds, 2-second timeout\n");
  Serial.print(" Hold the device still for 2 seconds.\n");
  
  MPU6000_Init();
  delay(100);
}

//*****************************************************
void loop() 
//*****************************************************
{
  // Read the values from the inertial sensors and photosensor.
  // Record the time we read these values.
  int accX = getXAccelerometerCounts();
  int accY = getYAccelerometerCounts();
  int accZ = getZAccelerometerCounts();
  int gyroX = getXGyroCounts();
  int gyroY = getYGyroCounts();
  int gyroZ = getZGyroCounts();
  int brightness = analogRead(A0);
  unsigned long now = micros();
  
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
  
  // Keep track of the reversals in brightness and the time at
  // which the brightness reached within threshold of the value
  // it held before the inverted jump.  We keep track of the
  // previous change away from threshold and check it against
  // a current change to see if the polarities are in the opposite
  // direction.  If so, then we set last_brightness_reach_end_time to
  // now.  Brightness change direction and last brightness to
  // reach the end should both be set to 0 when entering the
  // S_REVERSE_BRIGHTNESS state so that they will have to be
  // filled in by actual changes.
  static int last_brightness_change_direction = 0;
  static unsigned long int last_brightness_reach_end_time = 0;

  // Keep track of when the last time the gyroscope's motion
  // dropped below threshold.  Used by the brightness state to
  // determine latency.  Set to 0 when entering the S_REVERSE_DIRECTION
  // state.
  static unsigned long last_gyroscope_settling_time = 0;
  
  // We run a finite-state machine that cycles through cases of waiting for
  // a period of non motion, determining the axis and brightness direction
  // associated with motion, looking for extrema of motion, looking for
  // extrema of brightness, and reporting statistics.
  static enum { S_CALM, S_CALIBRATE, S_REVERSE_DIRECTION, 
                S_REVERSE_BRIGHTNESS } state = S_CALM;
  static unsigned long calm_start = now;
  static unsigned long calibration_start = now;
  
  // Which axis to use for tracking the motion?  Initially set to
  // NULL and then set in the calibration code to the axis with the
  // largest motion.
  static int *tracking_axis = NULL;
  
  // Statistics of latencies.
  static int latency_count = 0;
  static unsigned long latency_sum = 0;
  static unsigned long latency_max = 0;
  static unsigned long latency_min = TIMEOUT_USEC;
  
  // Whether or not we're in calm mode, if we hold still for the
  // timeout duration, we reset statistics and go into calibrate
  // mode.
  if (moving(accX, accY, accZ, gyroX, gyroY, gyroZ)) {
    calm_start = now;
  } else if (now - calm_start >= TIMEOUT_USEC) {
    if (latency_count > 0) {
      Serial.print("Min latency: "); Serial.print(latency_min); Serial.print("\n");
      Serial.print("Max latency: "); Serial.print(latency_max); Serial.print("\n");
      Serial.print("Mean latency: "); Serial.print(latency_sum/latency_count); Serial.print("\n");
    }
    Serial.print("Statistics reset.  Initiate periodic motion to calibrate.\n");
    
    latency_count = 0;
    latency_sum = 0;
    latency_max = 0;
    latency_min = TIMEOUT_USEC;
    
    calm_start = now;
    calibration_start = now;
    tracking_axis = NULL;
    state = S_CALIBRATE;
  }
  
#ifdef PRINT_TRACE
  if ( (state != S_CALIBRATE) && (state != S_CALM) ) {
    if (++trace_skip_count >= TRACE_SKIP) {
      trace_skip_count = 0;
      if (trace_count < TRACE_SIZE) {
        trace_gyroY[trace_count] = gyroY/100;
        trace_bright[trace_count] = brightness/4;        
        trace_count++;
      }
    }
  } 
#endif
  
  switch (state) {
    // Wait for a period where there is no motion above
    // the motion threshold.
    case S_CALM:
      {
        // Nothing extra to do; we're always checking for calm above.
      }
      break;
      
    // Determine which of the rotational axes is moving the most and
    // then wait for the start of a move in the direction that makes
    // the photosensor brighter.
    case S_CALIBRATE:
      {
        // The first loop iteration, we reset our variables.
        static int minX, maxX, minY, maxY, minZ, maxZ;
        static int minBright, maxBright;
        if (calibration_start == now) {
          minX = maxX = gyroX;
          minY = maxY = gyroY;
          minZ = maxZ = gyroZ;
          minBright = maxBright = brightness;
        }
        
        // Keep track of the maximum and minimum in all 3 axes;
        if (gyroX < minX) { minX = gyroX; }
        if (gyroX > maxX) { maxX = gyroX; }
        if (gyroY < minY) { minY = gyroY; }
        if (gyroY > maxY) { maxY = gyroY; }
        if (gyroZ < minZ) { minZ = gyroZ; }
        if (gyroZ > maxZ) { maxZ = gyroZ; }
        if (brightness < minBright) { minBright = brightness; }
        if (brightness > maxBright) { maxBright = brightness; }
        
        // If it has been long enough, and the tracking axis has not yet been
        // set, set it to the one that has the largest range.
        // First, check to make sure we've seen sufficient change in the
        // maximum axis and in the brightness.
        if ( (tracking_axis == NULL) && (now - calibration_start >= CALIBRATE_USEC) ) {
          int maxRange = maxX - minX;
          tracking_axis = &gyroX;
          int yRange = maxY - minY;
          if (yRange > maxRange) {
            maxRange = yRange;
            tracking_axis = &gyroY;
          }
          int zRange = maxZ - minZ;
          if (zRange > maxRange) {
            maxRange = zRange;
            tracking_axis = &gyroZ;
          }
          if ( (maxRange < GYRO_CALIBRATION_THRESHOLD) ||
               (maxBright - minBright < BRIGHTNESS_CALIBRATION_THRESHOLD) ) {
            Serial.print("Insufficient change for calibrating, retrying\n");
            Serial.print("  Gyroscope difference = ");
            Serial.print(maxRange);
            Serial.print("\n");
            Serial.print("  Brightness difference = ");
            Serial.print(maxBright - minBright);
            Serial.print("\n");
            tracking_axis = NULL;
            calibration_start = now;
          } else {
            // We have not yet dropped below motion threshold
            // for this iteration.
            last_gyroscope_settling_time = 0;
            state = S_REVERSE_DIRECTION;
            Serial.print("Calibration complete, measuring latencies on ");
            if (tracking_axis == &gyroX) {
              Serial.print("X axis");
            } else if (tracking_axis == &gyroY) {
              Serial.print("Y axis");
            } else {
              Serial.print("Z axis");
            }
            Serial.print(" (brightness difference ");
            Serial.print(maxBright - minBright);
            Serial.print(")\n");
            Serial.print("Continue periodic motion to test latency.\n");
          }
        }
      }
      break;

    // Wait until the motion settles down (so that we were moving
    // but now have stopped.
    case S_REVERSE_DIRECTION:
      {
        // Don't say that we were moving until we get to a speed that
        // is above a larger threshold to keep us from getting lots
        // of spurious false stopping estimates.
        static bool have_moved_fast = false;
        
        // Keep track of when the gyroscope value most recently
        // dropped below threshold (was above threshold in the previous
        // time step and dropped below threshold this time).
        static int last_gyroscope_value = 0;
        if (tracking_axis != NULL) {
          if ( (*tracking_axis > GYRO_MIN_SPEED_THRESHOLD) ||
               (*tracking_axis < -GYRO_MIN_SPEED_THRESHOLD) ) {
                 have_moved_fast = true;
          }
          if ( have_moved_fast &&
               (gyro_direction(*tracking_axis) == 0) &&
               (gyro_direction(last_gyroscope_value) != 0) ) {
            last_gyroscope_settling_time = now;
          }
          last_gyroscope_value = *tracking_axis;
        } else {
          last_gyroscope_value = 0;
        }
  
        if (last_gyroscope_settling_time != 0) {
          //Serial.print("Stopped\n");
          
          // We have dropped to a stop, so we need to set the trigger for
          // moving fast again and make sure we actually move before the
          // last gyro value registers a move.
          have_moved_fast = false;
          last_gyroscope_value = 0;

          // Flush the change status so that we have to get a brightness change
          // in one direction followed by a change in the other starting now.
          last_brightness_reach_end_time = 0;
          last_brightness_change_direction = 0;
          state = S_REVERSE_BRIGHTNESS;
        }
      }
      break;

    // Wait until the brightness changes direction and then report
    // the time difference between when motion dropped below
    // threshold and when brightness came within threshold of its
    // extreme value before reversing direction.
    case S_REVERSE_BRIGHTNESS:
      {
        static int last_unchanged_brightness_value = 0;
        static unsigned long last_brightness_change_time = 0;
        int this_change = brightness_direction(brightness - last_unchanged_brightness_value);
        if (this_change != 0) {
          if (this_change * last_brightness_change_direction == -1) {
            last_brightness_reach_end_time = last_brightness_change_time;
            /*
            if (this_change == -1) {
              Serial.print(" +");
            } else {
              Serial.print(" -");
            }
            Serial.print(last_unchanged_brightness_value);
            Serial.print("->");
            Serial.print(brightness);
            Serial.print("\n");
            */
          }
          last_unchanged_brightness_value = brightness;
          last_brightness_change_direction = this_change;
          last_brightness_change_time = now;
        }
      
        if (last_brightness_reach_end_time != 0) {
          if (last_brightness_reach_end_time <= last_gyroscope_settling_time) {
            Serial.print("Error: Inverted settling times: gyro settling time ");
            Serial.print(last_gyroscope_settling_time);
            Serial.print(" (resetting)\n");
            state = S_CALM;
          }

#ifdef PRINT_TRACE
          Serial.print("GyroY/100*100,brightness/4*4\n");
          for (int i = 0; i < trace_count; i++) {
            Serial.print(static_cast<int>(trace_gyroY[i])*10);
            Serial.print(",");
            Serial.print(static_cast<int>(trace_bright[i])*4);
            Serial.print("\n");
          }
          Serial.print("\n");
          trace_count = 0;
#endif     
          unsigned long value = last_brightness_reach_end_time - last_gyroscope_settling_time;
          Serial.print(value);
          Serial.print("\n");

          // Track statistics;
          latency_count++;
          if (value < latency_min) { latency_min = value; }
          if (value > latency_max) { latency_max = value; }
          latency_sum += value;
          
          last_gyroscope_settling_time = 0;
          state = S_REVERSE_DIRECTION;
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
inline bool moving(int accX, int accY, int accZ, int gyroX, int gyroY, int gyroZ)
//*****************************************************
{
  bool ret = false;
  
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

// Determine if we are moving in the positive direction above
// threshold (1), in the negative direction below threshold (-1),
// or within threshold of the origin (0);
//*****************************************************
inline int gyro_direction(int value)
//*****************************************************
{
  if (value >= GYRO_THRESHOLD) {
    return 1;
  }
  
  if (value <= -GYRO_THRESHOLD) {
    return -1;
  }
  
  return 0;
}

// Determine if we brightness changed in the positive direction above
// threshold (1), in the negative direction below threshold (-1),
// or not (0);
//*****************************************************
inline int brightness_direction(int value)
//*****************************************************
{
  if (value >= BRIGHTNESS_THRESHOLD) {
    return 1;
  }
  
  if (value <= -BRIGHTNESS_THRESHOLD) {
    return -1;
  }
  
  return 0;
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

