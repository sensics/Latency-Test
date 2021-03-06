// OSRVR latency-testing hardware firmware source code.
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

//*****************************************************
void setup() 
//*****************************************************
{
  Serial.begin(9600);
  Serial.print("OSVR_latency_hardware_firmware print_intertial v01.00.00\n");
  
  MPU6000_Init();
  delay(100);
}

//*****************************************************
void loop() 
//*****************************************************
{
  int accX, accY, accZ;
  int gyroX, gyroY, gyroZ;
  accX = getXAccelerometerCounts();
  accY = getYAccelerometerCounts();
  accZ = getZAccelerometerCounts();
  gyroX = getXGyroCounts();
  gyroY = getYGyroCounts();
  gyroZ = getZGyroCounts();

  Serial.print("Acc XYZ, Gyro XYZ "); Serial.print(accX);
  Serial.print(" "); Serial.print(accY);
  Serial.print(" "); Serial.print(accZ);
  Serial.print(", "); Serial.print(gyroX);
  Serial.print(" "); Serial.print(gyroY);
  Serial.print(" "); Serial.print(gyroZ);
  Serial.print("\n");
  delay(500);
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

