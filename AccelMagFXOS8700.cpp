// ----------------------------------------------------------------------------
// FXOS8700 ACCELEROMETER & MAGNETOMETER SENSOR LIBRARY
// A sensor library for the FXOS8700 I2C accelerometer and magnetometer sensor.
// Inspired by Adafruit's FXOS8700 Library (see Resources).
//
// Code By: Michael Wrona | GitHub: @michaelwro
// Created: 24 July 2020
// ----------------------------------------------------------------------------
/**
 * This is the sensor library for the FXOS8700 I2C accelerometer/magnetometer
 * sensor. This library was inspired by Adafruit's FXOS8700 Library
 * (see Resources). Tested and verified with Adafruit's FXAS21002C/FXOS8700
 * 9-DOF IMU.
 * 
 * If available, magnetometer calibration parameters for soft and hard-iron
 * distortions can be defined in AccelMagFXOS8700.h
 * 
 * Resources
 * ---------
 * ~ Adafruit FXOS8700 Sensor Library (GitHub):
 *     https://github.com/adafruit/Adafruit_FXOS8700
 */


#if ARDUINO >= 100
    #include "Arduino.h"
#else
    #include "WProgram.h"
#endif

#include <Wire.h>
#include <limits.h>
#include "AccelMagFXOS8700.h"



// ------------------------------------
// Public methods
// ------------------------------------

// ----------------------------------------------------------------------------
// AccelMagSensor(int32_t accelID, int32_t magID)
// ----------------------------------------------------------------------------
/**
 * Constructor for the FXOS8700 Accelerometer/Magnetometer class. Be sure to
 * specify the sensor ID's.
 * accelID = 0x8700A
 * magID = 0x8700B
 * 
 * @param accelID  I2C address of the accelerometer
 * @param magID    I2C address of the magnetometer
 */
AccelMagSensor::AccelMagSensor(int32_t accelID, int32_t magID)
{
    this->accelID = accelID;
    this->magID = magID;
}


// ----------------------------------------------------------------------------
// InitializeSensor(AccelRanges_t accRange)
// ----------------------------------------------------------------------------
/**
 * Initialize accelerometer, set accel. measurement range, configure
 * magnetometer. ACCEL_RNG_2G is default if the accelerometer measurement range
 * is not specified.
 * 
 * @param accRange  Desired accelerometer measurement range (ACCEL_RNG_2G,
 *                  ACCEL_RNG_4G, ACCEL_RNG_8G)
 */
bool AccelMagSensor::InitializeSensor(AccelRanges_t accRange)
{
    uint8_t connectedSensorID;

    Wire.begin();  // Init. communication
    this->accelRange = accRange; // Set accelerometer range
    
    /* Check to make sure the ID register on the sensor matches the
    expected FXOS8700 ID. */
    connectedSensorID = this->I2Cread8(ACCELMAG_REG_ID);
    
    if (connectedSensorID != FXOS8700_ID)
        return false;
    
    /* Set sensor to STANDBY MODE in order to make changes to/configure
    a few registers. */
    this->I2Cwrite8(ACCELMAG_REG_CTRL1, 0);
    
    // Begin configuration
    switch (this->accelRange)  // Set accel. measurement range
    {
        case (ACCEL_RNG_2G):
            this->I2Cwrite8(ACCELMAG_REG_XYZ_CFG, 0x00);
            break;
        case (ACCEL_RNG_4G):
            this->I2Cwrite8(ACCELMAG_REG_XYZ_CFG, 0x01);
            break;
        case (ACCEL_RNG_8G):
            this->I2Cwrite8(ACCELMAG_REG_XYZ_CFG, 0x02);
            break;
    }

    this->I2Cwrite8(ACCELMAG_REG_CTRL2, 0x02);  // High resolution
    this->I2Cwrite8(ACCELMAG_REG_CTRL1, 0x15);  // Active, Normal Mode, Low Noise, 100Hz in Hybrid Mode

    // Configure magnetometer
    this->I2Cwrite8(ACCELMAG_REG_MCTRL1, 0x1F);  // Hybrid Mode, Over Sampling Rate = 1
    this->I2Cwrite8(ACCELMAG_REG_MCTRL2, 0x20);  // Jump to reg 0x33 after reading 0x06

    return true;
}


// ----------------------------------------------------------------------------
// ReadSensor(void)
// ----------------------------------------------------------------------------
/**
 * Read acceleration and magnetometer data from the FXOS8700 sensor registers.
 * If defined, apply sensor calibration paramerters.
 * 
 * @return  True if successful, false if failed.
 */
bool AccelMagSensor::ReadSensor(void)
{
    // Read 13 bytes from sensor
    Wire.beginTransmission((byte)FXOS8700_ADDRESS);
    #if ARDUINO >= 100
        Wire.write(ACCELMAG_REG_STATUS | 0x80);
    #else
        Wire.send(ACCELMAG_REG_STATUS | 0x80);
    #endif
    Wire.endTransmission();
    Wire.requestFrom((byte)FXOS8700_ADDRESS, (byte)13);

    #if ARDUINO >= 100
        uint8_t status = Wire.read();
        uint8_t axhi = Wire.read();
        uint8_t axlo = Wire.read();
        uint8_t ayhi = Wire.read();
        uint8_t aylo = Wire.read();
        uint8_t azhi = Wire.read();
        uint8_t azlo = Wire.read();
        uint8_t mxhi = Wire.read();
        uint8_t mxlo = Wire.read();
        uint8_t myhi = Wire.read();
        uint8_t mylo = Wire.read();
        uint8_t mzhi = Wire.read();
        uint8_t mzlo = Wire.read();
    #else
        uint8_t status = Wire.receive();
        uint8_t axhi = Wire.receive();
        uint8_t axlo = Wire.receive();
        uint8_t ayhi = Wire.receive();
        uint8_t aylo = Wire.receive();
        uint8_t azhi = Wire.receive();
        uint8_t azlo = Wire.receive();
        uint8_t mxhi = Wire.receive();
        uint8_t mxlo = Wire.receive();
        uint8_t myhi = Wire.receive();
        uint8_t mylo = Wire.receive();
        uint8_t mzhi = Wire.receive();
        uint8_t mzlo = Wire.receive();
    #endif

    /* Read and shift values from registers into integers.
    Accelerometer data is 14-bit and left-aligned. Shift two bits right. */
    this->ax = (int16_t)((axhi << 8) | axlo) >> 2;
    this->ay = (int16_t)((ayhi << 8) | aylo) >> 2;
    this->az = (int16_t)((azhi << 8) | azlo) >> 2;
    this->mx = (int16_t)((mxhi << 8) | mxlo);
    this->my = (int16_t)((myhi << 8) | mylo);
    this->mz = (int16_t)((mzhi << 8) | mzlo);

    // Convert acceleration from int16_t to float in units of [m/s/s]
    switch (this->accelRange)
    {
        case (ACCEL_RNG_2G):
            this->ax *= ACCEL_MSS_LSB_2G * CONSTS_STD_GRAV;
            this->ay *= ACCEL_MSS_LSB_2G * CONSTS_STD_GRAV;
            this->az *= ACCEL_MSS_LSB_2G * CONSTS_STD_GRAV;
            break;
        case (ACCEL_RNG_4G):
            this->ax *= ACCEL_MSS_LSB_4G * CONSTS_STD_GRAV;
            this->ay *= ACCEL_MSS_LSB_4G * CONSTS_STD_GRAV;
            this->az *= ACCEL_MSS_LSB_4G * CONSTS_STD_GRAV;
            break;
        case (ACCEL_RNG_8G):
            this->ax *= ACCEL_MSS_LSB_8G * CONSTS_STD_GRAV;
            this->ay *= ACCEL_MSS_LSB_8G * CONSTS_STD_GRAV;
            this->az *= ACCEL_MSS_LSB_8G * CONSTS_STD_GRAV;
            break;
    }

    // TODO: APPLY ACCEL. CALIBRATION IF NEEDED

    // Convert magnetometer to uT
    this->mx *= MAG_UT_LSB;
    this->my *= MAG_UT_LSB;
    this->mz *= MAG_UT_LSB;

    // Apply magnetometer calibration
    #ifdef APPLY_MAG_CALIB
        this->mx = SENSORCALIB_MAG_SX * (this->mx - SENSORCALIB_MAG_BX);
        this->my = SENSORCALIB_MAG_SY * (this->my - SENSORCALIB_MAG_BY);
        this->mz = SENSORCALIB_MAG_SZ * (this->mz - SENSORCALIB_MAG_BZ);
    #endif

    return true;
}


// ----------------------------------------------------------------------------
// GetAccelX(float &ptrOut)
// ----------------------------------------------------------------------------
/**
 * Return calibrated x-acceleration in [m/s/s]. Assign value to pointer.
 * 
 * @param ptrOut  Pointer to to assign value to.
 */
void AccelMagSensor::GetAccelX(float &ptrOut)
{
    ptrOut = this->ax;
}


// ----------------------------------------------------------------------------
// GetAccelY(float &ptrOut)
// ----------------------------------------------------------------------------
/**
 * Return calibrated y-acceleration in [m/s/s]. Assign value to pointer.
 * 
 * @param ptrOut  Pointer to to assign value to.
 */
void AccelMagSensor::GetAccelY(float &ptrOut)
{
    ptrOut = this->ay;
}


// ----------------------------------------------------------------------------
// GetAccelZ(float &ptrOut)
// ----------------------------------------------------------------------------
/**
 * Return calibrated z-acceleration in [m/s/s]. Assign value to pointer.
 * 
 * @param ptrOut  Pointer to to assign value to.
 */
void AccelMagSensor::GetAccelZ(float &ptrOut)
{
    ptrOut = this->az;
}


// ----------------------------------------------------------------------------
// GetMagX(float &ptrOut)
// ----------------------------------------------------------------------------
/**
 * Return calibrated x-mangetometer reading in [uT]. Assign value to pointer.
 * 
 * @param ptrOut  Pointer to to assign value to.
 */
void AccelMagSensor::GetMagX(float &ptrOut)
{
    ptrOut = this->mx;
}


// ----------------------------------------------------------------------------
// GetMagY(float &ptrOut)
// ----------------------------------------------------------------------------
/**
 * Return calibrated y-mangetometer reading in [uT]. Assign value to pointer.
 * 
 * @param ptrOut  Pointer to to assign value to.
 */
void AccelMagSensor::GetMagY(float &ptrOut)
{
    ptrOut = this->my;
}


// ----------------------------------------------------------------------------
// GetMagZ(float &ptrOut)
// ----------------------------------------------------------------------------
/**
 * Return calibrated z-mangetometer reading in [uT]. Assign value to pointer.
 * 
 * @param ptrOut  Pointer to to assign value to.
 */
void AccelMagSensor::GetMagZ(float &ptrOut)
{
    ptrOut = this->mz;
}


// ----------------------------------------------------------------------------
// ~AccelMagSensor(void)
// ----------------------------------------------------------------------------
/**
 * Deconstructor for the FXOS8700 Accelerometer/Magnetometer class.
 */
AccelMagSensor::~AccelMagSensor(){}



// ------------------------------------
// Private methods
// ------------------------------------


// ----------------------------------------------------------------------------
// I2Cwrite8(byte regOfInterest, byte valToWrite)
// ----------------------------------------------------------------------------
/**
 * Write to FXOS8700 register over I2C.
 * 
 * @param regOfInterest  Register address on device.
 * @param valToWrite     Value to write to register.
 */
void AccelMagSensor::I2Cwrite8(byte regOfInterest, byte valToWrite)
{
    // Init. communication
    Wire.beginTransmission(FXOS8700_ADDRESS);
    #if ARDUINO >= 100
        Wire.write((uint8_t)regOfInterest);
        Wire.write((uint8_t)valToWrite);
    #else
        Wire.send(regOfInterest);
        Wire.send(valToWrite);
    #endif
    Wire.endTransmission();
}


// ----------------------------------------------------------------------------
// I2Cread8(byte regOfInterest)
// ----------------------------------------------------------------------------
/**
 * Read FXOS8700 register value over I2C.
 * 
 * @param regOfInterest  Register address on device.
 * @return               Value/data in register.
 */
byte AccelMagSensor::I2Cread8(byte regOfInterest)
{
    byte val;

    // Init. communication
    Wire.beginTransmission((byte)FXOS8700_ADDRESS);
    #if ARDUINO >= 100
        Wire.write((uint8_t)regOfInterest);
    #else
        Wire.send(regOfInterest);
    #endif

    // Check for failure
    if (Wire.endTransmission(false) != 0) return 0;
    
    // Read register
    Wire.requestFrom((byte)FXOS8700_ADDRESS, (byte)1);
    #if ARDUINO >= 100
        val = Wire.read();
    #else
        val = Wire.receive();
    #endif

    return val;
}
