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



#ifndef __ACCELMAGFXOS8700_H__
#define __ACCELMAGFXOS8700_H__


// Include statements
// Check Arduino version
#if (ARDUINO >= 100)
    #include "Arduino.h"
#else
    #include "WProgram.h"
#endif

#include <Wire.h>

#define CONSTS_STD_GRAV (9.81f)  // Gravitational acceleration [m/s/s]

// ----------------------------------------------------------------------------
// Magnetometer calibration constants
// ----------------------------------------------------------------------------
// M_calib[3x1] = S[3x3] * (M_raw[3x1] - B[3x1])
// #define APPLY_MAG_CALIB  // Uncomment to apply magnetometer calibration parameters
#define SENSORCALIB_MAG_SX (1.0f)    // Soft iron matrix X
#define SENSORCALIB_MAG_SY (1.0f)    // Soft iron matrix Y
#define SENSORCALIB_MAG_SZ (1.0f)    // Soft iron matrix Z
#define SENSORCALIB_MAG_BX (0.0f)   // Hard iron offset X
#define SENSORCALIB_MAG_BY (0.0f)  // Hard iron offset Y
#define SENSORCALIB_MAG_BZ (0.0f)  // Hard iron offset Z


// ----------------------------------------------------------------------------
// I2C Address and ID of sensor
// ----------------------------------------------------------------------------
#define FXOS8700_ADDRESS (0x1F)  // 0011111
#define FXOS8700_ID (0xC7)       // 1100 0111


// ----------------------------------------------------------------------------
// Define conversion factors to convert accel. and mag. data to floats
// ----------------------------------------------------------------------------
#define ACCEL_MSS_LSB_2G (0.000244f)  // Convert to [m/s/s]
#define ACCEL_MSS_LSB_4G (0.000488f)  // Convert to [m/s/s]
#define ACCEL_MSS_LSB_8G (0.000976f)  // Convert to [m/s/s]
#define MAG_UT_LSB (0.1f)             // Convert to [uT]


// ----------------------------------------------------------------------------
// Accelerometer speeds/ranges
// ----------------------------------------------------------------------------
typedef enum
{
    ACCEL_RNG_2G = 0x00,
    ACCEL_RNG_4G = 0x01,
    ACCEL_RNG_8G = 0x02
} AccelRanges_t;


// ----------------------------------------------------------------------------
// Accelerometer & magnetometer registers
// ----------------------------------------------------------------------------
typedef enum
{
    ACCELMAG_REG_STATUS     = 0x00,          
    ACCELMAG_REG_ID         = 0x0D,       
    ACCELMAG_REG_XYZ_CFG    = 0x0E,
    ACCELMAG_REG_AOUT_X_MSB = 0x01,
    ACCELMAG_REG_AOUT_X_LSB = 0x02,
    ACCELMAG_REG_AOUT_Y_MSB = 0x03,
    ACCELMAG_REG_AOUT_Y_LSB = 0x04,
    ACCELMAG_REG_AOUT_Z_MSB = 0x05,
    ACCELMAG_REG_AOUT_Z_LSB = 0x06,
    ACCELMAG_REG_CTRL1      = 0x2A,       
    ACCELMAG_REG_CTRL2      = 0x2B,
    ACCELMAG_REG_CTRL3      = 0x2C,
    ACCELMAG_REG_CTRL4      = 0x2D,
    ACCELMAG_REG_CTRL5      = 0x2E,
    ACCELMAG_REG_MSTATUS    = 0x32,         
    ACCELMAG_REG_MOUT_X_MSB = 0x33,      
    ACCELMAG_REG_MOUT_X_LSB = 0x34,
    ACCELMAG_REG_MOUT_Y_MSB = 0x35,
    ACCELMAG_REG_MOUT_Y_LSB = 0x36,
    ACCELMAG_REG_MOUT_Z_MSB = 0x37,
    ACCELMAG_REG_MOUT_Z_LSB = 0x38,
    ACCELMAG_REG_MCTRL1     = 0x5B,      
    ACCELMAG_REG_MCTRL2     = 0x5C,
    ACCELMAG_REG_MCTRL3     = 0x5D
} MagAccelRegisters_t;


// ----------------------------------------------------------------------------
// Accelerometer & Magnetometer Class
// ----------------------------------------------------------------------------
class AccelMagFXOS8700
{
    public:
        AccelMagFXOS8700(int32_t accelID = -1, int32_t magID = -1);
        bool InitializeSensor(AccelRanges_t accRange = ACCEL_RNG_2G);
        bool ReadSensor(void);
        void GetAccelX(float &ptrOut);  // [m/s/s]
        void GetAccelY(float &ptrOut);  // [m/s/s]
        void GetAccelZ(float &ptrOut);  // [m/s/s]
        void GetMagX(float &ptrOut);  // [uT]
        void GetMagY(float &ptrOut);  // [uT]
        void GetMagZ(float &ptrOut);  // [uT]
        float ax;  // X-acceleration [m/s/s]
        float ay;  // Y-acceleration [m/s/s]
        float az;  // Z-acceleration [m/s/s]
        float mx;  // X-magnetic field [uT]
        float my;  // Y-magnetic field [uT]
        float mz;  // Z-magnetic field [uT]
    
    protected:
    private:
        void I2Cwrite8(byte regOfInterest, byte valToWrite);
        byte I2Cread8(byte regOfInterest);
        AccelRanges_t accelRange;
        int32_t accelID;  // 0x8700A
        int32_t magID;  // 0x8700B
};


#endif  // __ACCELMAGFXOS8700_H__
