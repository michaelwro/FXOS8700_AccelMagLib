# FXOS8700 Accelerometer/Magnetometer Arduino Sensor Library

**Code By:** Michael Wrona | B.S. Aerospace Engineering | GitHub: [@michaelwro](https://github.com/michaelwro)

## Description

This is a sensor library for the I2C FXOS8700 Accelerometer/Magnetometer sensor. Tested with Adafruit's [Precision NXP 9-DOF Breakout Board - FXOS8700 + FXAS21002](https://www.adafruit.com/product/3463) and an Arduino Uno.

## How to Use

To install this code library, clone this repository and place it in your Arduino libraries folder. __An example Arduino sketch is also included to get you up and going and demonstrate how to use the library.__

If you have magnetometer calibration parameters for hard and soft iron distortions, they can be programmed in AccelMagFXOS8700.h.

```cpp
// M_calib = S[3x3] * (M_raw[3x1] - B[3x1])
#define APPLY_MAG_CALIB  // Toggle comment to apply magnetometer calibration parameters
#define SENSORCALIB_MAG_SX (1.0f)    // Soft iron matrix X
#define SENSORCALIB_MAG_SY (1.0f)    // Soft iron matrix Y
#define SENSORCALIB_MAG_SZ (1.0f)    // Soft iron matrix Z
#define SENSORCALIB_MAG_BX (0.0f)   // Hard iron offset X
#define SENSORCALIB_MAG_BY (0.0f)  // Hard iron offset Y
#define SENSORCALIB_MAG_BZ (0.0f)  // Hard iron offset Z
```

## Arduino Wiring

* FXOS8700 VIN -> Arduino Uno 3.3V
* FXOS8700 GND -> Arduino Uno Ground
* FXOS8700 SDA -> Arduino Uno A4
* FXOS8700 SCL -> Arduino Uno A5


## To-Do

- [ ] Add debug print statements?
- [ ] Add accelerometer calibration parameters?



