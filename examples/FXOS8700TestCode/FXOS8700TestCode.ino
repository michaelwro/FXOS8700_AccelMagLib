// ----------------------------------------------------------------------------
// 
// EXAMPLE SKETCH FOR THE ACCELMAGFXOS8700 SENSOR LIBRARY
// 
// Code By: Michael Wrona | GitHub: @michaelwro
// Created: 24 July 2020
// ----------------------------------------------------------------------------
/**
 * This is an example Arduino sketch that uses the AccelMagFXOS8700
 * accelerometer/magnetometer sensor class. Accelerometer readings are output
 * in [m/s^2] and magnetometer readings are output in [uT] (microtesla).
 * 
 * This code was tested with Adafruit's FXAS21002C/FXOS8700 I2C 9-DOF IMU.
 * 
 * I2C ARDUINO UNO WIRING
 * ----------------------
 * FXOS8700 VIN -> Arduino Uno 3.3V
 * FXOS8700 GND -> Arduino Uno Ground
 * FXOS8700 SDA -> Arduino Uno A4
 * FXOS8700 SCL -> Arduino Uno A5
 * 
 * References
 * ----------
 * ~ FXOX8700 + FXAS21002 9-DOF IMU (Adafruit):
 *     https://www.adafruit.com/product/3463
 */


// Include statements
#include <Wire.h>
#include "AccelMagFXOS8700.h"

// Sensor class
AccelMagSensor AccelMag = AccelMagSensor(0x8700A, 0x8700B);

// Variables
unsigned long samplePeriod = 100;  // Read sensor every n milliseconds.
unsigned long currMillis = 0;
unsigned long prevMillis = 0;

float ax, ay, az;  // Store acceleration readings
float mx, my, mz;  // Store magnetometer readings


void setup() {
    Serial.begin(9600);  // Serial port
    while (Serial == false)
        delay(1);
    
    /**
     * Initialize sensor. Accelerometer measurement range options are ACCEL_RNG_2G,
     * ACCEL_RNG_4G, and ACCEL_RNG_8G.
     */
    if (AccelMag.InitializeSensor(ACCEL_RNG_4G) == false) {
        Serial.println("Error initializing FXOS8700. Check wiring.");
        while (1);
    }

    delay(500);  // Wait a sec, just in case it needs to finish init'ing
}


void loop() {

    currMillis = millis();  // Use millis() to perform readings
    if (currMillis - prevMillis >= samplePeriod) {
        prevMillis = currMillis;  // Replace time

        // Perform reading. Terminate if reading failed.
        if (AccelMag.ReadSensor() == false) {
            Serial.println("Sensor reading failed. Terminating.")
            return;
        } 
        
        /**
         * Sensor data can be extracted two ways. The first assigns a value to a
         * variable via its pointer. The second is to use the public variables
         * stored in the class. You can use whatever method suits your project
         * better.
         * 
         * NOTE: The data type is float!
         */

        /* Method 1: Pointer method (preferred) */
        AccelMag.GetAccelX(ax);
        AccelMag.GetAccelY(ay);
        AccelMag.GetAccelZ(az);
        AccelMag.GetMagX(mx);
        AccelMag.GetMagY(my);
        AccelMag.GetMagZ(mz);

        Serial.print("Ax: ");  // In [m/s^2]
        Serial.print(ax, 4);
        Serial.print("\tAy: ");
        Serial.print(ay, 4);
        Serial.print("\tAz: ");
        Serial.print(az, 4);
        Serial.print("\tMx: ");  // In [uT] (microtesla)
        Serial.print(mx, 2);
        Serial.print("\tMy: ");
        Serial.print(my, 2);
        Serial.print("\tMz: ");
        Serial.println(mz, 2);

            
        /* Method 2: Public class variables */
        // Serial.print("Ax: ");  // In [m/s^2]
        // Serial.print(AccelMag.ax, 4);
        // Serial.print("\tAy: ");
        // Serial.print(AccelMag.ay, 4);
        // Serial.print("\tAz: ");
        // Serial.print(AccelMag.az, 4);
        // Serial.print("\tMx: ");  // In [uT] (microtesla)
        // Serial.print(AccelMag.mx, 2);
        // Serial.print("\tMy: ");
        // Serial.print(AccelMag.my, 2);
        // Serial.print("\tMz: ");
        // Serial.println(AccelMag.mz, 2);
    }

}
