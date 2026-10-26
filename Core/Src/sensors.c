#include "sensors.h"
#include "main.h" //check include (need to include constants in main.c)
#include <string.h>

/** sensor funtions (setup, test) */

/* SETUP & TEST FUNCTIONS */
/** @brief Initialize all sensors */
bool setupSensors() {
    #if SIMULATE
        Serial.println("(Simulation) Sensors connected successfully!");
        return true;
    #endif

    if (setupLSM6DSOX() && setupLIS3MDL() && setupMS5611()) {
        if (DEBUG) {Serial.println("Sensors initialized successfully!");}
        //CHANGE: m_bmp.performReading();
        Serial.println("Setup reading good");
        return true;
    }
    return false;
}

/** @brief Test Altimeter and IMU */
bool testSensors() {
    #if SIMULATE
        return true;
    #endif
    //CHANGE: 
    // if (!m_bmp.performReading()) {
    //     Serial.println("One or more altimeter sensors are not working");
    //     return false;
    // }
    if (!m_sox.getEvent(&accel, &gyro, &temp)) {
        Serial.println("One or more IMU sensors are not working");
        return false;
    }
    return true;
}

/** @brief Setup IMU */
bool setupLSM6DSOX() {
    if (!m_sox.begin_I2C()) {
        Serial.println("Unable to connect to IMU");
        return false;
    }
    Serial.println("IMU good");
    return true;
}

/** @brief Setup Accelerometer */
bool setupLIS3MDL() {
    //TO-DO
}

/** @brief Setup Barometer */
bool setupMS5611() {
    //TO-DO
}


/* DATA COLLECTION & LOGGING FUNCTIONS */
//CHANGE: 
/** @brief Refresh sensors and update globals with latest measurements
  * Output string can be viewed when DEBUG is true
  * @returns a CSV string with format:
  * - time, pressure (hPa), altitude_raw (m), acceleration_raw_x (m/s^2), 
  * - acceleration_raw_y, acceleration_raw_z, gyro_x (radians/s), gyro_y, gyro_z, 
  * - gAltFiltered, gVelocityFiltered, gAccelFiltered, 
  * - temperature (from IMU, degrees C), gATSPosition (degrees) */
//needs to be updated to char pointer
String getMeasurements() {
    #if SIMULATE
        // If simulation mode is ON, get simulated data from serial bus instead
        return getSimulatedMeasurements();
    #endif

    // Update sensors
    m_bmp.performReading();               // Update Altimeter
    m_sox.getEvent(&accel, &gyro, &temp); // Update IMU
    
    // Pass raw data to be filtered
    filterData(readAltimeter(), readIMU()); 

    // Print measurements
    String movementData = String(m_bmp.pressure/100.0) + ", " + 
                          String(m_bmp.readAltitude(SEA_LEVEL_PRESSURE_HPA)) + ", " + 
                          String(accel.acceleration.x*METERS_TO_FEET) + ", " + 
                          String(accel.acceleration.y*METERS_TO_FEET) + ", " + 
                          String(accel.acceleration.z*METERS_TO_FEET) + ", " + 
                          String(gyro.gyro.x) + ", " + 
                          String(gyro.gyro.y) + ", " + 
                          String(gyro.gyro.z) + ", " + 
                          String(gAltFiltered) + ", " + 
                          String(gVelocityFiltered) + ", " + 
                          String(gAccelFiltered);
    String timeData = String(millis() - gStartTime);
    String sensorData = String(readThermometer());
    if (DEBUG) {Serial.println(timeData + ", " + movementData + ", " + sensorData + ", " + String(gATSPosition));}
    return timeData + "," + movementData + ", " + sensorData + ", " + String(gATSPosition) + ", " + String(gPredictedAltitude);

    // if (DEBUG) {Serial.println(timeData + "," + movementData + "," + sensorData + "," + String(gATSPosition));}
    return timeData + "," + movementData + "," + sensorData + "," + String(gATSPosition) + "," + String(gPredictedAltitude);
}

/** @brief Filter raw data and updates globals
  * still need to implement (TO-DO)*/ 
void filterData(float alt, float acc) {
    float DT = ((float)gTimeDelta) / 1000;

    //CHANGE: 
    KalmanFilter.F = {1.0, DT,  DT*DT / 2,
                      0.0, 1.0, DT,
                      0.0, 0.0, 1.0};

    measurement_state(0) = (double)alt;
    measurement_state(1) = (double)0.0;
    measurement_state(2) = (double)acc;

    obs = KalmanFilter.H * measurement_state;
    KalmanFilter.update(obs);
    // BLA::Matrix<NumObservations, 3 > current_obs = {alt, 0.0, 0.0,
    //                                                 0.0, 0.0, acc};
    // KalmanFilter.update(current_obs);
    gAltFiltered = KalmanFilter.x(0);
    gVelocityFiltered = KalmanFilter.x(1);
    gAccelFiltered = KalmanFilter.x(2);
    // Serial << alt << "," << acc <<"," << gAltFiltered << "," << gVelocityFiltered << "," << gAccelFiltered << "\n";
    //Serial << "Current accel: " << gAccelFiltered << "\n";
    // Serial.println("Filtered Altitude: " + String(gAltFiltered) + " Filtered Velocity: " + String(gVelocityFiltered) + " Filtered Acceleration: " + String(gAccelFiltered));
}

/** @brief Detect if rocket has landed 
 * if the rocket has been reasonably still for 5 seconds, it is considered landed
*/
bool detectLanding() {
    if (gLaunchTime != 0 && millis() - gLaunchTime > DEF_cutoff_landing_time) { return true; } 
    //return false;

    //check landed
    static unsigned int landed_cd = 5000;
    static unsigned long last_check = millis();

    if ((abs(gAccelFiltered)-32) < 2 && abs(gVelocityFiltered)< 2){
        landed_cd -= (int)(millis() - last_check);
    }
    else if (landed_cd > 0){
        landed_cd = 5000;
    }

    if (landed_cd <= 0){
        return true;
    }
    return false;
}

//CHANGE: 
// /**  @brief  Read altitude from altimeter
// * IMPORTANT: update altimeter with `performreading()` beforehand
// * @returns raw altitude (meters)
// */
// float readAltimeter() {
//     float altimeter_data = m_bmp.readAltitude(SEA_LEVEL_PRESSURE_HPA) * METERS_TO_FEET;
//     if (DEBUG) {Serial.println("Altimeter: " + String(altimeter_data));}
//     return altimeter_data;
// }

/** @brief Read acceleration from IMU 
  * IMPORTANT: update IMU with `getevent()` first for current reading
  * @returns raw acceleration (ft/s^2) */
float readIMU() {
    float imu_data = pow(pow(accel.acceleration.x,2) + pow(accel.acceleration.y, 2) + pow(accel.acceleration.z, 2), 0.5) * METERS_TO_FEET;
    if (DEBUG) {Serial.println("IMU: " + String(imu_data));}
    return imu_data;
}

/** @brief Read temperature from IMU
  * IMPORTANT: update IMU with `getevent()` beforehand
  * @returns temperature (degrees C) */
float readThermometer() {
    float thermometer_data = temp.temperature;
    if (DEBUG) {Serial.println("Thermometer: " + String(thermometer_data));}
    return thermometer_data;
}

/** @brief Runs PID (proportional (current), integral (past), derivative (future))
  * @arg error: acceleration error
  * @arg Kp: proportional gain
  * @arg Kd: derivative gain
  * - positive error means rocket is going too slow
  * - negative error means rocket is going too fast
  * - Kp: higher = stronger immediate response
  * - Kd: adjusts response based on how frequently error changes
  * @return the absolute position to adjust ATS to in degrees */
float PIDFactor(int error, float Kp, float Kd) {
    static int old_error = 0;
    float proportional = error * Kp;
    float derivative = (error - old_error) * Kd;
    old_error = error;
    return proportional + derivative; 
}