/** sensor funtions (setup, test, data collection) */

/* SETUP & TEST FUNCTIONS */
/** @brief Initialize all sensors */
bool setupSensors();

/** @brief Test sensors */
bool testSensors();

/** @brief Setup IMU */
bool setupLSM6DSOX();

/** @brief Setup Accelerometer */
bool setupLIS3MDL();

/** @brief Setup Barometer */
bool setupMS5611();

/* DATA COLLECTION & LOGGING FUNCTIONS */
/** @brief Refresh sensors and update globals with latest measurements
  * Output string can be viewed when DEBUG is true
  * @returns a CSV string with format:
  * - time, pressure (hPa), altitude_raw (m), acceleration_raw_x (m/s^2), 
  * - acceleration_raw_y, acceleration_raw_z, gyro_x (radians/s), gyro_y, gyro_z, 
  * - gAltFiltered, gVelocityFiltered, gAccelFiltered, 
  * - temperature (from IMU, degrees C), gATSPosition (degrees) */
String getMeasurements();

/** @brief Filter raw data and updates globals
  * still need to implement (TO-DO)*/ 
void filterData(float alt, float acc);

/** @brief Detect if rocket has landed 
 * if the rocket has been reasonably still for 5 seconds, it is considered landed
*/
bool detectLanding();

//CHANGE: 
// /**  @brief  Read altitude from altimeter
// * IMPORTANT: update altimeter with `performreading()` beforehand
// * @returns raw altitude (meters)
// */
// float readAltimeter();

/** @brief Read acceleration from IMU 
  * IMPORTANT: update IMU with `getevent()` first for current reading
  * @returns raw acceleration (ft/s^2) */
float readIMU();

/** @brief Read temperature from IMU
  * IMPORTANT: update IMU with `getevent()` beforehand
  * @returns temperature (degrees C) */
float readThermometer();

/** @brief Runs PID (proportional (current), integral (past), derivative (future))
  * @arg error: acceleration error
  * @arg Kp: proportional gain
  * @arg Kd: derivative gain
  * - positive error means rocket is going too slow
  * - negative error means rocket is going too fast
  * - Kp: higher = stronger immediate response
  * - Kd: adjusts response based on how frequently error changes
  * @return the absolute position to adjust ATS to in degrees */
float PIDFactor(int error, float Kp, float Kd);
