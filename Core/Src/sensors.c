#include "sensors.h"

bool setupSensors() {
    #if SIMULATE
        Serial.println("(Simulation) Sensors connected successfully!");
        return true;
    #endif

    if (setupMS5611() && setupLSM6DSOX()) {
        if (DEBUG) {Serial.println("Sensors initialized successfully!");}
        m_bmp.performReading();
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
    if (!m_bmp.performReading()) {
        Serial.println("One or more altimeter sensors are not working");
        return false;
    }
    if (!m_sox.getEvent(&accel, &gyro, &temp)) {
        Serial.println("One or more IMU sensors are not working");
        return false;
    }
    return true;
}

/** @brief Setup Altimeter */
//new altimeter new function
bool setupMS5611() {
    if (!m_bmp.begin_SPI(altimeter_chip_select)) {
        Serial.println("Unable to connect to altimeter");
        return false;
    }
    m_bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
    m_bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
    m_bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    m_bmp.setOutputDataRate(BMP3_ODR_50_HZ);
    Serial.println("Altimeter good");
    m_bmp.performReading();
    Serial.println(m_bmp.readAltitude(SEA_LEVEL_PRESSURE_HPA));
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
