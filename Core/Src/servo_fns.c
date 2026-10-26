#include "servo_fns.h"

/** previously "ATS functions" these functions test and control the ATS servo */

/** @brief Test the ATS: fully extend and retract it, then detach the servo */
void testATS() {
    if (DEBUG) { Serial.println("Testing ATS...");}
    attachATS();
    setATSPosition(ATS_IN);
    delay(2000);
    setATSPosition(ATS_OUT); // Initial position
    delay(2000);
    setATSPosition(ATS_IN);  // Reset position
    delay(2000);
    detachATS();
    Serial.println("ATS Test Sucessful...");
}

/** @brief Turn on ATS servo */
void attachATS() {
    requires(!m_atsServo.attached());
    #if SKIP_ATS
        return;
    #endif
    m_atsServo.attach(ATS_PIN);
}

/** @brief Set ATS position  
  * @arg percent_rot: float between 0 (fully in) and 1 (fully out)
  * Only call this function when servo position changes */
void setATSPosition(float percent_rot) {
    #if SKIP_ATS
        return;
    #endif
    float pos = ATS_MIN - (ATS_MIN - ATS_MAX) * (1.0f - percent_rot);
    m_atsServo.write(int(pos));
    if (DEBUG) {Serial.println("ATS position set to " + String(pos));}
    // for debugging: 
    Serial.println("ATS position set to " + String(percent_rot));
}

/** @brief Adjust ATS with PID
  * Adjust the ATS based on the current altitude and desired apogee 
   (stored in absolute_alt_target; the ALT_TARGET represents altitude above the
    launch level, while absolute_alt_target is the altitude above sea level) */
void adjustATS() {
    float targetAcceleration = abs(pow(gVelocityFiltered,2)/(2*(absolute_alt_target-gAltFiltered)));

    // Retract ATS fully after 18 seconds
    if (millis() - gLaunchTime > 18000) {
        setATSPosition(ATS_IN);
        gATSPosition = ATS_IN;
        delay(10);
        return;
    }
    
    // Fully deploy ATS if reached Altitude target
    if (gAltFiltered >= absolute_alt_target) {
        gATSPosition = ATS_OUT;
    }
    else {
        // Calculate desired surface-area to reach target altitude
        float target_area = (pow(gVelocityFiltered, 2)/(absolute_alt_target - gAltFiltered) - 2*GRAVITY)*ROCKET_MASS/(gVelocityFiltered*ATMOSPHERE_FLUID_DENSITY*ROCKET_DRAG_COEFFICIENT);
        Serial.println("Old ATS pos: " + String(target_area/ATS_MAX_SURFACE_AREA));

        // Calculate error in acceleration
        /*Positive if drag + gravity >= target; 
          means if <, we are going TOO FAST and need to slow down*/
        float error = gAccelFiltered - targetAcceleration;

        // Calculate adjustment
        float adjustment;
        if (error > 0) { // Too slow
            adjustment = 0;
        } 
        else { // Too fast
            // Should normalize to 0 to 1
            adjustment = PIDFactor(abs(error), 0.03, 0);
        }

        gATSPosition = adjustment;
        gPredictedAltitude = (0.5*ROCKET_MASS*pow(gVelocityFiltered, 2))/(ROCKET_MASS*GRAVITY + 0.5*ATMOSPHERE_FLUID_DENSITY*ROCKET_DRAG_COEFFICIENT*pow(gVelocityFiltered,2)*ATS_MAX_SURFACE_AREA*adjustment);
        //debug if (DEBUG) {;}
        Serial.println("Predicted altitude: " + String(gPredictedAltitude));
    }

    // ATS window
    if (millis() - gLaunchTime > 4500) {
        if (millis() - gLaunchTime < 6000) {
            // Adjust ATS based on position
            if (millis() - gLaunchTime > 5500){
                setATSPosition(ATS_IN);  // Reset position
            }
            else if (millis() - gLaunchTime > 5000){
                setATSPosition(ATS_OUT); // Initial position
            }
            else{
                setATSPosition(ATS_IN);
            }
            Serial.println("Cycling ATS position");
        } else {
        // Adjust ATS based on position
          setATSPosition(gATSPosition);
          Serial.println("ATS position: " + String(gATSPosition));
        }
    }
}

/** @brief Turn off ATS servo (for saving power) */
void detachATS() {
    requires(m_atsServo.attached());
    #if SKIP_ATS
        return;
    #endif
    m_atsServo.detach();
}
