/** previously "ATS functions" these functions test and control the ATS servo */

/** @brief Test the ATS: fully extend and retract it, then detach the servo */
void testATS();

/** @brief Turn on ATS servo */
void attachATS();

/** @brief Set ATS position  
  * @arg percent_rot: float between 0 (fully in) and 1 (fully out)
  * Only call this function when servo position changes */
void setATSPosition(float percent_rot);

/** @brief Adjust ATS with PID
  * Adjust the ATS based on the current altitude and desired apogee 
   (stored in absolute_alt_target; the ALT_TARGET represents altitude above the
    launch level, while absolute_alt_target is the altitude above sea level) */
void adjustATS();

/** @brief Turn off ATS servo (for saving power) */
void detachATS();
