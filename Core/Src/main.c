/* USER CODE BEGIN Header */
/**
  *****************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  *****************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  *****************************************************************************
  */
/* USER CODE END Header */


/* Includes -----------------------------------------------------------------*/
#include "main.h"


/* Private includes ---------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/** @brief includes here will be protected when regenerating code from CubeMX */
#include "linear-algebra.h"
#include <Servo.h>
#include <Kalman.h>
#include <cassert>
/* USER CODE END Includes */


/* Private typedef ----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/** @brief we do not use any custom types in Kingfisher (yet) */
/* USER CODE END PTD */


/* Private define -----------------------------------------------------------*/
/* USER CODE BEGIN PD */
/** @brief constants used throughout main Kingfisher code defined here */
// true sets subscale altitude target, false sets fullscale altitude target
#define SUBSCALE false

// Assertions (for debugging)
#if ENABLE_ASSERTS
  #define requires(condition) assert(condition)
  #define ensures(condition) assert(condition)
#else
  #define requires(condition) ((void)0)
  #define ensures(condition) ((void)0)
#endif

// ****************************** UNITS (in IPS) ******************************
#define SEA_LEVEL_PRESSURE_HPA 1013.25f
#define METERS_TO_FEET 3.28084f
#define ATMOSPHERE_FLUID_DENSITY 0.076474f // lbs/ft^3
#define GRAVITY 32.174f // ft/s^2

// ****************************** CONSTANTS ***********************************
// Average value from OpenRocket
#define ROCKET_DRAG_COEFFICIENT 0.46f

// The surface area (ft^2) of the rocket facing upwards
#define ROCKET_CROSS_SECTIONAL_AREA 0.0490873852f

// lbs in dry mass (with engine housing but NOT propellant, assuming no ballast)
#if SUBSCALE  
  #define ROCKET_MASS 11.28125f
#else
  #define ROCKET_MASS 16.5f
#endif

#define MAX_FLAP_SURFACE_AREA 0.0479010049f

// The maximum surface area (ft^2) of the rocket with flaps extended, 
// including rocket's area
#define ATS_MAX_SURFACE_AREA MAX_FLAP_SURFACE_AREA + ROCKET_CROSS_SECTIONAL_AREA

// Kalman filter parameters
#define NumStates 3
#define NumObservations 2
#define AltimeterNoise 1.0 // TODO: change
#define IMUNoise 1.0       // TODO: change

// Model covariance          (TODO: change)
#define m_p 0.1
#define m_s 0.1
#define m_a 0.8

//Engine/Flight Constants (in ms) - take from OpenRocket
//prevent ATS turn on until time is reached
#define DEF_motor_burnout_time_min 4000
//turn on ats when motor burnout is detected or cutoff_time is reached
#define DEF_motor_burnout_time_max 5000
//turn off ats when apogee is detected or cutoff_time is reached
#define DEF_cutoff_apogee_time 65000
//mark as landed when detected or cutoff_time is reached
#define DEF_cutoff_landing_time 300000

// ****************************** GLOBALS *************************************
// Whether the rocket is NOT running ATS, so don't try to mount servos, etc.
#define SKIP_ATS false 
#define ENABLE_ASSERTS true

// ****************************** DEBUGGING CHECK *****************************
//#define DEBUG false
#define DEBUG_C
/* USER CODE END PD */


/* Private macro ------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/** @brief Kingfisher does not have any macros, which are like calculated constants */
/* USER CODE END PM */

/* Private variables --------------------------------------------------------*/
/* USER CODE BEGIN PV */
//** @brief  */
// ****************************** FLIGHT PARAMETERS ***************************
// Whether to print debugging messages to the serial monitor
const bool DEBUG = true; 
const int LOOP_TARGET_MS = 30; // How frequently data should be collected (in milliseconds)

// Target altitude in feet
#if SUBSCALE
  const float ALT_TARGET = 3750.0f; // ft
#else
  const float ALT_TARGET = 4500.0f; // ft above launch pad
#endif
const float ACCEL_THRESHOLD = 3 * GRAVITY; // Acceleration threshold for launch detection (ft/s^2)
const float VELOCITY_THRESHOLD = 0.1f;     // Velocity threshold for landing detection (ft/s)

// ****************************** ATS SERVO PARAMETERS ************************
Servo m_atsServo;
float gATSPosition = 0.0f;
const int ATS_MIN = 180;
const int ATS_MAX = 13; // 254 constraint from flaps / 270 (servo max) * 180 (library function mapping)
const float ATS_IN = 0.0f;
const float ATS_OUT = 1.0f;

// memory parameters

// ****************************** SENSOR OBJECTS *****************
Adafruit_BMP3XX m_bmp;   // Altimeter
Adafruit_LSM6DSOX m_sox; // IMU
sensors_event_t accel, gyro, temp;

// ***************** MEASUREMENT VARIABLES *****************
String gBuffer; // Keeps track of data until it is written to the SD card
const int buffer_size = 50; // Number of measurements to take before writing to SD card
unsigned long gStartTime, gCurrTime, gTimer, gTimeDelta, gPrevLoopTime = 0; // Keeps track of time to make sure we are taking measurements at a consistent rate
float gAltFiltered, gVelocityFiltered, gAccelFiltered, gPredictedAltitude; // Filtered measurements shall be kept as global variables; raw data will be kept local to save memory

// Internal stuff for the Kalman Filter
float altitude_filtered_previous, acceleration_filtered_previous = 0.0f;
float gain_altitude, gain_acceleration, cov_altitude_current, cov_acceleration_current, cov_altitude_previous, cov_acceleration_previous = 0.0f;
float variance_altitude, variance_acceleration = 0.1f; // Might want to change these based on experiments or by calculating in flight
float previous_velocity_filtered = 0.0f; // We don't necessarily need this variable at this point, but it will be used when more advanced filtering techniques are implemented
bool gLaunched, gLanded; // Remembers if the rocket has launched and landed
unsigned long gLaunchTime;
float absolute_alt_target = ALT_TARGET;

// Kalman filter stuff
matrix* time_evolution_mat;
matrix* measurement_mat;
matrix* measurement_covariance_mat;
matrix* model_covariance_mat;

// ***************** PIN DEFINITIONS *****************
const int ATS_PIN = 6; //TODO
const int LED_PIN = LED_BUILTIN; //TODO
const int altimeter_chip_select = 10;     // TODO
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
/** @brief Initializes all devices, test devices and ATS */
void setup();

/** @brief Ensures data collected at same rate
  * Ensures each iteration in arduino main loop for loop runs at same rate */
void runTimer();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* user entry point */
void setup() {
    LEDSetup();

    // Setup serial terminal
    Serial.begin(115200); //Baud rate (bps)
    Serial.println("Initializing...");

    // Initialize SD card
    if (!initializeSDCard()) {
        Serial.println("SD card setup failed. Aborting.");
        LEDError();
    }

    // Determine File to write to
    determineWriteFile();

    // Initialize sensors
    if (!setupSensors()) {
        Serial.println("Sensor setup failed. Aborting.");
        LEDError();
    }
    
    // Test if all sensors on Altimeter works
    if (!testSensors()) {
        Serial.println("One or more altimeter sensors are not working");
        LEDError();
    }
    
    Serial.println("All sensors working!");
    testATS();

    // Initalize time evolution matrix
    double *time_evol_arr = calloc(9 * sizeof(double));
    time_evol_arr[0] = 1.0;
    time_evol_arr[4] = 1.0;
    time_evol_arr[8] = 1.0;
    time_evolution_mat = newMatrix(time_evol_arr, 3, 3);

    // measurement matrix (first row: altimeter, second row: accelerometer)
    double *measurement_arr = calloc(6 * sizeof(double));
    measurement_arr[0] = 1.0;
    measurement_arr[5] = 1.0;
    measurement_mat = newMatrix(measurement_arr, 2, 3);
    
    // measurement covariance matrix
    double *measurement_arr = calloc(4 * sizeof(double))
    KalmanFilter.R = {AltimeterNoise*AltimeterNoise, 0.0,
                      0.0,                           IMUNoise*IMUNoise};
    
    // model covariance matrix
    KalmanFilter.Q = {m_p*m_p, 0.0,     0.0,
                      0.0,     m_s*m_s, 0.0,
                      0.0,     0.0,     m_a*m_a};

    obs.Fill(0.0);
    measurement_state.Fill(0.0);

    Serial.println("Arduino is ready!");
    LEDSuccess();
    gStartTime = millis();
    gLaunchTime = gStartTime;

}

void loop() {
    gBuffer = "";
    for (int i = 0; i < buffer_size; i++) {
        runTimer(); // Ensures loop runs at a consistent rate

        // Get measurements from sensors and add to buffer
        gBuffer = gBuffer + getMeasurements() + "\n";

        // **** (PRE-FLIGHT) ****
        // Detect launch based on acceleration threshold
        if (gAccelFiltered > ACCEL_THRESHOLD && !gLaunched) {
            // Write CSV header to the file
            writeData("***************** START OF DATA ***************** TIME SINCE READY: " + String(millis() - gStartTime) + " ***************** TICK SPEED: " + String(LOOP_TARGET_MS) + "ms\n");
            writeData("time, pressure (hPa), altitude_raw (ft), acceleration_raw_x (ft/s^2), acceleration_raw_y, acceleration_raw_z, gyro_x (radians/s), gyro_y, gyro_z, gAltFiltered (ft), gVelocityFiltered (ft/s), gAccelFiltered (ft/s^2), temperature (from IMU; degrees C), gATSPosition (servo degrees), gAltPredicted (ft)\n");

            if (DEBUG) {Serial.println("Rocket has launched!");}
            gLaunched = true;
            gLaunchTime = millis();
            
            // Bring the ATS back online
            attachATS();
            setATSPosition(ATS_IN);

            // Set status LED
            LEDLogging();
        }

        // **** (DURING FLIGHT) ****
        // If the rocket has launched, adjust the ATS as necessary, and detect whether the rocket has landed
        if (gLaunched) {
            LEDFlying();
            adjustATS();
            if (detectLanding()) {
                gLanded = true;
            }
        }
        else {
            // If we are still on the pad, measure the altitude of the launch pad
            absolute_alt_target = ALT_TARGET + gAltFiltered;
        }
    }

    if (gLaunched) {
        writeData(gBuffer);
    }

    // **** (END OF FLIGHT) ****
    if (gLanded) {
        // End the program
        detachATS();
        if (DEBUG) {Serial.println("Rocket has landed, ending program");}
        while (true);
    }
}

void runTimer() {
    long tempTime = millis() - gPrevLoopTime;
    // Serial.println(tempTime);
    if (tempTime < LOOP_TARGET_MS) {
        delayMicroseconds((LOOP_TARGET_MS - tempTime) * 1000);
    }
    else if (tempTime > LOOP_TARGET_MS + 1) {
        Serial.println("Board is unable to keep up with target loop time of " + String(LOOP_TARGET_MS) + " ms (execution took "+ String(tempTime) + " ms)");
    }
    gCurrTime = millis();
    gTimer = gCurrTime - gStartTime;
    gTimeDelta = gCurrTime - gPrevLoopTime;
    // Serial.println(loop_time);
    gPrevLoopTime = gCurrTime;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  // Repeats indefinitely after setup() is finished
  /** @brief Collect/log data and control ATS */
  while (1)
  {
    gBuffer = "";
    for (int i = 0; i < buffer_size; i++) {
      runTimer(); // Ensures loop runs at a consistent rate

      // Get measurements from sensors and add to buffer
      gBuffer = gBuffer + getMeasurements() + "\n";

      // **** (PRE-FLIGHT) ****
      // Detect launch based on acceleration threshold
      if (gAccelFiltered > ACCEL_THRESHOLD && !gLaunched) {
        // Write CSV header to the file
        writeData("***************** START OF DATA ***************** TIME SINCE READY: " + String(millis() - gStartTime) + " ***************** TICK SPEED: " + String(LOOP_TARGET_MS) + "ms\n");
        writeData("time, pressure (hPa), altitude_raw (ft), acceleration_raw_x (ft/s^2), acceleration_raw_y, acceleration_raw_z, gyro_x (radians/s), gyro_y, gyro_z, gAltFiltered (ft), gVelocityFiltered (ft/s), gAccelFiltered (ft/s^2), temperature (from IMU; degrees C), gATSPosition (servo degrees), gAltPredicted (ft)\n");

        if (DEBUG) {Serial.println("Rocket has launched!");}
        gLaunched = true;
        gLaunchTime = millis();
        
        // Bring the ATS back online
        attachATS();
        setATSPosition(ATS_IN);

        // Set status LED
        LEDLogging();
      }

        // **** (DURING FLIGHT) ****
        // If the rocket has launched, adjust the ATS as necessary, and detect whether the rocket has landed
        if (gLaunched) {
          LEDFlying();
          adjustATS();
          if (detectLanding()) {
              gLanded = true;
          }
        }
        else {
          // If we are still on the pad, measure the altitude of the launch pad
          absolute_alt_target = ALT_TARGET + gAltFiltered;
        }
    }

    if (gLaunched) {
      writeData(gBuffer);
    }

    // **** (END OF FLIGHT) ****
    if (gLanded) {
      // End the program
      detachATS();
      if (DEBUG) {Serial.println("Rocket has landed, ending program");}
      while (true);
    }
    /* USER CODE END WHILE */
    
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
