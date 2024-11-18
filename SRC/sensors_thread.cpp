/* File for the sensors' thread function definitions */

// LIBRARIES ------------------------------------------------------------------------------------
#include "mbed.h"
#include "sensors_thread.h"
#include "mma8451.h"
#include "si7021.h"
#include "tcs34725.h"
#include "soilmoisture.h"
#include "phototrans.h"
#include "message_q.h"

// EXTERN VARIABLES --------------------------------------------------------------------
extern bool TEST_MODE_SAMPLING_FLAG;
extern bool NORMAL_MODE_SAMPLING_FLAG;

//STATIC VARIABLES -----------------------------------------------------------------------------
static float ax, ay, az;                                            // Variables to store the accelerations
static float moistPercAnalogValue;                                  // Variable to store the normalized analog value of the sensor (0 - 1)
static float lightPercAnalogValue;
static uint16_t clear, red, green, blue;
static float temperature;
static float humidity;

// ISR FLAGS ------------------------------------------------------------------------------------
volatile bool tap_detected = false;                          // Flag to indicate tap event
volatile bool freefall_detected = false;                     // Flag to indicate freefall event

// CONSTRUCTORS ---------------------------------------------------------------------------------
I2C i2c(SDA_PIN, SCL_PIN);                                   // I2C communication
DigitalOut whiteLED(LED_PIN);                                // DigitalOut for builtin white LED control
AnalogIn moistureIn(MOISTURE_PIN);                           // Analog pin corresponding to Arduino's A0
AnalogIn lightIn(PHTRANS_PIN);                               // Analog pin corresponding to Arduino's A2
InterruptIn int1_pin(INT_PIN_PULSE);                         // Interruption for tap/pulse detection
InterruptIn int2_pin(INT_PIN_FF);                            // Interruption for freefall detection

// FREEFALL ISR =================================================================================
void tap_ISR() {
    tap_detected = true;
}

void freefall_ISR(){
    freefall_detected = true;
}

// ==============================================================================================
// SENSORS MAIN FUNCTION
// ==============================================================================================
void sensor_th_routine(){
    // THREAD SETUP -----------------------------------------------------------------------------
    // Accelerometer MMA8451 initialization
    init_mma8451_pulse_ff();
    // ATTACH BOTH INTERRUPTIONS ---------------------------------------------------------------------------
    int1_pin.fall(&tap_ISR);
    int2_pin.fall(&freefall_ISR);
    // Colour sensor TCS34725 initialization
    tcs34725_init();                                         // Initialize the TCS34725 sensor
    // THREAD SETUP END -------------------------------------------------------------------------

    // THREAD LOOP ------------------------------------------------------------------------------
    while(true){                                             // While true so it does update as expected
        // Accelometer MMA8451 measurements -----------------------------------------------------
        read_accelerations(&ax, &ay, &az);                   // Read the acceleration values for each axis
        
        // Ambient sensor Si7021 measurements ---------------------------------------------------
        humidity = read_humidity();
        temperature = read_temperature();

        // Soil moisture measurements -----------------------------------------------------------
        moistPercAnalogValue = moistureIn.read() * 100;      // Read the normalized analog value of soil moisture
        
        // Ambient light measurements -----------------------------------------------------------
        lightPercAnalogValue = lightIn.read() * 100;         // Read the normalized analog value of ambient light
        
        // Colour sensor TCS34725 measurements --------------------------------------------------
        whiteLED = 1;                                        // Turn on the white LED before taking a measurement
        ThisThread::sleep_for(30ms);                         // Wait for the integration time (24ms) + small extra time for stable readings

        // Read Clear, Red, Green, and Blue channels
        clear = read_register_16(TCS34725_CDATAL);
        red   = read_register_16(TCS34725_RDATAL);
        green = read_register_16(TCS34725_GDATAL);
        blue  = read_register_16(TCS34725_BDATAL);

        whiteLED = 0;                                        // Turn off the white LED after the measurement

        //sending message to the main thread
        send_sensors_message_through_main_thread(ax, ay, az, moistPercAnalogValue, lightPercAnalogValue, clear, red, green, blue, temperature, humidity);

        if(TEST_MODE_SAMPLING_FLAG){
            ThisThread::sleep_for(TEST_MODE_SENSOR_THREAD_SLEEP);        // Wait 2 seconds until the next measurement if in TEST_MODE
        }else if(NORMAL_MODE_SAMPLING_FLAG){
            ThisThread::sleep_for(NORMAL_MODE_SENSOR_THREAD_SLEEP);      // Wait 10 seconds until the next measurement if in NORMAL_MODE
        }
    }
    // THREAD LOOP END --------------------------------------------------------------------------
}
// SENSORS MAIN FUNCTION END ====================================================================