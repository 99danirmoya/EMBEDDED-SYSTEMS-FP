/* EMBEDDED SYSTEMS FINAL PROJECT - ALVARO RODRÍGUEZ PIÑEIRO AND DANIEL RODRÍGUEZ MOYA
- File for the MAIN thread function definitions.

- This project consists on a plant health monitoring station with various sensors.
  Those sensors are implemented in our self-crafted libraries and executed in differnt
  threads to better handle events, like interrumptions or blocking timers, if the project
  gets expanded.

- The implemented ADVANCED_MODE consists in adding interruptions for the accelerometer 
  freefall and tap detection */

// LIBRARIES ----------------------------------------------------------------------------------
#include "mbed.h"
#include "sensors_thread.h"
#include "gps_thread.h"
#include "message_q.h"

// MACROS -------------------------------------------------------------------------------------
#define MAIN_THREAD_FREQ   100ms                                                 // Main thread sampling frequency
#define FREEFALL_BLINK     500ms                                                 // Timer to blink LED4 in case of frefall detection
#define TEST_TICKER_FREQ   2000ms                                                // Ticker timer to print measurements in TEST_MODE
#define NORMAL_TICKER_FREQ 30000ms                                               // Ticker timer to print measurements in NORMAL_MODE
#define STATS_TICKER_FREQ  3600000ms                                             // Ticker timer to print stats in NORMAL_MODE
#define LED1_PIN           PB_5                                                  // Pin internally connected to LED1
#define LED3_PIN           PB_6                                                  // Pin internally connected to LED3
#define LED4_PIN           PB_7                                                  // Pin internally connected to LED4
#define RGB_RED_PIN        PH_0                                                  // Pin connected to the RGB red
#define RGB_GREEN_PIN      PA_14                                                 // Pin connected to the RGB green
#define RGB_BLUE_PIN       PA_13                                                 // Pin connected to the RGB blue
#define BUTTON_PIN         PB_2                                                  // Pin internally connected to USER BUTTON

// CONSTRUCTORS -------------------------------------------------------------------------------
Ticker test_ticker;                                                              // Constructor for ticker TEST_MODE
Ticker normal_ticker;                                                            // Constructor for ticker NORMAL_MODE
Ticker stats_ticker;                                                             // Constructor for printing stats in NORMAL_MODE

// THREADS ------------------------------------------------------------------------------------
static Thread sensors_th(osPriorityNormal, 512);                                 // Thread for the measurements of ANALOGIC and I2C sensors
static Thread gps_th(osPriorityHigh, 1024);                                      // Thread for the measurements of the GPS

// I/O INITIALIZATION -------------------------------------------------------------------------
BusOut myLED(LED1_PIN, LED3_PIN, LED4_PIN);                                      // Control of the built-in LEDs
BusOut myRGB(RGB_RED_PIN, RGB_GREEN_PIN, RGB_BLUE_PIN);                          // BusOut to control the RGB LED with just an object
InterruptIn button(BUTTON_PIN);                                                  // USER button to swipe between modes by means of an interrupt

// ENUMERATION OF MODES -----------------------------------------------------------------------
enum Mode{TEST_MODE, NORMAL_MODE, ADVANCED_MODE};                                // Sequential state machine modes
volatile Mode current_mode = TEST_MODE;                                          // Mode to store the current mode

// GLOBAL VARIABLES ---------------------------------------------------------------------------
bool TEST_MODE_SAMPLING_FLAG = true;                                             // Initially, we are in TEST_MODE
bool NORMAL_MODE_SAMPLING_FLAG = false;
bool T_INVALID = false;
bool RH_INVALID = false;

// EXTERN VARIABLES ---------------------------------------------------------------------------
extern volatile bool tap_detected;                                               // Flag that is received from the sensors' thread when MMA8451Q interruptions take place
extern volatile bool freefall_detected;

// FLAGS --------------------------------------------------------------------------------------
volatile bool test_tick_event = false;                                           // Flag to display measurements in TEST_MODE
volatile bool normal_tick_event = false;                                         // Flag to display measurements in NORMAL_MODE
volatile bool stats_tick_event = false;                                          // Flag for stats calculation in NORMAL_MODE
volatile bool mode_change_flag = false;                                          // Flag to be set at mode change

// STATS VARIABLES ----------------------------------------------------------------------------
float humidity_min = 100.0, humidity_max = 0.0, humidity_sum = 0.0;              // Variables to store stats of Si7021 %RH. FOR ALL VARIABLES, THESE INITIALIZATIONS HAVE BEEN DONE LIKE THIS TO WORK WITH 'fmin()' and 'fmax()' FUNCTIONS
float temperature_min = 100.0, temperature_max = -100.0, temperature_sum = 0.0;  // Variables to store stats of Si7021 T
float moist_min = 100.0, moist_max = 0.0, moist_sum = 0.0;                       // Variables to store stats of soil moisture sensor
float light_min = 100.0, light_max = 0.0, light_sum = 0.0;                       // Variables to store stats of phototransistor
float ax_min = 100.0, ax_max = -100.0;                                           // Variables to store stats of x-axis
float ay_min = 100.0, ay_max = -100.0;                                           // Variables to store stats of y-axis
float az_min = 100.0, az_max = -100.0;                                           // Variables to store stats of z-axis
uint8_t tap_count = 0;                                                           // Counter for the amount of taps on the accelerometer
uint8_t sample_count = 0;                                                        // Counter to computate stats after the performed amount of measurements
uint8_t humidity_count = 0, temperature_count = 0;                               // Valid temperature and relative humidity samples count
uint8_t red_count = 0, green_count = 0, blue_count = 0;                          // Color dominance counters

// STATIC VARIABLES (SENSORS AND GPS QUEUE MESSAGES) ------------------------------------------
static float humidity, temperature;
static float moistPercAnalogValue;
static float lightPercAnalogValue;
static float ax, ay, az;
static uint16_t clear, red, green, blue;
static uint8_t fix_status, gps_hour, gps_minute;
static float gps_seconds, latitude, longitude, altitude;

// FUNCTION PROTOTYPES ------------------------------------------------------------------------
static void startAllThreads();
static void set_mode_change_flag();
static void next_mode();
static void printSensorsInfo();
static void resetStats();
static void printStats();                                                        // REMEMBER THIS FUNCTION IS TO CALCULATE STATS FOR THE REQUIRED SENSORS, NOT ALL OF THEM

// ============================================================================================
// INTERRUPTION SUBROUTINES
// ============================================================================================
// TICKER ISR TEST MODE
static void test_ticker_ISR(){                                                   // ISR to update measure messages every 2 seconds
    test_tick_event = true;
}

// TICKER ISR NORMAL MODE
static void normal_ticker_ISR(){                                                 // ISR to update measure messages every 30 seconds
    normal_tick_event = true;
}

// TICKER ISR STATS IN NORMAL MODE
static void stats_ticker_ISR(){
    stats_tick_event = true;
}

// BUTTON PRESS ISR
static void button_press_ISR(){
    mode_change_flag = true;                                                     // Set flag to change mode
}
// INTERRUPTION SUBROUTINES END ===============================================================

// ============================================================================================
// MAIN
// ============================================================================================
int main(){
    // SETUP ==================================================================================
    // ISR callbacks
    test_ticker.attach(&test_ticker_ISR, TEST_TICKER_FREQ);                      // Attach function to tick event for TEST_MODE as it is the initial
    button.fall(&button_press_ISR);                                              // Set flag on button press (falling edge)

    // Setup conditions
    startAllThreads();                                                           // Launch of all tasks
    myLED = 0b001;                                                               // Initial condition is to switch on LED1 for TEST_MODE
    myRGB = 0b111;                                                               // Ensure RGB LED is OFF
    // SETUP END ==============================================================================

    // LOOP ===================================================================================
    while(true){
        // If a USER BUTTON interruption takes place, mode change is toggled
        if(mode_change_flag){
            mode_change_flag = false;
            next_mode();
        }

        // PULLING MESSAGES FROM MESSAGES QUEUES IF EXISTS
        receive_info_from_sensors(&ax, &ay, &az, &moistPercAnalogValue, &lightPercAnalogValue, &clear, &red, &green, &blue, &temperature, &humidity);
        receive_info_from_GPS(&fix_status,  &gps_hour, &gps_minute, &gps_seconds, &latitude, &longitude, &altitude);

        // TEST_MODE --------------------------------------------------------------------------
        if(current_mode == TEST_MODE){                                           // Check if we are in TEST MODE            
            if(tap_detected){
                tap_count++;
                tap_detected = false;   
            }

            if(test_tick_event){                                                 // Check for ticker event
                printf("--------------------------------\n\r");
                printf("TEST MODE (Period: 2s)\n\r");
                printSensorsInfo();                                              // Print sensor information
                
                tap_count = 0;                                                   // Reset the tap counter after printing measurements

                // Determine and print the most dominant color, turn on its RGB counterpart
                if(red > green && red > blue){
                    printf("Dominant Color: Red\n\r");
                    myRGB = 0b110;
                }else if(green > red && green > blue){
                    printf("Dominant Color: Green\n\r");
                    myRGB = 0b101;
                }else if(blue > red && blue > green){
                    printf("Dominant Color: Blue\n\r");
                    myRGB = 0b011;
                }else{
                    printf("Dominant Color: No clear dominant color\n\r");
                    myRGB = 0b111;
                }
                
                test_tick_event = false;                                         // Reset tick_event flag
            }
        }
        // NORMAL_MODE ------------------------------------------------------------------------
        else if(current_mode == NORMAL_MODE){            
            if(tap_detected){
                tap_count++;
                tap_detected = false;   
            }

            if(normal_tick_event){       
                printf("--------------------------------\n\r");
                printf("NORMAL MODE (Period: 30s)\n\r");                         // Check for ticker event
                printSensorsInfo();                                              // Print sensor information
                
                tap_count = 0;                                                   // Reset the tap counter after printing measurements

                // Turn on RGB if a measurement is out of valid range
                if(T_INVALID){
                    myRGB = 0b110;                                               // Set the RGB to RED if T is out of valid range
                }
                if(RH_INVALID){
                    myRGB = 0b011;                                               // Set the RGB to BLUE if %RH is out of valid range
                }
                if(!T_INVALID && !RH_INVALID){
                    myRGB = 0b111;                                               // If none is true, RGB OFF
                }

                T_INVALID = false;                                               // Set INVALID flags to false
                RH_INVALID = false;

                // Update humidity stats if within valid range
                if(humidity >= 25.0 && humidity <= 75.0) {
                    humidity_sum += humidity;
                    humidity_min = fmin(humidity_min, humidity);                 // Function that returns the smaller of two floating-point numbers
                    humidity_max = fmax(humidity_max, humidity);                 // Function that returns the larger of two floating-point numbers.
                    humidity_count++;
                }

                // Update temperature stats if within valid range
                if(temperature >= -10.0 && temperature <= 50.0) {
                    temperature_sum += temperature;
                    temperature_min = fmin(temperature_min, temperature);
                    temperature_max = fmax(temperature_max, temperature);
                    temperature_count++;
                }

                // Always update soil moisture and ambient light stats
                moist_sum += moistPercAnalogValue;
                moist_min = fmin(moist_min, moistPercAnalogValue);
                moist_max = fmax(moist_max, moistPercAnalogValue);
                light_sum += lightPercAnalogValue;
                light_min = fmin(light_min, lightPercAnalogValue);
                light_max = fmax(light_max, lightPercAnalogValue);

                // Update min and max acceleration for each axis
                ax_min = fmin(ax_min, ax);
                ax_max = fmax(ax_max, ax);
                ay_min = fmin(ay_min, ay);
                ay_max = fmax(ay_max, ay);
                az_min = fmin(az_min, az);
                az_max = fmax(az_max, az);

                sample_count++;

                // Determine and count the dominant color
                if(red > green && red > blue){
                    red_count++;
                }else if(green > red && green > blue){
                    green_count++;
                }else if(blue > red && blue > green){
                    blue_count++;
                }

                normal_tick_event = false;                                       // Reset tick_event flag
            }

            if(stats_tick_event){
                printStats();
                stats_tick_event = false;
            }
        }

        // ADVANCED MODE ----------------------------------------------------------------------
        else{
            if(freefall_detected){
                printf("Freefall detected on Z-axis. SYSTEM SHUT DOWN!\n");
                printf("================================\n\r");
                
                // System switch OFF conditions
                button.fall(nullptr);                                            // Detaches the interrupt on the falling edge
                sensors_th.terminate();                                          // Inmediately stop the sensors' thread
                gps_th.terminate();
                
                // System switch OFF blinking indicator
                while(true){
                    myLED = myLED | 0b100;                                       // Set to 1 LED4 and 0 to the others
                    ThisThread::sleep_for(FREEFALL_BLINK);

                    myLED = myLED & ~0b100;                                      // Set the complement of the 3 bit, LED4, to 0 while leaving the others unchanged, still 0
                    ThisThread::sleep_for(FREEFALL_BLINK);
                }
            }
        }

        ThisThread::sleep_for(MAIN_THREAD_FREQ);                                 // Main thread frquency set to 100ms, not 2000ms, as other functions could be implemented in parallel and be compromised by such slow speed
    }
    // LOOP END ===============================================================================
}
// MAIN END ===================================================================================

// ============================================================================================
// CUSTOM FUNCTIONS
// ============================================================================================
// FUNCTION TO START ALL THREADS --------------------------------------------------------------
static void startAllThreads(){    
    sensors_th.start(sensor_th_routine);
    gps_th.start(gps_th_routine);    
}

// FUNCTION TO SWITCH TO THE NEXT MODE --------------------------------------------------------
static void next_mode(){
    // Detatch tickers to reset timing
    test_ticker.detach();
    normal_ticker.detach();
    stats_ticker.detach();

    // Reset ticker flags
    test_tick_event = false;
    normal_tick_event = false;
    stats_tick_event = false;

    // Switch OFF RGB
    myRGB = 0b111;

    if(current_mode == TEST_MODE){
        current_mode = NORMAL_MODE;
        myLED = 0b010;                                                           // Turn on LED3 for NORMAL_MODE

        // Set threads sampling frequency to NORMAL_MODE (30 seconds)
        TEST_MODE_SAMPLING_FLAG = false;
        NORMAL_MODE_SAMPLING_FLAG = true;

        // Attach NORMAL_MODE tickers
        normal_ticker.attach(&normal_ticker_ISR, NORMAL_TICKER_FREQ);
        stats_ticker.attach(&stats_ticker_ISR, STATS_TICKER_FREQ);

    }else if(current_mode == NORMAL_MODE){
        current_mode = ADVANCED_MODE;
        myLED = 0b100;                                                           // Turn on LED4 for ADVANCED_MODE

        resetStats();                                                            // Reset stats when exiting NORMAL_MODE to avoid stale data
        printf("--------------------------------\n\r");
        printf("ADVANCED MODE (FREEFALL DETECTION)\n\r");
        printf("--------------------------------\n\r");

    }else{
        current_mode = TEST_MODE;                                                // Go back to TEST_MODE after pressing the button from ADVANCED_MODE
        myLED = 0b001;                                                           // Turn on LED1 for TEST_MODE        
        
        // Set threads sampling frequency to TEST_MODE (2 seconds)
        TEST_MODE_SAMPLING_FLAG = true;
        NORMAL_MODE_SAMPLING_FLAG = false;
        
        // Attach ticker for TEST_MODE
        test_ticker.attach(&test_ticker_ISR, TEST_TICKER_FREQ);
    }
}

// FUNCTION TO PRINT SENSORS MEASUREMENTS -----------------------------------------------------
static void printSensorsInfo(){
    printf("--------------------------------\n\r");
    // TCS34725 measurements
    printf("C = %u, R = %u, G = %u, B = %u\n\r", clear, red, green, blue);

    // MMA8451Q measurements
    printf("ax = %.2f G, ay = %.2f G, az = %.2f G\n\r", ax, ay, az);
    
    // Si7021 measurements
    if(temperature > -10 && temperature < 50){
        printf("T = %.1f celsius, ", temperature);
    }else{
        printf("Temperature out of valid range! ");
        T_INVALID = true;
    }
    if(humidity > 25 && humidity < 75){
        printf("RH = %.1f %%\n\r", humidity);
    }else{
        printf("Relative humidity out of valid range!\n\r");
        RH_INVALID = true;
    }

    // Analogic sensors measurements
    printf("Soil moisture = %.1f %%\n\r", moistPercAnalogValue);
    printf("Ambient light = %.1f %%\n\r", lightPercAnalogValue);

    // GPS measurements
    if(fix_status > 0 && fix_status <= 2){                                       // Print values only if there is a valid fix. ONLY 1 AND 2 ARE VALID
        printf("Fix Status = %d, Time (UTC): %02d:%02d:%.1f, Alt = %.2f m, Lat = %.6f deg, Lon = %.6f deg\n\r", fix_status, gps_hour, gps_minute, gps_seconds, altitude, latitude, longitude);
    }else{
        printf("No GPS fix yet, please wait for signal...\n\r");
    }

    // Taps counted
    printf("Total Taps: %d\n\r", tap_count);
}

// FUNCTION TO RESET STATS VARIABLES ----------------------------------------------------------
static void resetStats(){
    // Reset stats for next period - Si7021
    humidity_min = 100.0; humidity_max = 0.0; humidity_sum = 0.0;
    temperature_min = 100.0; temperature_max = -100.0; temperature_sum = 0.0;
    
    // Reset stats for next period - Analogic sensors
    moist_min = 100.0; moist_max = 0.0; moist_sum = 0.0;
    light_min = 100.0; light_max = 0.0; light_sum = 0.0;
    
    // Reset stats for next period - MMA8451Q
    ax_min = 100.0; ax_max = -100.0;                                             // Reset acceleration min/max values
    ay_min = 100.0; ay_max = -100.0;
    az_min = 100.0; az_max = -100.0;
    
    // Reset counters for next period
    sample_count = 0;
    humidity_count = 0; temperature_count = 0;                                   // Reset Si7021 counters
    red_count = 0; green_count = 0; blue_count = 0;                              // Reset color counters
}

// FUNCTION TO CALCULATE STATS FOR Si7021 AND ANALOGIC SENSORS --------------------------------
static void printStats(){
    printf("--------------------------------\n\r");
    printf("ONE HOUR STATS:\n\r");
    printf("--------------------------------\n\r");
    // Only print Si7021 relative humidity stats if we have valid samples
    if(humidity_count > 0){
        printf("RHmin = %.1f %%, RHmax = %.1f %%, RHavg = %.1f %%\n\r", humidity_min, humidity_max, humidity_sum / humidity_count);
    }else{
        printf("No valid data for relative humidity\n\r");
    }

    // Only print Si7021 temperature stats if we have valid samples
    if(temperature_count > 0){
        printf("Tmin = %.1f celsius, Tmax = %.1f celsius, Tavg = %.1f celsius\n\r", temperature_min, temperature_max, temperature_sum / temperature_count);
    }else{
        printf("No valid data for temperature\n\r");
    }

    printf("SMmin = %.1f %%, SMmax = %.1f %%, SMavg = %.1f %%\n\r", moist_min, moist_max, moist_sum / sample_count);
    printf("ALmin = %.1f %%, ALmax = %.1f %%, ALavg = %.1f %%\n\r", light_min, light_max, light_sum / sample_count);

    // Print min and max acceleration for each axis
    printf("axmin = %.2f G, axmax = %.2f G\n\r", ax_min, ax_max);
    printf("aymin = %.2f G, aymax = %.2f G\n\r", ay_min, ay_max);
    printf("azmin = %.2f G, azmax = %.2f G\n\r", az_min, az_max);

    // Determine and print the most dominant color
    if(red_count > green_count && red_count > blue_count){
        printf("Dominant Color: Red\n\r");
    }else if(green_count > red_count && green_count > blue_count){
        printf("Dominant Color: Green\n\r");
    }else if(blue_count > red_count && blue_count > green_count){
        printf("Dominant Color: Blue\n\r");
    }else{
        printf("Dominant Color: No clear dominant color\n\r");
    }

    resetStats();
}
// CUSTOM FUNCTIONS END =======================================================================