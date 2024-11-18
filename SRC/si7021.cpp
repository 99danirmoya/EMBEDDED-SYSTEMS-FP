/* File for the ambient sensor Si7021 function definitions */

// LIBRARIES ---------------------------------------------------------------------------------------------------------------
#include "mbed.h"
#include "sensors_thread.h"
#include "si7021.h"

// GLOBAL VARIABLES --------------------------------------------------------------------------------------------------------
static float temperature, humidity;

// CONSTRUCTOR -------------------------------------------------------------------------------------------------------------
extern I2C i2c;

// FUNCTION TO READ 16-BIT DATA FROM SENSOR Si7021 =========================================================================
uint16_t read_register_si7021(char command) {
    char data[2];                                           // Data buffer of 16-bit size
    i2c.write(SI7021_ADDR, &command, 1);                    // Send command to start measurement
    i2c.read(SI7021_ADDR, data, 2);                         // Read the data after waiting

    return (data[0] << 8) | data[1];                        // Combine the two bytes (0 is the value if the reading is not successful)
}

// FUNCTION TO READ %RH (HUMIDITY) =======================================================================================
float read_humidity(){
    uint16_t raw_humidity = read_register_si7021(CMD_MEASURE_HUMIDITY);
    
    humidity = ((125.0 * raw_humidity) / 65536.0) - 6.0;    // As noted in the datasheet, convert raw humidity to percentage

    return humidity;   
}

// FUNCTION TO READ TEMPERATURE AND CONVERT IT TO CELSIUS ================================================================
float read_temperature(){
    uint16_t raw_temperature = read_register_si7021(CMD_MEASURE_TEMP);

    temperature = ((175.72 * raw_temperature) / 65536.0) - 46.85;  // As noted in the datasheet, convert raw temperature to Celsius  

    return temperature;                               
}