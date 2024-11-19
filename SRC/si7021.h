/* File for the ambient sensor Si7021 function declarations and macros */

// LIBRARIES ------------------------------------------------------------------------------------
#include "mbed.h"

// LIBRARY GUARD --------------------------------------------------------------------------------
#ifndef SI7021_H
#define SI7021_H

// Si7021 MACROS --------------------------------------------------------------------------------
#define SI7021_ADDR 0x40 << 1                               // Si7021 I2C Address: 7-bit I2C address SHIFTED BY 1 BIT
#define CMD_MEASURE_HUMIDITY 0xE5                           // Si7021 Command: Measure Relative Humidity, Hold Master Mode
#define CMD_MEASURE_TEMP 0xE3                               // Si7021 Command: Measure Temperature, Hold Master Mode

// PROTOTYPES ===================================================================================
float read_humidity();
float read_temperature();
// PROTOTYPES END ===============================================================================

#endif