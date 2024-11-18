/* File for the sensors' thread function declarations and macros */

// LIBRARIES ------------------------------------------------------------------------------------
#include "mbed.h"

// LIBRARY GUARD --------------------------------------------------------------------------------
#ifndef SENSORS_THREAD_H
#define SENSORS_THREAD_H

// ==============================================================================================
// MACROS
// ==============================================================================================
// Thread macros
#define TEST_MODE_SENSOR_THREAD_SLEEP   2000ms                    // Sensor measuring every 2 seconds - TEST_MODE
#define NORMAL_MODE_SENSOR_THREAD_SLEEP 30000ms                   // Sensor measuring every 30 seconds - NORMAL_MODE

// I2C macros
#define SDA_PIN PB_9
#define SCL_PIN PB_8
// MACROS END ===================================================================================

// ==============================================================================================
// PROTOTYPES
// ==============================================================================================
extern void sensor_th_routine();
// PROTOTYPES END ===============================================================================

#endif