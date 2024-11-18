/* File for the GPS' thread function declarations and macros */

// LIBRARIES ------------------------------------------------------------------------------------
#include "mbed.h"

// LIBRARY GUARD --------------------------------------------------------------------------------
#ifndef GPS_THREAD_H
#define GPS_THREAD_H

// ==============================================================================================
// MACROS
// ==============================================================================================
// Thread macros
#define GPS_THREAD_SLEEP   500ms              // GPS measuring every 500 ms

// UART macros
#define GPS_TX           PA_9
#define GPS_RX           PA_10
#define GPS_BAUD_RATE    9600
#define GPS_BUFFER_SIZE  256

// Commands to send (RX)
#define ENABLE_STATUS_ANTENA "$PGCMD,33,1*6C\r\n"
#define SET_UPDATING_NMEA_1HZ_RATE "$PMTK300,1000,0,0,0,0*2C\r\n"
#define SET_SAMPLE_1HZ "$PMTK220,1000*1F\r\n"
// MACROS END ===================================================================================

// ==============================================================================================
// PROTOTYPES
// ==============================================================================================
extern bool parse_GPS_data(char *nmea_data);
extern void read_GPS();
extern void gps_th_routine();
// PROTOTYPES END ===============================================================================

#endif