/* File for the message queues function declarations and macros */

// LIBRARIES ------------------------------------------------------------------------------------
#include "mbed.h"

// LIBRARY GUARD --------------------------------------------------------------------------------
#ifndef MESSAGE_Q_H
#define MESSAGE_Q_H

// MACROS ---------------------------------------------------------------------------------------
#define MESSAGE_QUEUE_MAX_LENGTH 16

// ==============================================================================================
// MESSAGE STRUCTS definition (format of messages between task)
// ==============================================================================================
typedef struct {
    float ax, ay, az;                                            // Variables to store the accelerations
    float moistPercAnalogValue;                                  // Variable to store the normalized analog value of the sensor (0 - 1)
    float lightPercAnalogValue;                                  // Light percentage
    uint16_t clear, red, green, blue;                            // Raw measures of color channels
    float temperature, humidity;                                 // Temperature in celsius and %RH
} message_t_sensors;

typedef struct{
    int fix_status;                                              // Fix status
    int gps_hour, gps_minute;                                    // Time
    float gps_seconds;                                           // Seconds
    float latitude , longitude, altitude;                        // Position
}message_t_gps;
// MESSAGE STRUCTS ==============================================================================

// ==============================================================================================
// PROTOTYPES
// ==============================================================================================
extern void send_sensors_message_through_main_thread(float ax,float ay,float az,float moistPercAnalogValue, float lightPercAnalogValue, uint16_t c, uint16_t r, uint16_t g, uint16_t b, float temperature, float humidity);
extern void receive_info_from_sensors(float * ax,float * ay,float * az,float * moistPercAnalogValue, float * lightPercAnalogValue, uint16_t * c, uint16_t * r, uint16_t * g, uint16_t * b, float * temperature, float * humidity);
extern void send_GPS_message_through_main_thread(uint8_t fix_status,  uint8_t gps_hour, uint8_t gps_minute, float gps_seconds, float latitude, float longitude,  float altitude);
extern void receive_info_from_GPS(uint8_t * fix_status,  uint8_t * gps_hour, uint8_t * gps_minute, float * gps_seconds, float * latitude, float * longitude,  float * altitude);
// PROTOTYPES END ===============================================================================

#endif