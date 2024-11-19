/* File for the message queues function definitions */

// LIBRARIES -----------------------------------------------------------------------------------------------------
#include "mbed.h"
#include "message_q.h"

// STATIC DEFINITIONS --------------------------------------------------------------------------------------------
static MemoryPool<message_t_sensors, MESSAGE_QUEUE_MAX_LENGTH> mpool_sensors;
static MemoryPool<message_t_gps, MESSAGE_QUEUE_MAX_LENGTH> mpool_gps;

static Queue<message_t_sensors, MESSAGE_QUEUE_MAX_LENGTH> queue_sensors;
static Queue<message_t_gps, MESSAGE_QUEUE_MAX_LENGTH> queue_gps;

// GLOBAL FUNCTIONS ----------------------------------------------------------------------------------------------
void send_sensors_message_through_main_thread(float ax,float ay,float az,float moistPercAnalogValue, float lightPercAnalogValue, uint16_t c, uint16_t r, uint16_t g, uint16_t b, float temperature, float humidity){
    //this functions use a message inside the message pool to send a message to another task
    //in our app the comuser thread will be the main thread (thread in charge of printing info through the terminal).

    //booking dinamic space for the new message
    message_t_sensors *new_message = mpool_sensors.alloc();

    //filling sensor message structure
    new_message->ax = ax;
    new_message->ay = ay;
    new_message->az = az;

    new_message->moistPercAnalogValue = moistPercAnalogValue;

    new_message->lightPercAnalogValue = lightPercAnalogValue;

    new_message->clear = c;
    new_message->red = r;
    new_message->green = g;
    new_message->blue = b;

    new_message->temperature = temperature;
    new_message->humidity = humidity;    

    //sending message to the queue
    queue_sensors.put(new_message);
}

void send_GPS_message_through_main_thread(uint8_t fix_status,  uint8_t gps_hour, uint8_t gps_minute, float gps_seconds, float latitude, float longitude,  float altitude){
    //this functions use a message inside the message pool to send a message to another task
    //in our app the comuser thread will be the main thread (thread in charge of printing info through the terminal).

    //booking dinamic space for the new message
    message_t_gps *new_message = mpool_gps.alloc();

    //filling sensor message structure
    new_message->fix_status = fix_status;

    new_message->gps_hour = gps_hour;
    new_message->gps_minute = gps_minute;
    new_message->gps_seconds = gps_seconds;

    new_message->latitude = latitude;
    new_message->longitude = longitude;
    new_message->altitude = altitude;    

    //sending message to the queue
    queue_gps.put(new_message);
}

void receive_info_from_sensors(float * ax,float * ay,float * az,float * moistPercAnalogValue, float * lightPercAnalogValue, uint16_t * c, uint16_t * r, uint16_t * g, uint16_t * b, float * temperature, float * humidity){
    //feching older message in the queue
    osEvent evt = queue_sensors.get(0);

    //checking that we have received a message from the queue
    if (evt.status == osEventMessage) {
        //extracting the data from the message
        message_t_sensors *message_received = (message_t_sensors*)evt.value.p;

        //extracting each field of the sturcture
        *ax = message_received->ax;
        *ay = message_received->ay;
        *az = message_received->az; 

        *moistPercAnalogValue = message_received->moistPercAnalogValue;

        *lightPercAnalogValue = message_received->lightPercAnalogValue;

        *c = message_received->clear;
        *r = message_received->red;
        *g = message_received->green;
        *b= message_received->blue;

        *temperature = message_received->temperature;
        *humidity = message_received->humidity;
        

        //releasing allocated memory from the message pool
        mpool_sensors.free(message_received);
    }  
}

void receive_info_from_GPS(uint8_t * fix_status,  uint8_t * gps_hour, uint8_t * gps_minute, float * gps_seconds, float * latitude, float * longitude,  float * altitude){
    //feching older message in the queue
    osEvent evt = queue_gps.get(0);

    //checking that we have received a message from the queue
    if (evt.status == osEventMessage) {

        //extracting the data from the message
        message_t_gps *message_received = (message_t_gps*)evt.value.p;

        //extracting each field of the sturcture
        *fix_status = message_received->fix_status;

        *gps_hour = message_received->gps_hour;
        *gps_minute = message_received->gps_minute; 
        *gps_seconds = message_received->gps_seconds;

        *latitude = message_received->latitude;
        *longitude = message_received->longitude;
        *altitude = message_received->altitude;

        //releasing allocated memory from the message pool
        mpool_gps.free(message_received);
    }
}