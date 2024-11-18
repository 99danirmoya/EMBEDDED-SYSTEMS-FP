/* File for the GPS' thread function definitions */

// LIBRARIES ---------------------------------------------------------------------------
#include "mbed.h"
#include "gps_thread.h"
#include <string.h>
#include "message_q.h"

// CONSTRUCTORS ------------------------------------------------------------------------
BufferedSerial gps(GPS_TX, GPS_RX, GPS_BAUD_RATE);                  // GPS Serial interface (Adjust TX, RX pins for your board)

// GLOBAL VARIABLES --------------------------------------------------------------------
// Variables to parse the GPGGA sentence buffer
char buffer[GPS_BUFFER_SIZE];                                         // GPS sentence buffer
int bufferIndex = 0;

// Variables to store GPS fields
static uint8_t fix_status = 0, gps_hour = 0, gps_minute = 0;
static float gps_seconds = 0.0f;
static float latitude = 0.0f, longitude = 0.0f, altitude = 0.0f;

//static functions
static void settingFrequency();
static void enableGettingStatusFromAntenna();
static void initializesSerialPort();

// Function to parse and extract lat, lon and alt from GPGGA sentence ------------------
bool parse_GPS_data(char *nmea_data){

    bool ret = false;

    if(strncmp(nmea_data, "$GPGGA", 6) == 0){                         // Check if it's a GPGGA sentence
        char *token = strtok(nmea_data, ",");
        int field = 0;

        ret = true;

        while(token != NULL){
            field++;

            // Field 2: UTC Time (hhmmss.sss format)
            if(field == 2 && strlen(token) >= 6) {
                int raw_time = atof(token);
                gps_hour = raw_time / 10000;
                gps_minute = (raw_time % 10000) / 100;
                gps_seconds = fmod(raw_time, 100.0f);

                // Adjust for Spain time (UTC+1)
                gps_hour ++;
                if (gps_hour >= 24) {
                    gps_hour -= 24; // Wrap around if exceeding 23
                }
            }

            // Field 7: GPS Fix status (0 = no fix, 1 = GPS fix, 2 = DGPS fix)
            if(field == 7){
                fix_status = atoi(token);                             // Convert to integer
            }

            // Field 3: Latitude
            if(field == 3 && strlen(token) > 0){
                float raw_latitude = atof(token);
                int degrees = (int)(raw_latitude / 100);
                float minutes = raw_latitude - (degrees * 100);
                latitude = degrees + minutes / 60.0f;
            }

            // Field 4: Latitude hemisphere (N/S)
            if(field == 4 && *token == 'S'){
                latitude = -latitude;                                 // Southern hemisphere
            }

            // Field 5: Longitude
            if(field == 5 && strlen(token) > 0){
                float raw_longitude = atof(token);
                int degrees = (int)(raw_longitude / 100);
                float minutes = raw_longitude - (degrees * 100);
                longitude = degrees + minutes / 60.0f;
            }

            // Field 6: Longitude hemisphere (E/W)
            if(field == 6 && *token == 'W'){
                longitude = -longitude;                               // Western hemisphere
            }

            // Field 10: Altitude
            if(field == 10 && strlen(token) > 0){
                altitude = atof(token);
            }

            token = strtok(NULL, ",");
        }
    }

    return ret;
}

// Function to read and process GPS data -----------------------------------------------
void read_GPS() {
    char c;
    
    // Read from GPS serial until a complete NMEA sentence is received
    while (gps.read(&c, 1)) {
        if (c == '\n') {
            buffer[bufferIndex] = '\0';                               // Null-terminate the string
            bufferIndex = 0;

            // Parse GPS data if it's a GPGGA sentence
            //printf("%s",buffer);
            if(parse_GPS_data(buffer)){
                send_GPS_message_through_main_thread(fix_status, gps_hour, gps_minute, gps_seconds, latitude, longitude,  altitude);
                break;
            }

        } else {
            buffer[bufferIndex++] = c;
            if (bufferIndex >= sizeof(buffer) - 1) {
                bufferIndex = 0;                                      // Prevent buffer overflow
            }
        }
    }
}

// Initializes UART 9600 8N1
static void initializesSerialPort(){
    // Initializes serial port 9600 bauds (8N1)
    gps.set_baud(9600);
    gps.set_format(
        /* bits */     8,
        /* parity */   BufferedSerial::None,
        /* stop bit */ 1
    );


}

// Enable trace to see the antena status
static void enableGettingStatusFromAntenna(){
    gps.write(ENABLE_STATUS_ANTENA, sizeof(ENABLE_STATUS_ANTENA));
}

// Set GPS measuring frequencies
static void settingFrequency(){
    gps.write(SET_UPDATING_NMEA_1HZ_RATE, sizeof(SET_UPDATING_NMEA_1HZ_RATE));  // Setting 1HZ of internal freq (updating NMEA traces frecuency at 1Hz)
    gps.write(SET_SAMPLE_1HZ, sizeof(SET_SAMPLE_1HZ));                // Setting receptor sampling at 1Hz
}

// =====================================================================================
// GPS MAIN FUNCTION
// =====================================================================================
void gps_th_routine(){
    // Initialization routine 
    initializesSerialPort();

    enableGettingStatusFromAntenna();

    settingFrequency();

    while (true) {
        read_GPS();                                                   // Read and process GPS data

        //sending message through message queue

        ThisThread::sleep_for(GPS_THREAD_SLEEP);
    }
}
// GPS MAIN FUNCTION END ===============================================================