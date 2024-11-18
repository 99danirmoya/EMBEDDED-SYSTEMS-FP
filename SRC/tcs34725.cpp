/* File for the colour sensor TCS34725 function definitions */

// LIBRARIES ---------------------------------------------------------------------------------------------------------------
#include "mbed.h"
#include "sensors_thread.h"
#include "tcs34725.h"

// CONSTRUCTORS ---------------------------------------------------------------------------------------------------------
extern I2C i2c;                                                             // I2C communication

// FUNCTION TO WRITE TO A REGISTER ==============================================================
void write_register(uint8_t reg, uint8_t value){
    char data[2] = {static_cast<char>(TCS34725_COMMAND_BIT | reg), static_cast<char>(value)};  // Command to write to the specific register, which is achieved by combining bit by bit TCS34725_COMMAND_BIT and 'reg' using the bitwise OR (|) operator
    i2c.write(TCS34725_ADDRESS, data, 2);
}

// FUNCTION TO READ FROM A 16-BIT REGISTER ======================================================
uint16_t read_register_16(uint8_t reg){
    char data[2];                                                           // Command to be read, 16-bit size
    char cmd = TCS34725_COMMAND_BIT | reg;                                  // Command to be written (which will be values of the measurements)
    i2c.write(TCS34725_ADDRESS, &cmd, 1);                                   // Write the command
    i2c.read(TCS34725_ADDRESS, data, 2);                                    // Read two bytes
    return (data[1] << 8) | data[0];                                        // Combine into 16-bit value
}

// FUNCTION TO INITIALIZE THE TCS34725 ==========================================================
void tcs34725_init(){
    write_register(TCS34725_ENABLE, TCS34725_ATIME);                        // Power on the device
    ThisThread::sleep_for(3ms);                                             // Wait 3ms for power ON

    write_register(TCS34725_ENABLE, TCS34725_ATIME | TCS34725_ENABLE_AEN);  // Enable the RGBC ADC

    write_register(TCS34725_ATIME, 0xF6);                                   // Integration time: 24ms (for good accuracy) - 2.4 x (256 - ATIME), where 0xF6 is 246
    write_register(TCS34725_AGAIN, 0x01);                                   // Gain control: 4x - BOTH INTEGRATION TIME AND GAIN ARE SET FOR BRIGHT AMBIENT LIGHT CONDITIONS
}
