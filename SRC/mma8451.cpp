/* File for the accelerometer MMA8451Q function definitions */

// LIBRARIES ------------------------------------------------------------------------------------------------------------
#include "mbed.h"
#include "sensors_thread.h"
#include "mma8451.h"

// CONSTRUCTORS ---------------------------------------------------------------------------------------------------------
extern I2C i2c;                                                   // I2C communication

// FUNCTION TO WRITE TO REGISTER ========================================================================================
static void write_register_mma8451(char reg, char value){         // WRITE function receives the Control Register 1 and the ax direction
    char data[2] = {reg, value};
    i2c.write(MMA8451_I2C_ADDRESS, data, 2);                      // We write in the MMA8451 direction the command to get, for example, ax
}

// FUNCTION TO READ A REGISTER ==========================================================================================
static char read_register_mma8451(char reg){                      // READ function does only get as a parameter the register to be read
    char data;
    i2c.write(MMA8451_I2C_ADDRESS, &reg, 1, true);                // Send register address from, for example, ax
    i2c.read(MMA8451_I2C_ADDRESS, &data, 1);                      // Read data returned from the register
    return data;
}

// FUNCTION TO READ 14-BIT AXIS VALUE (X, Y, Z) =========================================================================
static int16_t read_axis(char msb_reg){                           // This function receives as a parameter only the MSB acceleration register
    char msb = read_register_mma8451(msb_reg);                    // The MSB register for each component is read
    char lsb = read_register_mma8451(msb_reg + 1);                // The same with the LSB register which is actually 6 bit long, having bits 0 and 1, the most significant, empty
    return (int16_t)((msb << 8) | lsb) >> 2;                      // Combine MSB (8-bit) and LSB (6-bit), and shift by 2 for 14-bit value
}

// FUNCTION TO INITIALIZE THE ACCELEROMETER WITH FREEFALL DETECTION =====================================================
void init_mma8451_pulse_ff() {
    write_register_mma8451(CTRL_REG1, 0x08);                      // bit 3 = 1, Standby Mode, DRO = 400 Hz

    // FREEFALL INTERRUPT COMMAND --------------------------------------------------------------------------
    write_register_mma8451(FF_MT_CFG, 0xB8);                      // Enable motion detection on Z-axis with event latch enabled
    write_register_mma8451(FF_MT_THS, 0x03);                      // Set threshold to ~0.18g (0x03 * 0.063g/LSB)
    write_register_mma8451(FF_MT_COUNT, 0x06);                    // Set debounce count to filter transient events, 400 Hz (2.5 ms) timer 120 ms

    // TAP INTERRUPT COMMANDS ------------------------------------------------------------------------------
    write_register_mma8451(PULSE_CFG, 0x15);                      // Configure PULSE_CFG to enable single tap on X, Y, Z with latch enabled
    write_register_mma8451(PULSE_THSX, 0x19);                     // Set X threshold for 1.575g
    write_register_mma8451(PULSE_THSY, 0x19);                     // Set Y threshold for 1.575g
    write_register_mma8451(PULSE_THSZ, 0x2A);                     // Set Z threshold for 2.65g
    write_register_mma8451(PULSE_TMLT, 0x50);                     // Set time limit for 50 ms
    write_register_mma8451(PULSE_LTCY, 0xF0);                     // Set latency for 300 ms

    // SHARED INTERRUPT COMMANDS ---------------------------------------------------------------------------
    write_register_mma8451(CTRL_REG4, 0x0C);                      // Enable pulse (bit 3) and FF (bit 2) interrupt - 0000 1100
    write_register_mma8451(CTRL_REG5, 0x08);                      // Route INT_CFG_PULSE (bit 3 = 1) to IN1 and INT_CFG_FF_MT (bit 2 = 0) to IN2 - 0000 1000
    char data = read_register_mma8451(CTRL_REG1);                 // Initialize the MMA8451Q accelerometer by setting it to active mode, read the state of Control Register 1 (0x2A)
    write_register_mma8451(CTRL_REG1, data | 0x01);
}

// FUNCTION TO READ ACCELERATIONS =======================================================================================
void read_accelerations(float *ax, float *ay, float *az){         // This function receives as parameters pointers to the memory addresses of the variables so they are directly modified, instead of receiving just a copy of that variable
    int16_t raw_x = read_axis(OUT_X_MSB);                         // Remember, only the MSB is passed, but the function adds one to the register to get the LSB part
    int16_t raw_y = read_axis(OUT_Y_MSB);
    int16_t raw_z = read_axis(OUT_Z_MSB);

    // Sensitivity is 4096 counts/g for ±2g range
    *ax = (float)raw_x / 4096.0f;                                 // Asterisk is used again to dereference the pointer, meaning that the value at the memory address that 'ax' points to will be updated with the calculated acceleration for ax
    *ay = (float)raw_y / 4096.0f;                                 // With ±2g range the maximum positive acceleration is 2 * 4096 = 8192 and the same but negative for the negative range. NARROWER RANGE, BUT HIGHER SENSITIVITY
    *az = (float)raw_z / 4096.0f;                                 // By dividing it by 4096, again, the value of Gs is ±2
}