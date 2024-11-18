/* File for the accelerometer MMA8451Q function declarations and macros */

// LIBRARIES ------------------------------------------------------------------------------------
#include "mbed.h"

// LIBRARY GUARD --------------------------------------------------------------------------------
#ifndef MMA8451_H
#define MMA8451_H

// MMA8451 MACROS -------------------------------------------------------------------------------
#define MMA8451_I2C_ADDRESS (0x1D << 1)                           // MMA8451Q I2C Address: 7-bit address shifted because of mBed 8-bit format
#define INT_PIN_PULSE PA_12                                       // Pin connected to IN1 of the accelerometer to receive the pulse interruption
#define INT_PIN_FF PA_11                                          // Pin connected to IN2 of the accelerometer to receive the freefall interruption
#define CTRL_REG1 0x2A                                            // MMA8451Q Control Register 1 to allow writing
#define FF_MT_CFG 0x15                                            // Freefall/Motion configuration register
#define FF_MT_THS 0x17                                            // Freefall/Motion threshold register
#define FF_MT_COUNT 0x18                                          // Freefall/Motion debounce counter
#define PULSE_CFG 0x21                                            // Enable single pulse on X, Y and Z axis
#define PULSE_THSX 0x23                                           // X threshold
#define PULSE_THSY 0x24                                           // Y threshold
#define PULSE_THSZ 0x25                                           // Z threshold
#define PULSE_TMLT 0x26                                           // Time limit for tap detection
#define PULSE_LTCY 0x27                                           // Latency time limit
#define CTRL_REG4 0x2D                                            // Interrupt enable register
#define CTRL_REG5 0x2E                                            // Interrupt pin routing register
#define OUT_X_MSB 0x01                                            // Register for X-axis MSB
#define OUT_Y_MSB 0x03                                            // Register for Y-axis MSB
#define OUT_Z_MSB 0x05                                            // Register for Z-axis MSB

// PROTOTYPES ===================================================================================
void write_register_mma8451(char reg, char value);
char read_register_mma8451(char reg);
int16_t read_axis(char msb_reg);
void read_accelerations(float *ax, float *ay, float *az);
void init_mma8451_pulse_ff();
// PROTOTYPES END ===============================================================================

#endif