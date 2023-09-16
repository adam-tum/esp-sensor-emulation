/**
 * Software based I2C slave driver for ESP32.
 * 
 * "Software based" or "Bitbanged" means that voltage levels of the communicating GPIO pins are set "manually"
 * by software and not by a dedicated hardware peripheral.
 * 
 * This component provides an interface for software based I2C slave devices on the ESP32.
*/

#ifndef __I2C_BITBANG_H__
#define __I2C_BITBANG_H__

#include <stdatomic.h>
#include "esp_attr.h"

#ifdef __cplusplus
extern "C" {
#endif

// Possible states throughout I2C transaction of slave device
typedef enum {
    I2C_READ_MODE,
    I2C_WRITE_MODE,
    NO_DEVICE_ADDRESSED,
} i2c_bitbang_state_t ;

// Generic function signature for parser function.
// Every active slave device needs to provide a function of this type.
// The parser function is invoked, when received data is addressed to the correspongding I2C device.
// This function implements the processing of reveiced data and possibly generates a response.
typedef void ( *parser) (void *, uint8_t , uint8_t *, uint16_t *, i2c_bitbang_state_t, uint16_t);

/**
 * Arrays for every I2C device registered to the software I2C service.
 * Arrays are indexed by device address. 
 * If an array entry is NULL, then there is no active I2C device under the specified address.
 * 
 * Parser arrays contain callable functions to the response handling functions of active slave devices.
 * Emulator arrays contain void pointers to structures in memory that contain the emulation context of a registered slave 
 * device.
 * 
 * There are duplicate arrays, so that addition and removal of an I2C device can be done without interfering with an ongoing
 * transaction.
*/
extern DRAM_ATTR parser i2c_parsers1[128];
extern DRAM_ATTR void *i2c_emulators1[128];
extern DRAM_ATTR parser i2c_parsers2[128];
extern DRAM_ATTR void *i2c_emulators2[128];

// Atomic integer used as flag if transaction is currently in progress
extern DRAM_ATTR uint8_t modified;

/**
 * @brief Registers a new I2C slave device to the software I2C service.
 * @param address       The desired address of the newly registered device.
 * @param emulator      Pointer to the emulation context structure of the device.
 * @param parse         Parser function of the device.
*/
void IRAM_ATTR set_i2c_slave(uint8_t address, void *emulator, parser parse);

/**
 * @brief Returns a void pointer to the emulation context of an active I2C device.
 * @param address       Address of desired device.
*/
void * IRAM_ATTR get_i2c_slave(uint8_t address);

/**
 * @brief Starts and runs the I2C software slave service in a dedicated process.
 * @param sda       SDA GPIO pin number.
 * @param scl       SCL GPIO pin number.
 * @param cpu       CPU number on which the I2C service process will run.
*/
void IRAM_ATTR start_i2c_bitbang_slave( const uint8_t sda, const uint8_t scl, const uint8_t cpu);

#ifdef __cplusplus
}
#endif

/**@}*/

#endif /* __I2C_BITBANG_H__ */
