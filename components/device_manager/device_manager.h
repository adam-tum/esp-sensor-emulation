/**
* Interface that allows creation and destruction of new emulation devices on the system.
*/

#ifndef __DEVICE_MANAGER__H
#define __DEVICE_MANAGER__H

#include "esp_attr.h"

// Number of currently supported device interfaces (1 because currently only BMP180).
#define INTERFACE_NUM 1

#ifdef __cplusplus
extern "C" {
#endif

typedef void ( *creater) (uint8_t, uint8_t);
typedef void ( *destroyer) (void *);

// Array of possible creater functions for devices that can be created. Indexed via device-id.
extern creater emulator_creaters[INTERFACE_NUM];

// Array of destroyer functions for all possible devices. Indexed via device-id.
extern destroyer emulator_destroyers[INTERFACE_NUM];

// Array of destruction functions for all active devices (up to 128). Indexed by device address specified at device creation.
extern destroyer active_destroyers[128];

/**
 * @brief Function to create an active emulator on the system.
 * @param device    Device-Id for the emulator to create.
 * @param address   Address for the active emulator (same as I2C address).
 * @param flags     Unused.
*/
void create_emulator(uint16_t device, uint8_t address, uint8_t flags);

/**
 * @brief Function to destroy an active emulator on the system.
 * @param address   Address of the active emulator.
*/
void destroy_emulator(uint8_t address);

/**
 * @brief Initializer for the device manager component.
*/
void init_device_manager();

#ifdef __cplusplus
}
#endif

/**@}*/

#endif /* __DEVICE_MANAGER__H  */
