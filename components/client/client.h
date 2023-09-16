/*
Header File for scalable Sensor-Emulation Testbed TCP/IP client.
 */
#ifndef __CLIENT_H__
#define __CLIENT_H__

#include "esp_attr.h"

#ifdef __cplusplus
extern "C" {
#endif

// Functoin pointer to update method signature type.
// Used for array of updatable devices registered to updating process.
typedef void ( *updater) (void *, char *);

extern uint8_t running;

// Array of all registered devices ready to receive updates.
extern updater sensor_updaters[128];

/**
 * @brief Registers a device to the updating process.
 * @param address   Device address to be registered (0-128).
 * @param update    Function pointer to the device's update method.
*/
void set_update_slave(uint8_t address, updater update);

/**
 * @brief Starts the TCP/IP client on the ESP32.
*/
void start_client();

#ifdef __cplusplus
}
#endif

/**@}*/

#endif /* __CLIENT_H__ */
