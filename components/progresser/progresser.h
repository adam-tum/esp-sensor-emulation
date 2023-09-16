/**
 * Progresser component realizes progression service in independent process for all emulators active on the system.
 * 
 * Subscribed emulators get their progresser function invoked in an endless loop multiple times per second.
 * This way, output values for subscribed emulators can be updated in a timely manner at second granularity.
 * 
 * Progresser functions of subscribed emulators can check if the current emulation state is valid or the output value needs
 * to be updated, when invoked.
*/

#ifndef __PROGRESSER__H
#define __PROGRESSER__H

#include "esp_attr.h"

#ifdef __cplusplus
extern "C" {
#endif

// Function signature for progresser function of emulators.
typedef void ( *subscriber) (void *);

// Array of active emulators subscribed to progresser service.
extern subscriber subscribers[128];

/**
 * @brief Set or unset active progres subscriber.
 * @param address       Address of emulator to subscribe.
 * @param sub           Pointer to progresser function of emulator.
*/
void set_progress_subscriber(uint8_t address, subscriber sub);

/**
 * @brief Start endless progresser loop.
*/
void start_progresser( void *pvParameters );

#ifdef __cplusplus
}
#endif

/**@}*/

#endif /* __PROGRESSER__H  */
