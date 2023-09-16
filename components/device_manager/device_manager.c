/**
* Interface that allows creation and destruction of new emulation devices on the system.
*/

#include <stdlib.h>
#include <string.h>
#include <time.h>
#include "esp_attr.h"

#include "i2c_bitbang.h"
#include "bmp180em.h"
#include "timesync.h"
#include "device_manager.h"

creater emulator_creaters[INTERFACE_NUM] = { 0 };
destroyer emulator_destroyers[INTERFACE_NUM] = { 0 };
destroyer active_destroyers[128] = { 0 };

// Calls creater function of designated device and registers it under specified address.
// Turns off interrupts to avoid mistakes via concurrent function calls.
void create_emulator(uint16_t device, uint8_t address, uint8_t flags) {
    emulator_creaters[device](address, flags);
    taskDISABLE_INTERRUPTS();
    active_destroyers[address] = emulator_destroyers[device];
    taskENABLE_INTERRUPTS();
}

// Calls destroyer function of designated device.
// Turns off interrupts to avoid mistakes via concurrent function calls.
void destroy_emulator(uint8_t address) {
    void *destroyed = get_i2c_slave(address);
    active_destroyers[address](destroyed);
    taskDISABLE_INTERRUPTS();
    active_destroyers[address] = NULL;
    taskENABLE_INTERRUPTS();
}

// Initializes possible devices in the creaters and destroyers arrays under their device-ids (BMP180 is ID = 0).
void init_device_manager() {
    emulator_creaters[0] = &bmp180em_creater;
    emulator_destroyers[0] = &bmp180em_destroyer;
}