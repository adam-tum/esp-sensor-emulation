/**
 * Progresser component realizes progression service in independent process for all emulators active on the system.
 * 
 * Subscribed emulators get their progresser function invoked in an endless loop multiple times per second.
 * This way, output values for subscribed emulators can be updated in a timely manner at second granularity.
 * 
 * Progresser functions of subscribed emulators can check if the current emulation state is valid or the output value needs
 * to be updated, when invoked.
*/

#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_attr.h"

#include "i2c_bitbang.h"
#include "progresser.h"

subscriber subscribers[128] = { 0 };

void set_progress_subscriber(uint8_t address, subscriber sub) {
    taskDISABLE_INTERRUPTS();
    subscribers[address] = sub;
    taskENABLE_INTERRUPTS();
}

void start_progresser( void *pvParameters ) {
    while(1) {
        for(int i  = 0; i < 128; ++i) {
            if(subscribers[i])
                subscribers[i](get_i2c_slave(i));
        }
        vTaskDelay(100);
    }
    vTaskDelete(NULL);
}