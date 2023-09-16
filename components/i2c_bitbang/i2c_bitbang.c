/**
 * Software based I2C slave driver for ESP32.
 * 
 * "Software based" or "Bitbanged" means that voltage levels of the communicating GPIO pins are set "manually"
 * by software and not by a dedicated hardware peripheral.
 * 
 * This component provides an interface for software based I2C slave devices on the ESP32.
*/

#include <string.h>
#include <inttypes.h>
#include <esp_heap_caps.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_attr.h"

#include "driver/gpio.h"
#include "i2c_bitbang.h"

// Receive and transmit buffer sizes for I2C communication.
#define RXSIZE 1024
#define TXSIZE 1024

const DRAM_ATTR char *TAG = "i2c_bitbang_slave";

// static DRAM_ATTR uint8_t rx_fifo[RXSIZE] = { 0 };
static DRAM_ATTR uint8_t tx_fifo[TXSIZE] = { 0 };

/* State 0 means no device is yet addressed, State 1 means read mode, State 2 means write mode of a device */
static DRAM_ATTR uint8_t i2c_state = 2;
static DRAM_ATTR uint8_t device = 0;
static DRAM_ATTR uint16_t rx_count = 0;

DRAM_ATTR parser i2c_parsers1[128] = { 0 };
DRAM_ATTR void *i2c_emulators1[128] = { 0 };
DRAM_ATTR parser i2c_parsers2[128] = { 0 };
DRAM_ATTR void *i2c_emulators2[128] = { 0 };

DRAM_ATTR uint8_t modified = 0;

// Struct to pass slave parameters easier.
typedef struct {
    uint8_t sda;
    uint8_t scl;
    uint32_t sdamask;
} slave_params_t;

/**
 * @brief Transfer a single byte of data via I2C to master.
 * @param sda       SDA GPIO number.
 * @param scl       SCL GPIO number.
 * @param sdamask   Mask that enables access to SDA GPIO pin more easily via GPIO hardware register.
 * @param byte      The databyte to transmit.
*/
static uint8_t IRAM_ATTR i2c_bitbang_write_byte( const uint8_t sda, const uint8_t scl, const uint32_t sdamask, const uint8_t byte ) {
    uint8_t mask = 0x80;

    while(((GPIO.in >> scl) & 0x1) == 0);
    if(((GPIO.in >> sda) & 0x1) == 1) return 0;
    while(mask) {
        while(((GPIO.in >> scl) & 0x1) == 1);
        if(byte & mask) GPIO.out_w1ts = sdamask;
        else GPIO.out_w1tc = sdamask;
        mask >>= 1;
        while(((GPIO.in >> scl) & 0x1) == 0);
    }
    while(((GPIO.in >> scl) & 0x1) == 1);
    GPIO.out_w1ts = sdamask;
    return 1;
}

/**
 * @brief Transfer zero or more bytes via I2C to master.
 * @param sda       SDA GPIO number.
 * @param scl       SCL GPIO number.
 * @param sdamask   Mask that enables access to SDA GPIO pin more easily via GPIO hardware register.
 * @param bytes     The databytes to transmit.
 * @param len       Length of databytes.
*/
static void IRAM_ATTR i2c_bitbang_write_bytes( const uint8_t sda, const uint8_t scl, const uint32_t sdamask, const uint8_t *bytes, const uint8_t len ) {
    for(int i = 0; i < len; ++i) {
        if(!i2c_bitbang_write_byte(sda, scl, sdamask, bytes[i])) 
            return;
    }
}

/**
 * @brief Parse a single byte received over I2C. Called after every received byte.
 *        Invokes parser of corresponding device, if addressed device exists as active device.
 *        Afterwards, sends response to master if device generates a response.
 * @param sda       SDA GPIO number.
 * @param scl       SCL GPIO number.
 * @param sdamask   Mask that enables access to SDA GPIO pin more easily via GPIO hardware register.
 * @param byte      The received byte.
*/
static void IRAM_ATTR i2c_bitbang_parse_byte( const uint8_t sda, const uint8_t scl, uint32_t sdamask, const uint8_t byte ) {
    uint16_t len = 0;
    if(!modified) {
        i2c_parsers1[device](i2c_emulators1[device], byte, tx_fifo, &len, i2c_state, rx_count - 1 );
    } else {
        i2c_parsers2[device](i2c_emulators2[device], byte, tx_fifo, &len, i2c_state, rx_count - 1 );
    }
    if(!len) return;
    i2c_bitbang_write_bytes(sda, scl, sdamask, tx_fifo, len);
}

/**
 * @brief Heart of I2C slave service.
 *        Endless loop waits for and receives data from I2C master.
 *        After every received byte, i2c_bitbang_parse_byte is called.
*/
static void IRAM_ATTR i2c_bitbang_slave( void *pvParameters ) {
    taskDISABLE_INTERRUPTS();

    slave_params_t *params = (slave_params_t *) pvParameters;
    
    uint8_t sda, scl;
    uint32_t sdamask;
    uint8_t sda_state;
    uint8_t byte = 0;
    uint8_t mask = 0x80;

    sda = params->sda;
    scl = params->scl;
    sdamask = params->sdamask;

    while(1) {
        while(((GPIO.in >> scl) & 0x1) == 0);
        while(((GPIO.in >> scl) & 0x1) == 1) while(((GPIO.in >> sda) & 0x1) == 1);
        while(1) {
            RESTART:
            while(((GPIO.in >> scl) & 0x1) == 0);
            if(((GPIO.in >> sda) & 0x1)) byte |= mask;
            sda_state = (GPIO.in >> sda) & 0x1;
            // High logic level phase of SCL pin.
            while(((GPIO.in >> scl) & 0x1) == 1) {
                // I2C Restart condition occurence check.
                if(((GPIO.in >> sda) & 0x1) < sda_state) {
                    // Reset all relevant parameters and restart transaction.
                    byte = 0;
                    mask = 0x80;
                    while(((GPIO.in >> scl) & 0x1) == 1);
                    i2c_state = 2;
                    rx_count = 0;
                    goto RESTART;
                }
                // I2C STOP condition occurence check.
                if(((GPIO.in >> sda) & 0x1) > sda_state) {
                    goto STOP;
                }
            }
            if(!mask) GPIO.out_w1ts = sdamask;
            mask = !mask ? 0x80 : mask >> 1;
            if(!mask) {
                if(i2c_state == 2){
                    device = byte >> 1;
                    if(!modified) {
                        if(!i2c_parsers1[device]) goto CONTINUE;
                    } else {
                        if(!i2c_parsers2[device]) goto CONTINUE;
                    }
                    i2c_state = (byte & 0x1) ? 0 : 1;
                }
                GPIO.out_w1tc = sdamask;
                rx_count++;
                i2c_bitbang_parse_byte(sda, scl, sdamask, byte);
                CONTINUE:
                byte = 0;
            }
        }
        STOP:
        i2c_state = 2;
        rx_count = 0;
        byte = 0;
        mask = 0x80;
    }

    taskENABLE_INTERRUPTS();
    free(params);
    vTaskDelete(NULL);
}

void IRAM_ATTR set_i2c_slave(uint8_t address, void *emulator, parser parse) {
    modified = 1;
    taskDISABLE_INTERRUPTS();
    i2c_emulators1[address] = emulator;
    i2c_parsers1[address] = parse;
    taskENABLE_INTERRUPTS();
    modified = 0;
    taskDISABLE_INTERRUPTS();
    i2c_emulators2[address] = emulator;
    i2c_parsers2[address] = parse;
    taskENABLE_INTERRUPTS();
}

void * IRAM_ATTR get_i2c_slave(uint8_t address) {
    taskDISABLE_INTERRUPTS();
    void * emu = i2c_emulators1[address];
    taskENABLE_INTERRUPTS();
    return emu;
}

void IRAM_ATTR start_i2c_bitbang_slave( const uint8_t sda, const uint8_t scl, const uint8_t cpu) {
    // Prepare the GPIO pins and resistors for I2C communication.
    gpio_reset_pin(sda);
    gpio_set_level(sda, 1);
    gpio_set_direction(sda, GPIO_MODE_INPUT_OUTPUT_OD);
    gpio_set_pull_mode(sda, GPIO_PULLUP_ONLY);
    
    gpio_reset_pin(scl);
    gpio_set_level(scl, 1);
    gpio_set_direction(scl, GPIO_MODE_INPUT_OUTPUT_OD);
    gpio_set_pull_mode(scl, GPIO_PULLUP_ONLY);

    slave_params_t *params = heap_caps_malloc(sizeof(slave_params_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);

    params->sda = sda;
    params->scl = scl;
    params->sdamask = 1 << sda;

    // Start I2C process.    
    xTaskCreatePinnedToCore(&i2c_bitbang_slave, "i2c_bitbang_slave", 4096, (void *) params, configMAX_PRIORITIES - 1, NULL, cpu);
}