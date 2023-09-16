/**
 * This driver emulates the behavior of a BMP180 sensor as an I2C slave on the ESP32.
 * Therefore, this implementation is supposed to behave in the same manner, as a genuine BMP180 slave would.
 * In contrast to the bmp180em component, the underlying I2C protocol is realized via a hardware implementation.
 * 
 * Further details about the I2C implementation can be found in the "i2c" and "i2cdev" components.
 * 
 * This component contains some of the same functions as the "bmp180em" component.
 * They have been copied and renamed for convenience purposes.
*/

#ifndef __BMP180EMH_H__
#define __BMP180EMH_H__

#include <inttypes.h>
#include <esp_err.h>
#include <esp32/rom/ets_sys.h>
#include <stdatomic.h>
#include "esp_attr.h"

#include "i2c.h"
#include "linked_list.h"
#include "bmp180.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Device descriptor for a BMP180 emulator.
 * 
 * Linked lists for the temperature and pressure curves it emulates.
 * A semaphore to make concurrent access to this descriptor possible.
 * Output registers that contain the current temperature and pressure of the emulator.
 * 
 * And miscellaneous information used throughout the emulation process.
 * 
*/
typedef struct
{
    bmp180_dev_t *bmp180;
    bmp180_mode_t oss;

    list_t *temperatures;
    list_t *pressures;

    SemaphoreHandle_t semaphore;

    atomic_uint *output_temp;
    atomic_uint *output_press;

    uint8_t state;
    int16_t last_query;
} bmp180_emH_t;

/**
 * @brief Create, initialize and return a hardware based emulator, that is subscribed to all relevant handling processes.
 *
 * @param port      I2C hardware port.
 * @param sda       GPIO number of pin used for SDA line
 * @param scl       GPIO number of pin used for SCL line
 * @param address   I2C device address of emulator
 * @param flags     Unused, for possible future use
 */
bmp180_emH_t *bmp180emH_creater(i2c_port_t port, uint8_t sda, uint8_t scl, uint8_t address, uint8_t flags);

/**
 * @brief Called by a BMP180 emulator after every received byte of an I2C transaction addressed to it.
 *        Used to parse received information and possibly generate according response.
 *
 * @param data_buf          Contains received bytes
 * @param[out] rx_cnt       Number of received bytes
 * @param[out] ans          Answer buffer
 * @param[out] ans_size     Answer size
 * @param event             Type of I2C event that triggered this function       
 */
void IRAM_ATTR bmp180emH_parser(uint8_t *data_buf, int *rx_cnt, uint8_t *ans, int *ans_size, uint16_t event);

/**
 * @brief Appends updates to the temperature and pressure curves of an emulator.
 *
 * @param bmp180em      Emulation context for emulator that calls this function.
 * @param rx_buffer     Update array, that contains update data for a BMP180 emulator in a previously established format.
 */
void bmp180emH_updater(void *bmp180em, char *rx_buffer);

/**
 * @brief Checks timestamps of the head elements of temperature and pressure lists.
 *        Updates output temperature and/or pressure if necessary.
 *
 * @param bmp180em  Emulation context for emulator that calls this function.
 */
void bmp180emH_progresser(void *bmp180em );

/**
 * @brief Destroys the given emulator's emulation context.
 *
 * @param bmp180em  Emulation context for emulator that calls this function.
 */
void bmp180emH_destroyer();

#ifdef __cplusplus
}
#endif

/**@}*/

#endif /* __BMP180EMH_H__ */
