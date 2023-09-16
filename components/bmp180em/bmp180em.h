/**
 * This driver emulates the behavior of a BMP180 sensor as an I2C slave on the ESP32.
 * Therefore, this implementation is supposed to behave in the same manner, as a genuine BMP180 slave would.
 * The underlying I2C protocol is realized via a software implementation.
 * 
 * Further details about the I2C implementation can be found in the "i2c_bitbang" component.
*/

#ifndef __BMP180EM_H__
#define __BMP180EM_H__

#include <inttypes.h>
#include <esp_err.h>
#include <esp32/rom/ets_sys.h>
#include <stdatomic.h>
#include "esp_attr.h"

#include "i2c_bitbang.h"
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

    int16_t curr_register;
    int16_t last_query;
} bmp180_em_t;

/**
 * @brief Create, initialize and subscribe an emulator to all relevant handling processes.
 *
 * @param address   I2C bus device-address
 * @param flags     Unused, for possible future use
 */
void bmp180em_creater(uint8_t address, uint8_t flags);

/**
 * @brief Called by a BMP180 emulator after every received byte of an I2C transaction addressed to it.
 *        Used to parse received information and possibly generate according response.
 *
 * @param bmp180em      Emulation context for emulator that calls this function
 * @param received      Last received byte over I2C before calling this function
 * @param[out] ans      Answer buffer to store possibly generated answer of emulator
 * @param[out] ans_size Size of answer written to answer buffer.
 * @param mode          Current mode of I2C device. Can be read, write or invalid state.
 * @param rx_count      Counter for amount of bytes received during current I2C transaction.
 */
void IRAM_ATTR bmp180em_parser(void *bmp180em, uint8_t received, uint8_t *ans, uint16_t *ans_size, i2c_bitbang_state_t mode, uint16_t rx_count);

/**
 * @brief Appends updates to the temperature and pressure curves of an emulator.
 *
 * @param bmp180em      Emulation context for emulator that calls this function.
 * @param rx_buffer     Update array, that contains update data for a BMP180 emulator in a previously established format.
 */
void bmp180em_updater(void *bmp180em, char *rx_buffer);

/**
 * @brief Checks timestamps of the head elements of temperature and pressure lists.
 *        Updates output temperature and/or pressure if necessary.
 *
 * @param bmp180em  Emulation context for emulator that calls this function.
 */
void bmp180em_progresser(void *bmp180em );

/**
 * @brief Destroys the given emulator's emulation context.
 *
 * @param bmp180em  Emulation context for emulator that calls this function.
 */
void bmp180em_destroyer(void *bmp180em);

#ifdef __cplusplus
}
#endif

/**@}*/

#endif /* __BMP180EM_H__ */
