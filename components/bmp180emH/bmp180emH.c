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

#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <inttypes.h>
#include <time.h>
#include <sys/time.h>
#include <stdatomic.h>

#include <esp_err.h>
#include <esp_heap_caps.h>
#include <esp32/rom/ets_sys.h>
#include "esp_attr.h"

#include "logger.h"
#include "i2c.h"
#include "i2cdev.h"
#include "linked_list.h"
#include "progresser.h"
#include "timesync.h"
#include "client.h"
#include "bmp180.h"
#include "bmp180emH.h"

// Calibration values taken from a genuine BMP180 sensor
#define DEF_AC1 7520
#define DEF_AC2 -1127
#define DEF_AC3 -14559
#define DEF_AC4 34004
#define DEF_AC5 25078
#define DEF_AC6 22286
#define DEF_B1 6515
#define DEF_B2 42
#define DEF_MB -32768
#define DEF_MC -11786
#define DEF_MD 2772

#define BMP180_VERSION_REG        0xD0
#define BMP180_CONTROL_REG        0xF4
#define BMP180_RESET_REG          0xE0
#define BMP180_OUT_MSB_REG        0xF6
#define BMP180_OUT_LSB_REG        0xF7
#define BMP180_OUT_XLSB_REG       0xF8

#define BMP180_CALIBRATION_REG    0xAA

// Values for BMP180_CONTROL_REG
#define BMP180_MEASURE_TEMP       0x2E
#define BMP180_MEASURE_PRESS      0x34

// Reset value for BMP180_RESET_REG
#define BMP180_RESET_VALUE        0xB6

// CHIP ID stored in BMP180_VERSION_REG
#define BMP180_CHIP_ID            0x55

// Size of hardware I2C module rx and tx buffer
#define BMP180_QUEUE_SIZE      1024

// static const char *TAG = "bmp180emH";

static DRAM_ATTR bmp180_emH_t emulator;

static struct timeval tv;
static char timestring[64] = {0};

// Same as in bmp180em.
static esp_err_t bmp180emH_inverse(bmp180_dev_t *dev, double temperature, int32_t *ut, uint64_t pressure, uint64_t *up, int oss) {
    double UT, X1, B5, T;

    T = (temperature * 10);
    B5 = (T * 16) - 8;
    X1 = 0.5 * (B5 + sqrt(pow(B5, 2) + (2*B5*dev->MD) - (8192* dev->MC) +  pow(dev->MD, 2)) - dev->MD);
    UT = ((X1 * 32768) / dev->AC5) + dev->AC6;
    *ut = (int32_t) ceil(UT);

    if(up) {
        double UP, PREP, X1, X2, X3, B3, B4, B6, B7;

        B6 = B5 - 4000;
        X1 = (dev->B2 * (B6 * B6 / pow(2, 12))) / pow(2, 11);
        X2 = dev->AC2 * B6 / pow(2, 11);
        X3 = X1 + X2;
        B3 = (((dev->AC1 * 4 + X3) * pow(2,oss)) + 2) / 4;
        X1 = dev->AC3 * B6 / pow(2, 13);
        X2 = (dev->B1 * (B6 * B6 / pow(2, 12))) / pow(2, 16);
        X3 = ((X1 + X2) + 2) / 4;
        B4 = dev->AC4 * (unsigned long) (X3 + 32768) / pow(2, 15);
        PREP = (16384 * (sqrt((194432 * pressure) + 1084090937729) - 1041219)) / 1519;
        B7 = (PREP * B4) / 2;
        UP = (B7 / (50000 / pow(2, oss))) + B3;
        *up = ((uint64_t) UP) << (8 - oss);
    }

    return ESP_OK;
}

static uint16_t swap(uint16_t num) {
    return (num << 8 | num >> 8);
}

// Same as in bmp180em.
static void update_output(uint8_t temp, uint8_t press) {
    if(!temp && !press) return;

    int32_t ut;
    uint64_t up;

    bmp180emH_inverse(emulator.bmp180, *(double *)emulator.temperatures->head->data, &ut, *(uint64_t *)emulator.pressures->head->data, &up, emulator.oss);

    if(temp)
        atomic_store(emulator.output_temp, ((ut & 0x00ff0000) | ((ut & 0xff) << 8)) | (ut >> 8 & 0xff));
    if(press)
        atomic_store(emulator.output_press, (((up & 0xff) << 16) | ((up >> 8 & 0xff) << 8)) | (up >> 16 & 0xff));

}

// Same as in bmp180em.
static void delete_item(list_node_t * node) {
    free(node->data);
}

// Same as in bmp180em.
void bmp180emH_progresser(void *bmp180emH) {
    uint8_t tempchange = 0;	
    uint8_t presschange = 0;

	gettimeofday(&tv, NULL);

    xSemaphoreTake(emulator.semaphore, portMAX_DELAY);
    if(emulator.temperatures->length > 1 && emulator.temperatures->head->next->timestamp <= tv.tv_sec) {
        list_pop_front(emulator.temperatures);

        tempchange = 1;
        timestamp(timestring);
        log_temp_slave(timestring, emulator.bmp180->i2c_dev.addr, *(double *) emulator.temperatures->head->data);
    }
    xSemaphoreGive(emulator.semaphore);

    xSemaphoreTake(emulator.semaphore, portMAX_DELAY);
    if(emulator.pressures->length > 1 && emulator.pressures->head->next->timestamp <= tv.tv_sec) {
        list_pop_front(emulator.pressures);
        
        presschange = 1;
        timestamp(timestring);
        log_press_slave(timestring, emulator.bmp180->i2c_dev.addr, *(uint64_t *) emulator.pressures->head->data);
    }
    xSemaphoreGive(emulator.semaphore);

    xSemaphoreTake(emulator.semaphore, portMAX_DELAY);	
    update_output(tempchange, presschange);	
    xSemaphoreGive(emulator.semaphore);
}

// Almost the same as in bmp180em, but slightly different interface and parameters to hardware I2C implementation.
void IRAM_ATTR bmp180emH_parser(uint8_t *data_buf, int *rx_cnt, uint8_t *ans, int *ans_size, uint16_t event) {
    uint8_t received = *data_buf;
    
    if (!emulator.state) {
        switch(received) {
            case BMP180_CALIBRATION_REG:
                *(int16_t *) ans = (int16_t) swap(emulator.bmp180->AC1);
                break;
            case BMP180_CALIBRATION_REG + 2:
                *(int16_t *) ans = (int16_t) swap(emulator.bmp180->AC2);
                break;
            case BMP180_CALIBRATION_REG + 4:
                *(int16_t *) ans = (int16_t) swap(emulator.bmp180->AC3);
                break;
            case BMP180_CALIBRATION_REG + 6:
                *(int16_t *) ans = (int16_t) swap(emulator.bmp180->AC4);
                break;
            case BMP180_CALIBRATION_REG + 8:
                *(int16_t *) ans = (int16_t) swap(emulator.bmp180->AC5);
                break;
            case BMP180_CALIBRATION_REG + 10:
               *(int16_t *) ans = (int16_t) swap(emulator.bmp180->AC6);
                break;
            case BMP180_CALIBRATION_REG + 12:
                *(int16_t *) ans = (int16_t) swap(emulator.bmp180->B1);
                break;
            case BMP180_CALIBRATION_REG + 14:
                *(int16_t *) ans = (int16_t) swap(emulator.bmp180->B2);
                break;
            case BMP180_CALIBRATION_REG + 16:
                *(int16_t *) ans = (int16_t) swap(emulator.bmp180->MB);
                break;
            case BMP180_CALIBRATION_REG + 18:
                *(int16_t *) ans = (int16_t) swap(emulator.bmp180->MC);
                break;
            case BMP180_CALIBRATION_REG + 20:
                *(int16_t *) ans = (int16_t) swap(emulator.bmp180->MD);
                break;
            case BMP180_VERSION_REG:
                ans[0] = BMP180_CHIP_ID;
                break;
            case BMP180_OUT_XLSB_REG:
                /* break; */ /* FALLTHROUGH */
            case BMP180_OUT_LSB_REG:
                /* break; */ /* FALLTHROUGH */
            case BMP180_OUT_MSB_REG:
                if(emulator.last_query == BMP180_MEASURE_TEMP) {
                    *(uint32_t *) ans = atomic_load(emulator.output_temp);
                } else {
                    *(uint32_t *) ans = atomic_load(emulator.output_press);
                }
                break;
            case BMP180_CONTROL_REG:
                emulator.state = 1;
                break;
        }
    } else {
        switch(received) {
            case BMP180_MEASURE_TEMP: /* KICKOFF TEMP MEASUREMENT */
                emulator.last_query = BMP180_MEASURE_TEMP;
                break;
             /* KICKOFF PRESS MEASUREMENT */
            case (BMP180_MODE_ULTRA_HIGH_RESOLUTION << 6) | BMP180_MEASURE_PRESS:
                emulator.oss = BMP180_MODE_ULTRA_HIGH_RESOLUTION;
                emulator.last_query = BMP180_MEASURE_PRESS;
                break;
            case (BMP180_MODE_HIGH_RESOLUTION << 6) | BMP180_MEASURE_PRESS:
                emulator.oss = BMP180_MODE_HIGH_RESOLUTION;
                emulator.last_query = BMP180_MEASURE_PRESS;
                break;
            case (BMP180_MODE_STANDARD << 6) | BMP180_MEASURE_PRESS:
                emulator.oss = BMP180_MODE_STANDARD;
                emulator.last_query = BMP180_MEASURE_PRESS;
                break;
            case (BMP180_MODE_ULTRA_LOW_POWER << 6) | BMP180_MEASURE_PRESS:
                emulator.oss = BMP180_MODE_ULTRA_LOW_POWER;
                emulator.last_query = BMP180_MEASURE_PRESS;
                break;
            case BMP180_RESET_REG:
                break;
        }
        emulator.state = 0;
    }
    *ans_size = 3;
    *rx_cnt = 0;
}

// Same as in bmp180em.
void bmp180emH_updater(void *bmp180emH, char *rx_buffer) {
    uint16_t frame_index = 0;

    // Parse temperatures
    uint16_t temp_length = *(uint16_t *) (&rx_buffer[frame_index]);
    frame_index += 2;
    xSemaphoreTake(emulator.semaphore, portMAX_DELAY);
    if(temp_length > 0) {
        for (int n = 0; n < temp_length; ++n) {
            double *temperature = (double *) calloc(1, sizeof(double));
            *temperature = *(double *) (&rx_buffer[frame_index + n * sizeof(double)]);
            list_append(emulator.temperatures, (void *) temperature, *(time_t *) (&rx_buffer[frame_index + (temp_length * sizeof(double)) + (n * sizeof(time_t))]));
        }
        frame_index += temp_length * sizeof(double) + temp_length * sizeof(time_t);
    }
    xSemaphoreGive(emulator.semaphore);

    // Parse pressures
    uint16_t press_length = *(uint16_t *) (&rx_buffer[frame_index]);
    frame_index += 2;
    xSemaphoreTake(emulator.semaphore, portMAX_DELAY);
    if(press_length > 0) {
        for (int n = 0; n < press_length; ++n) {
            uint64_t *pressure = (uint64_t *) calloc(1, sizeof(uint64_t));
            *pressure = *(uint64_t *) (&rx_buffer[frame_index + n * sizeof(uint64_t)]);
            list_append(emulator.pressures, (void *) pressure, *(time_t *) (&rx_buffer[frame_index + (press_length * sizeof(uint64_t)) + (n * sizeof(time_t))]));
        } 
        frame_index += press_length * sizeof(uint64_t) + press_length * sizeof(time_t);
    }
    xSemaphoreGive(emulator.semaphore);
}

// Almost same as in bmp180em but different interface.
bmp180_emH_t *bmp180emH_creater(i2c_port_t port, uint8_t sda, uint8_t scl, uint8_t address, uint8_t flags) {
    bmp180_dev_t *bmp180dev = (bmp180_dev_t *) heap_caps_calloc(1, sizeof(bmp180_dev_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);

    emulator.output_temp = (atomic_uint *) heap_caps_malloc(sizeof(atomic_uint), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    emulator.output_press = (atomic_uint *) heap_caps_malloc(sizeof(atomic_uint), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    atomic_init(emulator.output_temp, 0);
    atomic_init(emulator.output_press, 0);

    bmp180dev->i2c_dev.port = port;
    bmp180dev->i2c_dev.cfg.sda_io_num = sda;
    bmp180dev->i2c_dev.cfg.sda_pullup_en = GPIO_PULLUP_ENABLE;
    bmp180dev->i2c_dev.cfg.scl_io_num = scl;
    bmp180dev->i2c_dev.cfg.scl_pullup_en = GPIO_PULLUP_ENABLE;
    bmp180dev->i2c_dev.cfg.slave.addr_10bit_en = 0;
    bmp180dev->i2c_dev.cfg.slave.slave_addr = address;
    bmp180dev->i2c_dev.rx_bufsize = BMP180_QUEUE_SIZE;
    bmp180dev->i2c_dev.tx_bufsize = BMP180_QUEUE_SIZE;
    i2c_dev_create_mutex(&bmp180dev->i2c_dev);

    emulator.bmp180 = bmp180dev;
    emulator.bmp180->AC1 = DEF_AC1;
    emulator.bmp180->AC2 = DEF_AC2;
    emulator.bmp180->AC3 = DEF_AC3;
    emulator.bmp180->AC4 = DEF_AC4;
    emulator.bmp180->AC5 = DEF_AC5;
    emulator.bmp180->AC6 = DEF_AC6;
    emulator.bmp180->B1 = DEF_B1;
    emulator.bmp180->B2 = DEF_B2;
    emulator.bmp180->MB = DEF_MB;
    emulator.bmp180->MC = DEF_MC;
    emulator.bmp180->MD = DEF_MD;

    emulator.oss = BMP180_MODE_ULTRA_LOW_POWER;
    emulator.semaphore = xSemaphoreCreateBinary();

    emulator.temperatures = list_new(&delete_item);
    double *init_temperature = (double *) calloc(1, sizeof(double));
    *init_temperature = 20.0;
    list_append(emulator.temperatures, (void *) init_temperature, LONG_MAX);

    emulator.pressures = list_new(&delete_item);
    uint64_t *init_pressure = (uint64_t *) calloc(1, sizeof(uint64_t));
    *init_pressure = 101325;
    list_append(emulator.pressures, (void *) init_pressure, LONG_MAX);

    update_output(1, 1);

    xSemaphoreGive(emulator.semaphore);

    // No registration as I2C device, cause communication is later handled via hardware module.
    set_progress_subscriber(address, &bmp180emH_progresser);
    set_update_slave(address, &bmp180emH_updater);
    return &emulator;
}

// Almost same as in bmp180em.
void bmp180emH_destroyer()
{
    i2c_dev_delete_mutex(&emulator.bmp180->i2c_dev);
    set_progress_subscriber(emulator.bmp180->i2c_dev.addr, NULL);
    set_update_slave(emulator.bmp180->i2c_dev.addr, NULL);

    xSemaphoreTake(emulator.semaphore, portMAX_DELAY);
    vSemaphoreDelete(emulator.semaphore);

    free(emulator.bmp180);
    list_free(emulator.temperatures);
    list_free(emulator.pressures);
}