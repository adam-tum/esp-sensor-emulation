/**
 * Main component for emulation system.
 * 
 * Contains functions to execute main functionalities and testing for proof-of-concept.
*/

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <sys/time.h>
#include <time.h>
#include <esp_heap_caps.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_attr.h"

// Include all project components
#include "bmp180.h"
#include "bmp180em.h"
#include "bmp180emH.h"
#include "client.h"
#include "connect.h"
#include "device_manager.h"
#include "i2c.h"
#include "i2cdev.h"
#include "linked_list.h"
#include "progresser.h"
#include "timesync.h"
#include "logger.h"
#include "i2c_bitbang.h"

// Colors for colorful text when debugging
#define KNRM "\x1B[0m"
#define KRED "\x1B[31m"
#define KGRN "\x1B[32m"
#define KYEL "\x1B[33m"
#define KBLU "\x1B[34m"
#define KMAG "\x1B[35m"
#define KCYN "\x1B[36m"
#define KWHT "\x1B[37m"

// Access Point login credentials
#define STA_SSID ""
#define STA_PASSWORD ""

// Since BMP180 is only supported device for now, ID 0 is the only valid Device-ID
#define BMP_DEVICE_ID 0

// static const char *TAG = "main";

/**
 * Configuration struct for an I2C master ESP32 for testing.
*/
typedef struct
{
    uint8_t sda;
    uint8_t scl;
    uint8_t *addresses;
    size_t addr_size;
    uint32_t i2c_frequency;
} i2c_master_cfg_t;

/**
 * Configuration struct for an I2C slave ESP32 emulator for testing.
*/
typedef struct
{
    i2c_port_t port;
    uint8_t addr;
    uint8_t sda;
    uint8_t scl;
    uint8_t *update_frame;
} i2c_slave_cfg_t;

// Pointers to all currently active emulators on ESP32 (Can only be one or multiple instances of BMP180 for now)
void *test_devices[128]  = { 0 };
static char timestring[64];

esp_err_t i2c_slave_em_en(i2c_port_t i2c_num, int thresh, void (*handler)(uint8_t *, int *, uint8_t *, int *, uint16_t), uint16_t ints);

// Get active device by address
void *get_bmp180( uint8_t address) {
    return test_devices[address];
}

// Set new active device via address
void set_bmp180( uint8_t address, void *sensor) {
    test_devices[address] = sensor;
}

// Function to initialize hardware I2C port communication from master with BMP180 slave
esp_err_t bmp180_initializer( uint8_t sda, uint8_t scl, uint8_t address, uint32_t i2c_frequency ) {
    bmp180_dev_t *bmp180sensor = (bmp180_dev_t *) heap_caps_calloc(1, sizeof(bmp180_dev_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);

    // Add to list of active devices
    set_bmp180(address, (void *) bmp180sensor);

    // Initialize BMP180 
    bmp180_init_desc(bmp180sensor, I2C_NUM_0, sda, scl, address, i2c_frequency);
    return bmp180_init(bmp180sensor);
}

// Function for I2C master to probe BMP180 for temperature and pressure once (must be initialized first)
esp_err_t bmp180_prober( uint8_t address )
{
    bmp180_dev_t *bmp180sensor =  (bmp180_dev_t *) get_bmp180(address);
    float temperature = 0.0;
    int32_t utemperature = 0;
    uint32_t pressure = 0;
    uint32_t upressure = 0;
    esp_err_t err = ESP_OK;
    
    // Measurement gets kicked off here
    err = bmp180_measure(bmp180sensor, &temperature, &utemperature, &pressure, &upressure, BMP180_MODE_ULTRA_LOW_POWER);
    timestamp(timestring);

    // Logging if error occurs during I2C transaction
    if(err == ESP_OK) {
        log_temp_master(timestring, address, temperature, utemperature);
        log_press_master(timestring, address, pressure, upressure);
        return ESP_OK;
    } else if (err == -1) {
        log_tmeaserr_master(timestring, address);
        return ESP_FAIL;
    } else if (err == -2) {
        log_pmeaserr_master(timestring, address);
        return ESP_FAIL;
    } else {
        log_measerr_master(timestring, address);
        return ESP_FAIL;
    }
}

// Connect ESP32 to access point corresponding to given ssid and password
void init_wireless(char *sta_ssid, char *sta_pass)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_connect(sta_ssid, STA_PASSWORD, 0, "", "", "");
}

// Generate a BMP180 update frame byte-array of the format as described in "bmp180em.c".
// Update frame contains passed array of temperatures, temperature times, pressures and pressure times.
uint8_t *test_generate_bmp180em_update_frame(double *temperatures, time_t *temp_times, uint16_t temp_len, uint64_t *pressures, time_t *press_times, uint16_t press_len)
{
    uint16_t frame_size = 0;

    // Calculate size of update frame. Dependent on size of given update arrays.
    frame_size += sizeof(uint16_t);
    frame_size += temp_len * sizeof(double);
    frame_size += temp_len * sizeof(time_t);
    frame_size += sizeof(uint16_t);
    frame_size += press_len * sizeof(uint64_t);
    frame_size += press_len * sizeof(time_t);

    uint8_t *frame = (uint8_t *)calloc(1, frame_size);
    uint32_t frame_index = 0;

    *(uint16_t *)&frame[frame_index] = temp_len;
    frame_index += 2;

    // Put temperatures into update frame.
    for (uint16_t i = 0; i < temp_len; ++i)
        *(double *)&frame[frame_index + i * sizeof(double)] = temperatures[i];
    frame_index += temp_len * sizeof(double);

    // Put temperature times into update frame.
    for (uint16_t i = 0; i < temp_len; ++i)
        *(time_t *)&frame[frame_index + i * sizeof(time_t)] = temp_times[i];
    frame_index += temp_len * sizeof(time_t);

    *(uint16_t *)&frame[frame_index] = press_len;
    frame_index += 2;

    // Put pressures in
    for (uint16_t i = 0; i < press_len; ++i)
        *(uint64_t *)&frame[frame_index + i * sizeof(uint64_t)] = pressures[i];
    frame_index += press_len * sizeof(uint64_t);

    // Put pressure times in
    for (uint16_t i = 0; i < press_len; ++i)
        *(time_t *)&frame[frame_index + i * sizeof(time_t)] = press_times[i];
    frame_index += press_len * sizeof(time_t);

    return frame;
}

/**
 * Helper method to easily generate an array of temperatures for an update of BMP180 emulator.
 * 
 * Example: Start Value: -40.0, Step Value: 0.1, Num Value: 5 generates an array like this:
 * 
 * { -40.0, -39.9, -39.8, -39.7, -39.6} and saves it at the location of the temperatures pointer.
*/
void test_generate_temperatures(double start, double step, double num, double **temperatures)
{
    *temperatures = (double *)calloc(1, num * sizeof(double));
    for (int i = 0; i < num; ++i)
        (*temperatures)[i] = start + i * step;
}

// Same as temperature generator only for pressures.
void test_generate_pressures(uint64_t start, int16_t step, uint16_t num, uint64_t **pressures)
{
    *pressures = (uint64_t *)calloc(1, num * sizeof(uint64_t));
    for (int i = 0; i < num; ++i)
        (*pressures)[i] = start + i * step;
}

// Same as temperatures and pressures only for timestamps.
void test_generate_times(time_t start, uint32_t step, uint16_t num, time_t **times)
{
    *times = (time_t *)calloc(1, num * sizeof(time_t));
    for (int i = 0; i < num; ++i)
        (*times)[i] = start + i * step;
}

// I2C master task during experiment run. Initializes all configured BMP180 devices.
// Endless loop queries BMP180s for pressure and temperature values.
void test_i2c_master_wifi(void *pvParameters)
{
    i2c_master_cfg_t *master_cfg = (i2c_master_cfg_t *)pvParameters;

    i2cdev_init();

    while(!running) vTaskDelay(1);

    esp_err_t err;

    // Initialize all BMP180 device addresses specified in configuration struct.
    for (uint8_t i = 0; i < master_cfg->addr_size; ++i)
    {
        err = ESP_FAIL;
        while (err != ESP_OK)
        {
            err = bmp180_initializer(master_cfg->sda, master_cfg->scl, master_cfg->addresses[i], master_cfg->i2c_frequency);
            if (err != ESP_OK)
            {
                timestamp(timestring);
                log_initerr_master(timestring, master_cfg->addresses[i]);
            }
        }
    }

    // Endless loop to query devices.
    while (1)
    {   
        for (int i = 0; i < 128; ++i)
        {
            if (running && get_bmp180(i)) {
                bmp180_prober(i);
            }
        }
    }
}

// Sets up necessary parameters and starts I2C master task for an experiment run with wifi functionality.
void test_i2c_master_start_wifi(uint8_t sda, uint8_t scl, uint8_t *addresses, size_t addr_size, uint32_t i2c_frequency)
{
    i2c_master_cfg_t *master_cfg = (i2c_master_cfg_t *)calloc(1, sizeof(i2c_master_cfg_t));

    // Connect to wifi.
    init_wireless(STA_SSID, STA_PASSWORD);
    printf("Initialized WiFi.\n");

    master_cfg->sda = sda;
    master_cfg->scl = scl;
    master_cfg->addresses = addresses;
    master_cfg->addr_size = addr_size;
    master_cfg->i2c_frequency = i2c_frequency;

    // Run TCP/IP client task and I2C master task.
    xTaskCreatePinnedToCore(&start_client, "client", 2048, NULL, configMAX_PRIORITIES - 1, NULL, 0);
    xTaskCreatePinnedToCore(&test_i2c_master_wifi, "test_i2c_master_wifi", 8 * 2048, (void *)master_cfg, configMAX_PRIORITIES - 1, NULL, 1);
}

// I2C master task for no-wifi variant of an experiment run.
void test_i2c_master_nowifi(void *pvParameters)
{
    i2c_master_cfg_t *master_cfg = (i2c_master_cfg_t *)pvParameters;

    i2cdev_init();

    timestamp(timestring);
    printf("%sSTART\n", timestring);

    esp_err_t err;

    // Initialize all BMP180 device addresses specified in config struct.
    for (uint8_t i = 0; i < master_cfg->addr_size; ++i)
    {
        err = ESP_FAIL;
        while (err != ESP_OK)
        {
            err = bmp180_initializer(master_cfg->sda, master_cfg->scl, master_cfg->addresses[i], master_cfg->i2c_frequency);
            if (err != ESP_OK)
            {
                timestamp(timestring);
                log_initerr_master(timestring, master_cfg->addresses[i]);
            }
        }
    }

    // Endless querying loop of BMP180s.
    while (1)
    {   
        for (int i = 0; i < 128; ++i)
        {
            if (get_bmp180(i)) {
                bmp180_prober(i);
            }
        }
    }
}

// Sets up I2C master and starts task for no-wifi experiment run variant.
void test_i2c_master_start_nowifi(uint8_t sda, uint8_t scl, uint8_t *addresses, size_t addr_size, uint32_t i2c_frequency)
{
    i2c_master_cfg_t *master_cfg = (i2c_master_cfg_t *)calloc(1, sizeof(i2c_master_cfg_t));

    master_cfg->sda = sda;
    master_cfg->scl = scl;
    master_cfg->addresses = addresses;
    master_cfg->addr_size = addr_size;
    master_cfg->i2c_frequency = i2c_frequency;

    xTaskCreatePinnedToCore(&test_i2c_master_nowifi, "test_i2c_master_nowifi", 8 * 2048, (void *)master_cfg, configMAX_PRIORITIES - 1, NULL, 1);
}

// Sets up and starts all tasks necessary for software I2C based emulation on ESP32 without wifi enabled.
void test_i2c_bitbangslave_nowifi(uint8_t sda, uint8_t scl, uint8_t *addresses, size_t addr_size, uint8_t **update_frames)
{
    // Enable device manager component.
    init_device_manager();

    // Create emulation interfaces for all BMP180 device addresses specified in config struct.
    // Also load up initial update with temperature and pressure curves to emulate throughout experiment.
    for (uint8_t i = 0; i < addr_size; ++i)
    {
        create_emulator(BMP_DEVICE_ID, addresses[i], 0);
        if (update_frames[addresses[i]])
            bmp180em_updater(get_i2c_slave(addresses[i]), (char *)update_frames[addresses[i]]);
    }

    timestamp(timestring);
    printf("%sSTART\n", timestring);

    // Start progressing service, so BMP180 emulators can advance their value curves in time.
    xTaskCreatePinnedToCore(&start_progresser, "progresser", 2048, NULL, configMAX_PRIORITIES - 1, NULL, 0);
    start_i2c_bitbang_slave(sda, scl, 1);
}

// Sets up and starts all tasks necessary for software I2C based emulation on ESP32 with wifi enabled.
void test_i2c_bitbangslave_wifi(uint8_t sda, uint8_t scl)
{
    // Enable device manager.
    init_device_manager();

    // Connect to wifi.
    init_wireless(STA_SSID, STA_PASSWORD);
    printf("Initialized WiFi.\n");

    // Notice: No emulators need to be created and initialized in code, since they can be created/destroyed/initialized/updated via wifi later before the start of the experiment by the controller.

    // Start TCP/IP client for wireless functionality and progresser for time progression functionality of emulators.
    xTaskCreatePinnedToCore(&start_client, "client", 2048, NULL, configMAX_PRIORITIES - 1, NULL, 0);
    xTaskCreatePinnedToCore(&start_progresser, "progresser", 2048, NULL, configMAX_PRIORITIES - 1, NULL, 0);
    start_i2c_bitbang_slave(sda, scl, 1);
}

// Kick of hardware I2C emulator without wifi.
void test_i2c_slave_start_updates(void *pvParameters)
{
    i2c_slave_cfg_t *slave_cfg = (i2c_slave_cfg_t *)pvParameters;

    // Initialize i2cdev component and its services.
    i2cdev_init();

    // Configure hardware emulation interface for BMP180 device according to config struct.
    bmp180_emH_t *emulator = bmp180emH_creater(slave_cfg->port, slave_cfg->sda, slave_cfg->scl, slave_cfg->addr, 0);

    // Initial update for BMP180 interface.
    if (slave_cfg->update_frame)
        bmp180emH_updater(emulator, (char *)slave_cfg->update_frame);

    // Configure I2C hardware port and enable emulation slave mode in i2c module.
    i2c_dev_cfg_port(&emulator->bmp180->i2c_dev);
    i2c_slave_em_en(slave_cfg->port, 1, &bmp180emH_parser, 0x80);
    while (1) vTaskDelay(1);
}

// Setup hardware I2C emulator without wifi.
void test_i2c_hardwareslave_nowifi(i2c_port_t port, uint8_t address, uint8_t sda, uint8_t scl, uint8_t *update_frame)
{
    i2c_slave_cfg_t *slave_cfg = (i2c_slave_cfg_t *)calloc(1, sizeof(i2c_slave_cfg_t));

    slave_cfg->port = port;
    slave_cfg->addr = address;
    slave_cfg->sda = sda;
    slave_cfg->scl = scl;
    slave_cfg->update_frame = update_frame;

    timestamp(timestring);
    printf("%sSTART\n", timestring);

    // Start progresser and hardware slave tasks.
    xTaskCreatePinnedToCore(&start_progresser, "progresser", 2048, NULL, configMAX_PRIORITIES - 1, NULL, 0);
    xTaskCreatePinnedToCore(&test_i2c_slave_start_updates, "i2c_hardware_slave", 2048, (void *)slave_cfg, configMAX_PRIORITIES - 1, NULL, 1);
}

// Kick of hardware I2C emulator with wifi.
void test_i2c_slave_start_noupdates(void *pvParameters)
{
    i2c_slave_cfg_t *slave_cfg = (i2c_slave_cfg_t *)pvParameters;
    i2cdev_init();
    bmp180_emH_t *emulator = bmp180emH_creater(slave_cfg->port, slave_cfg->sda, slave_cfg->scl, slave_cfg->addr, 0);
    i2c_dev_cfg_port(&emulator->bmp180->i2c_dev);
    i2c_slave_em_en(slave_cfg->port, 1, &bmp180emH_parser, 0x80);
    while (1) vTaskDelay(1);
}

// Setup hardware I2C emulator with wifi functionality.
void test_i2c_hardwareslave_wifi(i2c_port_t port, uint8_t sda, uint8_t scl, uint8_t address)
{
    i2c_slave_cfg_t *slave_cfg = (i2c_slave_cfg_t *)calloc(1, sizeof(i2c_slave_cfg_t));

    // Connect to wifi.
    init_wireless(STA_SSID, STA_PASSWORD);
    printf("Initialized WiFi.\n");

    slave_cfg->port = port;
    slave_cfg->addr = address;
    slave_cfg->sda = sda;
    slave_cfg->scl = scl;

    // Start client, progresser and hardware I2C module functionality.
    xTaskCreatePinnedToCore(&start_client, "client", 2048, NULL, configMAX_PRIORITIES - 1, NULL, 0);
    xTaskCreatePinnedToCore(&start_progresser, "progresser", 2048, NULL, configMAX_PRIORITIES - 1, NULL, 0);
    xTaskCreatePinnedToCore(&test_i2c_slave_start_noupdates, "i2c_hardware_slave", 2048, (void *)slave_cfg, configMAX_PRIORITIES - 1, NULL, 1);
}

// Generate and return a complete BMP180 update frame for all temperature and pressure values in the range of the BMP180.
// Has around 20 minutes of emulation duration.
uint8_t *complete_update(char *start, time_t start_seconds) {
    double *temperatures = NULL;
    uint64_t *pressures = NULL;
    time_t *temp_times = NULL;
    time_t *press_times = NULL;
    uint16_t temp_len = 1201;
    uint16_t press_len = 1201;
    time_t stamp = 0;

    test_generate_temperatures(-40.0, 0.1, temp_len, &temperatures);
    test_generate_pressures(30000, 67, press_len, &pressures);

    if(start) {
        stampfromstring(start, &stamp);
        test_generate_times(stamp, 1, temp_len, &temp_times);
        test_generate_times(stamp, 1, press_len, &press_times);
    } else {
        test_generate_times(start_seconds, 1, temp_len, &temp_times);
        test_generate_times(start_seconds, 1, press_len, &press_times);
    }

    uint8_t *frame_temp = test_generate_bmp180em_update_frame(temperatures, temp_times, temp_len, pressures, press_times, press_len);

    free(temperatures);
    free(pressures);
    free(temp_times);
    free(press_times);

    return frame_temp;
}

// Generate a shorter update frame of 7.5 minutes duration.
uint8_t *seven_point_five_minutes(char *start, time_t start_seconds) {
    double *temperatures = NULL;
    uint64_t *pressures = NULL;
    time_t *temp_times = NULL;
    time_t *press_times = NULL;
    uint16_t temp_len = 451;
    uint16_t press_len = 451;
    time_t stamp = 0;

    test_generate_temperatures(-2.5, 0.1, temp_len, &temperatures);
    test_generate_pressures(40000, 67, press_len, &pressures);

    if(start) {
        stampfromstring(start, &stamp);
        test_generate_times(stamp, 1, temp_len, &temp_times);
        test_generate_times(stamp, 1, press_len, &press_times);
    } else {
        test_generate_times(start_seconds, 1, temp_len, &temp_times);
        test_generate_times(start_seconds, 1, press_len, &press_times);
    }

    uint8_t *frame_temp = test_generate_bmp180em_update_frame(temperatures, temp_times, temp_len, pressures, press_times, press_len);

    free(temperatures);
    free(pressures);
    free(temp_times);
    free(press_times);

    return frame_temp;
}

// Generate an even shorter update frame of 5 minutes duration.
uint8_t *five_minutes(char *start, time_t start_seconds) {
    double *temperatures = NULL;
    uint64_t *pressures = NULL;
    time_t *temp_times = NULL;
    time_t *press_times = NULL;
    uint16_t temp_len = 301;
    uint16_t press_len = 301;
    time_t stamp = 0;

    test_generate_temperatures(5.0, 0.1, temp_len, &temperatures);
    test_generate_pressures(50000, 67, press_len, &pressures);

    if(start) {
        stampfromstring(start, &stamp);
        test_generate_times(stamp, 1, temp_len, &temp_times);
        test_generate_times(stamp, 1, press_len, &press_times);
    } else {
        test_generate_times(start_seconds, 1, temp_len, &temp_times);
        test_generate_times(start_seconds, 1, press_len, &press_times);
    }

    uint8_t *frame_temp = test_generate_bmp180em_update_frame(temperatures, temp_times, temp_len, pressures, press_times, press_len);

    free(temperatures);
    free(pressures);
    free(temp_times);
    free(press_times);

    return frame_temp;
}

// Configure ESP32 logger for complete update logging and sanity checks.
void log_set_complete() {
    uint16_t size = 1201;
    double *temperatures = NULL;
    uint64_t *pressures = NULL;

    test_generate_temperatures(-40.0, 0.1, size, &temperatures);
    test_generate_pressures(30000, 67, size, &pressures);
    set_log_curve(size, temperatures, pressures);

    free(temperatures);
    free(pressures);
}

// Configure ESP32 logger for 7.5 update logging and sanity checks.
void log_set_seven_point_five() {
    uint16_t size = 451;
    double *temperatures = NULL;
    uint64_t *pressures = NULL;

    test_generate_temperatures(-2.5, 0.1, size, &temperatures);
    test_generate_pressures(40000, 67, size, &pressures);
    set_log_curve(size, temperatures, pressures);

    free(temperatures);
    free(pressures);
}

// Configure ESP32 logger for 5 update logging and sanity checks.
void log_set_five() {
    uint16_t size = 301;
    double *temperatures = NULL;
    uint64_t *pressures = NULL;

    test_generate_temperatures(5.0, 0.1, size, &temperatures);
    test_generate_pressures(50000, 67, size, &pressures);
    set_log_curve(size, temperatures, pressures);

    free(temperatures);
    free(pressures);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * This component contains two main functions.
 * 
 * One for an I2C master and one for an I2C slave device. Both are needed for an experiment run.
 * Every device can only contain one of those at a time, therefore the other must be commented out.
*/

/* SLAVE MAIN */
void app_main()
{
    set_timezone();
    printf("Starting Main-Application.\n");

    // Arrays used for testruns
    uint8_t **updates = (uint8_t **) calloc(1, 128 * sizeof(uint8_t *));
    uint8_t *addresses = (uint8_t *) calloc(1, 128);
    uint8_t *frame = complete_update(NULL, 30);

    for(int i = 0; i < 128; i++) {
        addresses[i] = i;
        updates[i] = frame;
    }
    
    // Desired test configuration can be commented in. Rest must be commented out.
    // log_set_complete();
    test_i2c_bitbangslave_wifi(GPIO_NUM_25, GPIO_NUM_26);
    // test_i2c_bitbangslave_nowifi(GPIO_NUM_25, GPIO_NUM_26, addresses, 1, updates);
    // test_i2c_hardwareslave_wifi(I2C_NUM_0, GPIO_NUM_25, GPIO_NUM_26, 0x00);
    // test_i2c_hardwareslave_nowifi(I2C_NUM_0, 0x00, GPIO_NUM_25, GPIO_NUM_26, frame);

    vTaskDelete(NULL);
}

/* MASTER MAIN */
void app_main()
{
    set_timezone();
    printf("Starting Main-Application.\n");

    // Configuration Array for testing.
    uint8_t *addresses = (uint8_t *) calloc(1, 128);
    for(int i = 0; i < 128; i++) addresses[i] = i;

    // Desired master configuration for test run can be commented in.
    // Other must be commented out.
    // log_set_complete();
    test_i2c_master_start_wifi(GPIO_NUM_25, GPIO_NUM_26, addresses, 1, 100000);
    // test_i2c_master_start_nowifi(GPIO_NUM_25, GPIO_NUM_26, addresses, 1, 100000);

    vTaskDelete(NULL);
}