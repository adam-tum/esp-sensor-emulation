/**
 * Logger component to log results of emulation accuracy on ESP32.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <sys/time.h>

#include "esp_err.h"
#include "esp_heap_caps.h"

#include "timesync.h"
#include "logger.h"

// BMP180 calibration parameters for inverse function.
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

// OSS Level for inverse function
#define LOG_OSS 0

// Old logger was used, since verbose logger implementation is too slow.
#define OLD_LOGGER 1
// #define VERBOSE_LOGGER 1

#ifndef OLD_LOGGER
// Reference values for current experiment
static DRAM_ATTR size_t xsize = 0;
static DRAM_ATTR int32_t *utemperatures = NULL;
static DRAM_ATTR double *ntemperatures = NULL;
static DRAM_ATTR uint64_t *upressures = NULL;
static DRAM_ATTR uint64_t *npressures = NULL;

// Emulator initialization values
static DRAM_ATTR int32_t init_utemp = 0;
static DRAM_ATTR double init_temp = 0.0;
static DRAM_ATTR uint64_t init_upress = 0;
static DRAM_ATTR uint64_t init_press = 0;

// Statistical variables
static DRAM_ATTR uint32_t temp_counter = 0;
static DRAM_ATTR uint32_t wtemp_counter = 0;
static DRAM_ATTR uint32_t press_counter = 0;
static DRAM_ATTR uint32_t wpress_counter = 0;
static DRAM_ATTR uint32_t init_counter = 0;
static DRAM_ATTR uint32_t terr_counter = 0;
static DRAM_ATTR uint32_t perr_counter = 0;
static DRAM_ATTR uint32_t merr_counter = 0;

// Binary Search variables
static DRAM_ATTR int32_t left = 0;
static DRAM_ATTR int32_t right = 0;
static DRAM_ATTR int32_t middle = 0;

static esp_err_t bmp180_inverse(double temperature, int32_t *ut, uint64_t pressure, uint64_t *up, int oss)
{
    double UT, X1, B5, T;

    T = (temperature * 10);
    B5 = (T * 16) - 8;
    X1 = 0.5 * (B5 + sqrt(pow(B5, 2) + (2 * B5 * DEF_MD) - (8192 * DEF_MC) + pow(DEF_MD, 2)) - DEF_MD);
    UT = ((X1 * 32768) / DEF_AC5) + DEF_AC6;
    *ut = (int32_t)ceil(UT);

    if (up)
    {
        double UP, PREP, X1, X2, X3, B3, B4, B6, B7;

        B6 = B5 - 4000;
        X1 = (DEF_B2 * (B6 * B6 / pow(2, 12))) / pow(2, 11);
        X2 = DEF_AC2 * B6 / pow(2, 11);
        X3 = X1 + X2;
        B3 = (((DEF_AC1 * 4 + X3) * pow(2, oss)) + 2) / 4;
        X1 = DEF_AC3 * B6 / pow(2, 13);
        X2 = (DEF_B1 * (B6 * B6 / pow(2, 12))) / pow(2, 16);
        X3 = ((X1 + X2) + 2) / 4;
        B4 = DEF_AC4 * (unsigned long)(X3 + 32768) / pow(2, 15);
        PREP = (16384 * (sqrt((194432 * pressure) + 1084090937729) - 1041219)) / 1519;
        B7 = (PREP * B4) / 2;
        UP = (B7 / (50000 / pow(2, oss))) + B3;
        *up = ((uint64_t)UP) << (8 - oss);
    }
    return ESP_OK;
}

// Logs uncompensated temperature but performs sanitycheck to only log erroneous and unexpected values.
static uint8_t log_utemp_sanitycheck(int32_t utemperature) {
    if(utemperature == init_utemp) return 1;
    left = 0;
    right = xsize - 1;
    while(left <= right) {
        middle = left + (right-left) / 2;
        if(utemperatures[middle] == utemperature) return 1;
        if(utemperatures[middle] < utemperature) left = middle + 1;
        else right = middle - 1;
    }
    return 0;
}
// Logs temperature but performs sanitycheck to only log erroneous and unexpected values.
static uint8_t log_temp_sanitycheck(double temperature) {
    if(temperature == init_temp) return 1;
    left = 0;
    right = xsize - 1;
    while(left <= right) {
        middle = left + (right-left) / 2;
        if(ntemperatures[middle] == temperature) return 1;
        if(ntemperatures[middle] < temperature) left = middle + 1;
        else right = middle - 1;
    }
    return 0;
}

// Logs uncompensated pressure but performs sanitycheck to only log erroneous and unexpected values.
static uint8_t log_upress_sanitycheck(uint64_t upressure) {
    if(upressure == init_upress) return 1;
    left = 0;
    right = xsize - 1;
    while(left <= right) {
        middle = left + (right-left) / 2;
        if(upressures[middle] == upressure) return 1;
        if(upressures[middle] < upressure) left = middle + 1;
        else right = middle - 1;
    }
    return 0;
}

// Logs pressure but performs sanitycheck to only log erroneous and unexpected values.
static uint8_t log_press_sanitycheck(uint64_t pressure) {
    if(pressure == init_press) return 1;
    left = 0;
    right = xsize - 1;
    while(left <= right) {
        middle = left + (right-left) / 2;
        if(npressures[middle] == pressure) return 1;
        if(npressures[middle] < pressure) left = middle + 1;
        else right = middle - 1;
    }
    return 0;
}

// Logs temperature for I2C master in experiment.
void log_temp_master(char *timestring, uint8_t address, double temperature, int32_t utemperature) {
    ++temp_counter;
    if(log_utemp_sanitycheck(utemperature)) return;
    ++wtemp_counter;
#ifdef VERBOSE_LOGGER
    printf(DRAM_STR("%sA0x%02XUT%iT%5.2f\n"), timestring, address, utemperature, temperature);
#endif
}

// Logs pressure for I2C master in experiment.
void log_press_master(char *timestring, uint8_t address, uint64_t pressure, uint64_t upressure) {
    ++press_counter;
    if(log_upress_sanitycheck(upressure)) return;
    ++wpress_counter;
#ifdef VERBOSE_LOGGER
        printf(DRAM_STR("%sA0x%02XUP%uP%u\n"), timestring, address, upressure, pressure);
#endif
}

// Logs temperature for I2C slave in experiment.
void log_temp_slave(char *timestring, uint8_t address, double temperature) {
    ++temp_counter;
    if(log_temp_sanitycheck(temperature)) return;
    ++wtemp_counter;
#ifdef VERBOSE_LOGGER
    printf(DRAM_STR("%sA0x%02XT%5.2f\n"), timestring, address, temperature);
#endif
}

// Logs pressure for I2C slave in experiment.
void log_press_slave(char *timestring, uint8_t address, uint64_t pressure) {
    ++press_counter;
    if(log_press_sanitycheck(pressure)) return;
    ++wpress_counter;
#ifdef VERBOSE_LOGGER
    printf(DRAM_STR("%sA0x%02XP%llu\n"), timestring, address, pressure);
#endif
}

// Logs I2C error occured during temperature measurement initiated by master.
void log_tmeaserr_master(char *timestring, uint8_t address) {
    ++terr_counter;
#ifdef VERBOSE_LOGGER
    printf(DRAM_STR("%sA0x%02XTMEASERROR\n"), timestring, address);
#endif
}

// Logs I2C error occured during pressure measurement initiated by master.
void log_pmeaserr_master(char *timestring, uint8_t address) {
    ++perr_counter;
#ifdef VERBOSE_LOGGER
    printf(DRAM_STR("%sA0x%02XPMEASERROR\n"), timestring, address);
#endif
}

// Logs I2C error occured during any other kind of I2C transaction during measurement.
void log_measerr_master(char *timestring, uint8_t address) {
    ++merr_counter;
#ifdef VERBOSE_LOGGER
    printf(DRAM_STR("%sA0x%02XMEASERROR\n"), timestring, address);
#endif
}

// Logs error when initializing I2C slave device.
void log_initerr_master(char *timestring, uint8_t address) {
    ++init_counter;
#ifdef VERBOSE_LOGGER
    printf(DRAM_STR("%sA0x%02XINITERROR\n"), timestring, address);
#endif
}

// Logs counter for correctly and incorrectly measured temperatures during experiment.
void log_temp_count() {
    printf(DRAM_STR("TEMPCOUNT:%u;WRONGTEMPS:%u\n"), temp_counter, wtemp_counter);
}

// Logs counter for correctly and incorrectly measured pressures during experiment.
void log_press_count() {
    printf(DRAM_STR("PRESSCOUNT:%u;WRONGPRESSES:%u\n"), press_counter, wpress_counter);
}

// Logs counter for initialization errors during measurement.
void log_ierr_count() {
    printf(DRAM_STR("INITERRCOUNT:%u\n"), init_counter);
}

// Logs counter for temperature errors during measurement.
void log_terr_count() {
    printf(DRAM_STR("TERRCOUNT:%u\n"), terr_counter);
}

// Logs counter for pressure errors during measurement.
void log_perr_count() {
    printf(DRAM_STR("PERRCOUNT:%u\n"), perr_counter);
}

// Logs counter for miscellaneous errors during measurement.
void log_err_count() {
    printf(DRAM_STR("MERRCOUNT:%u\n"), merr_counter);
}

// Set expected temperature and pressure curves for experiment run. Is needed to perform sanitychecks in verbose logging.
void set_log_curve(size_t size, double *temperatures, uint64_t *pressures)
{
    ntemperatures = (double *)heap_caps_malloc((size) * sizeof(double), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    npressures = (uint64_t *)heap_caps_malloc((size) * sizeof(uint64_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    utemperatures = (int32_t *)heap_caps_malloc((size) * sizeof(int32_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    upressures = (uint64_t *)heap_caps_malloc((size) * sizeof(uint64_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);

    init_temp = 20.0;
    init_press = 101325;
    bmp180_inverse(init_temp, &init_utemp, init_press, &init_upress, LOG_OSS);
    init_upress >>= 8 - LOG_OSS;

    for (int i = 0; i < size; ++i)
    {
        ntemperatures[i] = temperatures[i];
        npressures[i] = pressures[i];
        bmp180_inverse(ntemperatures[i], &utemperatures[i], npressures[i], &upressures[i], LOG_OSS);
        upressures[i] >>= 8 - LOG_OSS;
    }
    xsize = size;
}

// Delete set log curve.
void delete_log_curve(void)
{
    free(ntemperatures);
    free(utemperatures);
    free(npressures);
    free(upressures);
}
#else

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * In this section, the old logging (the one used for the final experiment results) is implemented.
 * No sanitychecks are performed to reduce logging influence on the final measurement results.
*/

// Log uncompensated and converted temperature received by master from slave.
void log_temp_master(char *timestring, uint8_t address, double temperature, int32_t utemperature) {
    printf(DRAM_STR("%sA0x%02XUT%iT%5.2f\n"), timestring, address, utemperature, temperature);
}

// Log uncompensated and converted pressure received by master from slave.
void log_press_master(char *timestring, uint8_t address, uint64_t pressure, uint64_t upressure) {
    printf(DRAM_STR("%sA0x%02XUP%lluP%llu\n"), timestring, address, upressure, pressure);
}

// Log temperature set by slave.
void log_temp_slave(char *timestring, uint8_t address, double temperature) {
    printf(DRAM_STR("%sA0x%02XT%5.2f\n"), timestring, address, temperature);
}

// Log pressure set by slave.
void log_press_slave(char *timestring, uint8_t address, uint64_t pressure) {
    printf(DRAM_STR("%sA0x%02XP%llu\n"), timestring, address, pressure);
}

// Log temperature measurement error occured on master queuing slave.
void log_tmeaserr_master(char *timestring, uint8_t address) {
    printf(DRAM_STR("%sA0x%02XTMEASERROR\n"), timestring, address);
}

// Log pressure measurement error occured on master queuing slave.
void log_pmeaserr_master(char *timestring, uint8_t address) {
    printf(DRAM_STR("%sA0x%02XPMEASERROR\n"), timestring, address);
}

// Log miscellaneous error occured during master I2C transaction with slave.
void log_measerr_master(char *timestring, uint8_t address) {
    printf(DRAM_STR("%sA0x%02XMEASERROR\n"), timestring, address);
}

// Log initialization error occured during I2C communication to initialize slave device.
void log_initerr_master(char *timestring, uint8_t address) {
    printf(DRAM_STR("%sA0x%02XINITERROR\n"), timestring, address);
}

/* Placeholder function "implementations" for old logging capabilities to work with same header file. */
void log_temp_count() {
    /* Nothing to do here. */
}

void log_press_count() {
    /* Nothing to do here. */
}

void log_ierr_count() {
    /* Nothing to do here. */
}

void log_terr_count() {
    /* Nothing to do here. */
}

void log_perr_count() {
    /* Nothing to do here. */
}

void log_err_count() {
    /* Nothing to do here. */
}

void set_log_curve(size_t size, double *temperatures, uint64_t *pressures)
{
    /* Nothing to do here. */
}

void delete_log_curve(void)
{
    /* Nothing to do here. */
}

#endif