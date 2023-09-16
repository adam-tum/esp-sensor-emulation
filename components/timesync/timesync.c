/**
 * Timesync component used for time synchronization and generation of current timestamps of system.
 * 
 * Tiome synchronization is performed via SNTP protocol over lwIP stack. 
 * Network connection needs to be established beforehand.
*/

#include <string.h>
#include <time.h>
#include <sys/time.h>

#include "freertos/FreeRTOSConfig.h"
#include "freertos/portmacro.h"

#include "esp_system.h"
#include "esp_log.h"
#include "esp_attr.h"
#include "esp_sntp.h"
#include "lwip/ip_addr.h"

// static const char *TAG = "timesync_task";
static DRAM_ATTR struct timeval tv;
static DRAM_ATTR struct tm timeinfo;

/* GERMAN TZ STRING: |  Germany    |	CEST/CET    |	UTC +2	CEST-1CET,M3.2.0/2:00:00,M11.1.0/2:00:00    |*/
void set_timezone(void) {
    // Set timezone to German Time and print local time
    setenv("TZ", "CET-1CEST,M3.5.0,M10.5.0/3", 1);
    tzset();
}

void stampfromstring(char *timestring, time_t *time) {
    memset(&timeinfo, 0x0, sizeof(struct tm));
    timeinfo.tm_isdst = -1;
    strptime(timestring, "%Y-%m-%d %T", &timeinfo);
    *time = mktime(&timeinfo);
}

void stringtime(char *timestring, struct timeval *moment) {
    localtime_r(&moment->tv_sec, &timeinfo);
    strftime(timestring, 64, "%T", &timeinfo);
    snprintf(&timestring[8], 5, ".%03lu", (moment->tv_usec / 1000L));
}

void timestamp(char *timestring) {
	gettimeofday(&tv, NULL);
    stringtime(timestring, &tv);
}

void time_sync_notification_cb(struct timeval *tv)
{
    // ESP_LOGI(TAG, "Time has been synchronized!");
    printf("Synchronized Time.\n");
}
 
void synchronize(void)
{
    // ESP_LOGI(TAG, "Initializing and starting SNTP");
    sntp_setoperatingmode(SNTP_OPMODE_POLL);
    sntp_setservername(0, "pool.ntp.org");
    sntp_set_time_sync_notification_cb(time_sync_notification_cb);
    sntp_init();

    // Wait for time to be set.
    int retry = 0;
    const int retry_count = 10;
    while (sntp_get_sync_status() == SNTP_SYNC_STATUS_RESET && ++retry < retry_count) {
        // ESP_LOGI(TAG, "Waiting for system time to be set... (%d/%d)", retry, retry_count);
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }

    sntp_stop();
}