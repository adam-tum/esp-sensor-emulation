/**
 * Timesync component used for time synchronization and generation of current timestamps of system.
 * 
 * Tiome synchronization is performed via SNTP protocol over lwIP stack. 
 * Network connection needs to be established beforehand.
*/

#ifndef __TIMESYNC_H__
#define __TIMESYNC_H__

#include "esp_attr.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Sets timezone to german time (hardcoded for now).
*/
void set_timezone(void);

/**
 * @brief Returns a timestamp in relation to current system time extracted from a string.
 * @param timestring        String that contains time.
 * @param[out] time         Timestamp pointer where return value is stored.
*/
void stampfromstring(char *timestring, time_t *time);

/**
 * @brief Converts timeval struct information to human readable string.
 * @param[out] timestring   String from timeval struct.
 * @param moment            Timeval struct time information.    
*/
void stringtime(char *timestring, struct timeval *moment);

/**
 * @brief Gets current timestamp and converts it to printable string.
 * @param[out] timestring        Printable timestring of current time.
*/
void timestamp(char *timestring);

/**
 * @brief Initiates synchronization of time via SNTP on this system.
*/
void synchronize(void);

#ifdef __cplusplus
}
#endif

/**@}*/

#endif /* __TIMESYNC_H__ */
