/**
 * Connection driver that simplifies connecting to a WiFi network with the ESP32.
 * 
 * Provides an interface that allows to simply call a "connect" and "disconnect" function to a private network.
*/

#ifndef __CONNECT_H__
#define __CONNECT_H__

#include "esp_err.h"
#include "esp_attr.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Connects the ESP32 to a WiFi network.
 * @param sta_ssid          The network SSID as a string.
 * @param sta_pass          The network password as a string.
 * @param static_ip         Boolean value to determine if ESP's IP should be static.
 * @param ip                Static ip if static_ip is true.
 * @param gw                Gateway.
 * @param netmask           Netmask.
*/
esp_err_t wifi_connect(char * sta_ssid, char *sta_pass, bool static_ip, const char *ip, const char *gw, const char *netmask);

/**
 * @brief Disconnects the ESP32 from a previously connected-to WiFi network.
*/
esp_err_t wifi_disconnect(void);

#ifdef __cplusplus
}
#endif

/**@}*/

#endif /* __CONNECT_H__ */
