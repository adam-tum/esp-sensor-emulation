/**
 * Connection driver that simplifies connecting to a WiFi network with the ESP32.
 * 
 * Provides an interface that allows to simply call a "connect" and "disconnect" function to a private network.
*/

#include <string.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "esp_event.h"
#include "esp_event_legacy.h"
#include "esp_wifi.h"
#include "esp_wifi_default.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_err.h"
#include "esp_attr.h"

#include <lwip/netdb.h>
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"

#include "connect.h"

typedef struct {
    char *STA_SSID;
    char *STA_PASSWORD;
} wifi_params_t ;

// static const char * TAG = "connect_wifi";

static wifi_params_t param_config = { 0 };
static esp_netif_t *netif_STA = NULL;
static esp_ip4_addr_t s_ip_addr;

static SemaphoreHandle_t semph_ip_addr;

// Event handler for when the WiFi network is disconnected. Tries to reconnect upon disconnection.
static void sta_disconnected(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    // ESP_LOGI(TAG, "Wi-Fi disconnected, trying to reconnect...\n");
    esp_err_t err = esp_wifi_connect();
    if (err == ESP_ERR_WIFI_NOT_STARTED) return;
    ESP_ERROR_CHECK(err);
}

// Event handler for when the ESP32 gets an IP assigned.
static void sta_got_ip(void *arg, esp_event_base_t event_base,
                      int32_t event_id, void *event_data)
{
    ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
    // ESP_LOGI(TAG, "Got IPv4 from external AP: " IPSTR"\n", IP2STR(&event->ip_info.ip));
    memcpy(&s_ip_addr, &event->ip_info.ip, sizeof(s_ip_addr));
    xSemaphoreGive(semph_ip_addr);
}

// Start a WiFi conncetion to the network with the provided parameters.
static void wifi_start(esp_netif_t **netif_sta, bool static_ip, const char *ip, const char *gw, const char *netmask)
{
    // Create STA netif structure.
    esp_netif_inherent_config_t esp_netif_sta_config = ESP_NETIF_INHERENT_DEFAULT_WIFI_STA();
    *netif_sta = esp_netif_create_wifi(WIFI_IF_STA, &esp_netif_sta_config);
    esp_wifi_set_default_wifi_sta_handlers();
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, &sta_disconnected, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &sta_got_ip, NULL));

    if(static_ip) {
        ESP_ERROR_CHECK(esp_netif_dhcpc_stop(*netif_sta));

        esp_netif_ip_info_t ip_info;
        inet_pton(AF_INET, ip, &ip_info.ip);
        inet_pton(AF_INET, gw, &ip_info.gw);
        inet_pton(AF_INET, netmask, &ip_info.netmask);

        esp_netif_set_ip_info(*netif_sta, &ip_info);
    }

    // Initialize structure with default values.
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    
    // Select authentication method.
    wifi_config_t sta_config = {
        .sta = {
            .scan_method = WIFI_FAST_SCAN,
            .sort_method = WIFI_CONNECT_AP_BY_SIGNAL,
            .threshold.rssi = -127,
            .threshold.authmode = WIFI_AUTH_OPEN,
        },
    };
    memcpy(sta_config.sta.ssid, param_config.STA_SSID, 32);
    memcpy(sta_config.sta.password, param_config.STA_PASSWORD, 64);
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &sta_config));

    // Connect.
    ESP_ERROR_CHECK(esp_wifi_start());
    esp_wifi_connect();
    semph_ip_addr = xSemaphoreCreateCounting(1, 0);
}

// End connection to a WiFi network.
static void wifi_stop(void)
{
    if(netif_STA) {
        ESP_ERROR_CHECK(esp_event_handler_unregister(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, &sta_disconnected));
        ESP_ERROR_CHECK(esp_event_handler_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, &sta_got_ip));
    }

    esp_err_t err = esp_wifi_stop();
    if (err == ESP_ERR_WIFI_NOT_INIT) return;

    ESP_ERROR_CHECK(err);
    ESP_ERROR_CHECK(esp_wifi_deinit());
    if(netif_STA) {
        ESP_ERROR_CHECK(esp_wifi_clear_default_wifi_driver_and_handlers(netif_STA));
        esp_netif_destroy(netif_STA);
        netif_STA = NULL;
    }
}

esp_err_t wifi_connect(char * sta_ssid, char *sta_pass, bool static_ip, const char *ip, const char *gw, const char *netmask)
{
    param_config.STA_SSID = sta_ssid;
    param_config.STA_PASSWORD = sta_pass;
    wifi_start(&netif_STA, static_ip, ip, gw, netmask);
    ESP_ERROR_CHECK(esp_register_shutdown_handler(&wifi_stop));
    // ESP_LOGI(TAG, "Waiting for IP address...");
    xSemaphoreTake(semph_ip_addr, portMAX_DELAY);
    return ESP_OK;
}

esp_err_t wifi_disconnect(void)
{
    vSemaphoreDelete(semph_ip_addr);
    semph_ip_addr = NULL;
    wifi_stop();
    ESP_ERROR_CHECK(esp_unregister_shutdown_handler(&wifi_stop));
    return ESP_OK;
}