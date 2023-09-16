/**
 * This TCP/IP client implementation is based on the ESP32's lwIP stack.
 * 
 * It contains basic socket functionality that is executed by a dedicated process in the final application.
 * This way, an emulator is able to receive wireless data during an experiment.
 * 
 * This includes updates and simple commands.
*/

#include "sdkconfig.h"
#include <string.h>
#include <unistd.h>
#include <sys/socket.h>
#include <errno.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <esp_heap_caps.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_attr.h"

#include "logger.h"
#include "i2c_bitbang.h"
#include "device_manager.h"
#include "client.h"
#include "timesync.h"

// Static IP address of controller (server socket) and port
#define HOST_IP_ADDR ""
#define PORT 3333

// Receive buffer size for ESP32
#define BUF_SIZE 32768

// static const char *TAG = "client_task";

static DRAM_ATTR const char *ack_str = "ACK";

// All devices registered to wireless updating progress
updater sensor_updaters[128] = { 0 };
uint8_t running = 0;

void *get_bmp180( uint8_t address);
void set_bmp180( uint8_t address, void *sensor);

void set_update_slave(uint8_t address, updater update) {
    // Disable interrupts during device (de-)registration to prevent concurrent access to same address
    taskDISABLE_INTERRUPTS();
    sensor_updaters[address] = update;
    taskENABLE_INTERRUPTS();
}

// Robust sending function. Loops and blocks until all data is sent over socket.
static int robust_send(int sock, const char *msg, uint16_t len) {
    uint16_t sent = 0;
    while (len - sent > 0) {
        sent += send(sock, msg + sent, len - sent, 0);
    }
    return 0;
}

// Receive until minimum length "minlen" bytes have been received.
static int recv_until(int sock, char *memory, size_t minlen, int flags) {
    size_t received = 0;
    int len;
    while(minlen - received > 0) {
        len = recv(sock, memory + received, minlen - received, 0);
        if(len <= 0) return -1;
        received += len;
    }
    return received;
}

// Handler for ESP32 socket connection.
static void IRAM_ATTR handler(int sock) {
    char timestring[64];
    // Allocate memory for receive buffer.
    char *rx_buffer = heap_caps_malloc(BUF_SIZE * sizeof(char), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);

    // Temporary buffers for mac address and mac-address string.
    uint8_t mac[6] = { 0 };
    char mac_str[18] = { 0 };
    
    esp_base_mac_addr_get(mac);
    snprintf(mac_str, 18, DRAM_STR("%02X:%02X:%02X:%02X:%02X:%02X"), mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    printf(DRAM_STR("Connected to Controller.\n"));
    // Receiving loop.
    while (1) {
        int len = recv(sock, rx_buffer, BUF_SIZE - 1, 0);
        if(len <= 0 || len >= BUF_SIZE ) return;
        rx_buffer[len] = 0;

        // MAC command queries mac-address of ESP.
        if (strcmp(DRAM_STR("MAC"), rx_buffer) == 0) {
            robust_send(sock, mac_str, 17);
            robust_send(sock, ack_str, 3);
        // MCONFIG command configures the ESP32 as an I2C master.
        } else if(strcmp(DRAM_STR("MCONFIG"), rx_buffer) == 0) {
            robust_send(sock, ack_str, 3);
            len = recv_until(sock, rx_buffer, 1, 0); // Receive Address, 8 Bit.
            if(len <= 0) goto NACK;
            robust_send(sock, ack_str, 3);
            uint8_t address = *(uint8_t *) rx_buffer;
            if(address & 0x80) {
                set_bmp180(address & 0x7f, NULL);
            } else {
                set_bmp180(address, (void *) 0x1);
            }
        // SCONFIG configures the ESP32 as an I2C slave.
        // According to received device-id and address, create or destroy emulator.
        } else if(strcmp(DRAM_STR("SCONFIG"), rx_buffer) == 0) {
            robust_send(sock, ack_str, 3);
            len = recv_until(sock, rx_buffer, 3, 0); // Receive device-id (BMP180 == 0) 16_bit and Address, 8 Bit.
            if(len <= 0) goto NACK;
            robust_send(sock, ack_str, 3);
            uint16_t device = *(uint16_t *) rx_buffer;
            uint8_t address = (uint8_t) rx_buffer[2];
            if(address & 0x80) {
                destroy_emulator(address & 0x7f);
            } else {
                create_emulator(device, address, 0);
            }
        // SYNC synchronizes the time on the system.
        } else if(strcmp(DRAM_STR("SYNC"), rx_buffer) == 0) {
            synchronize();
            timestamp(timestring);
            robust_send(sock, timestring, 12);
            robust_send(sock, ack_str, 3);
        // START logs starting point of an experiment.
        } else if(strcmp(DRAM_STR("START"), rx_buffer) == 0) {
            timestamp(timestring);
            printf(DRAM_STR("%sSTART\n"), timestring);
            running = 1;
            robust_send(sock, ack_str, 3);
        // STOP logs stopping point and other information of an experiment.
        } else if(strcmp(DRAM_STR("STOP"), rx_buffer) == 0) {
            timestamp(timestring);
            running = 0;
            printf(DRAM_STR("%sSTOP\n"), timestring);
            log_temp_count();
            log_press_count();
            log_ierr_count();
            log_err_count();
            log_terr_count();
            log_perr_count();
            robust_send(sock, ack_str, 3);
        // DATA triggers the receival of an update frame.
        } else if(strcmp(DRAM_STR("DATA"), rx_buffer) == 0) {
            robust_send(sock, ack_str, 3); // Send Acknowledgement to receive frame-length

            len = recv_until(sock, rx_buffer, 2, 0); // Receive frame-length
            if(len <= 0) goto NACK;

            uint16_t frame_length = *(uint16_t *) rx_buffer;
            robust_send(sock, ack_str, 3); // Send Acknowledgement to receive data-frame

            len = recv_until(sock, rx_buffer, frame_length, 0); // Receive data-frame of at least length frame-length
            if(len <= 0) goto NACK;
            robust_send(sock, ack_str, 3); // Send Acknowledgement to complete transaction.

            // Parse update frame and update all emulators it includes.
            uint8_t emulators = *(uint8_t *) rx_buffer;

            uint16_t frame_index = 1;
            for(int i = 0; i < emulators; ++i) {
                uint8_t address = *(uint8_t *) (&rx_buffer[frame_index]);
                frame_index += 1;

                if(!sensor_updaters[address]) goto NACK;
                sensor_updaters[address](get_i2c_slave(address), &rx_buffer[frame_index]);
            }

        } else  {
            NACK:
            robust_send(sock, "NACK", 4);
        }
    }
    return;
}

void start_client( void *pvParameters) {
    char host_ip[] = HOST_IP_ADDR;
    int addr_family = 0;
    int ip_protocol = 0;

    // Try to connect to controller (server socket) loop.
    while (1) {
        struct sockaddr_in dest_addr;
        inet_pton(AF_INET, host_ip, &dest_addr.sin_addr);
        dest_addr.sin_family = AF_INET;
        dest_addr.sin_port = htons(PORT);
        addr_family = AF_INET;
        ip_protocol = IPPROTO_IP;

        int sock =  socket(addr_family, SOCK_STREAM, ip_protocol);
        if (sock < 0) {
            // ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
            goto CLEAN_UP;
        }
        // ESP_LOGI(TAG, "Socket created, connecting to %s:%d", host_ip, PORT);

        int err = connect(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
        if (err != 0) {
            // ESP_LOGE(TAG, "Socket unable to connect: errno %d", errno);
            goto CLEAN_UP;
        }
        // ESP_LOGI(TAG, "Successfully connected");

        handler(sock);

        CLEAN_UP:
        if (sock != -1) {
            // ESP_LOGI(TAG, "Shutting down socket and restarting...");
            shutdown(sock, 0);
            close(sock);
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    vTaskDelete(NULL);
}
