#ifndef __ESP_NETWORK_H_
#define __ESP_NETWORK_H_

#include <string.h>
#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "lwip/sockets.h"
#include "esp_netif_types.h"
#include "esp_http_server.h"

// ----- CẤU HÌNH THÔNG SỐ WI-FI & PORT -----
#define WIFI_SSID             "V29"
#define WIFI_PASSWORD         "12346789"
#define SERVER_PORT           8080
#define WIFI_CONNECTED_BIT    BIT0

#define ESP_WIFI_CHANNEL      1                    // Kênh Wi-Fi
#define MAX_STA_CONN          4                    // Số lượng Client tối đa có thể kết nối vào
#define ESP_WIFI_AP_SSID     "NAROA"
#define ESP_WIFI_AP_PASS     "vcld1876317"  

#define PART_BOUNDARY       "123456789000000000000987654321"

#define WIFI_SSID_ACESS     "LT29"
#define WIFI_PASS_ACESS     "vcld1876317"
#define SERVER_PORT_ACCESS   1357
#define AUTH_PASSWORD       "pass187631\n" 


void wifi_init_softap(void);
void wifi_init_sta(void);
void start_host_web_server(void *pvParameters);
void video_stream_task(void *pvParameters);

#endif