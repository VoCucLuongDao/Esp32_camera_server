#include "esp_network.h"
#include "esp_video_capture.h"


static const char *TAG = "NETWORK";

extern uint8_t *shared_frame_buf;
extern size_t shared_frame_len;
extern SemaphoreHandle_t s_frame_mutex;
extern uint8_t fps;

// Trình quản lý sự kiện Wi-Fi Host (Soft-AP)
static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    uint8_t target_mac[6];
    int parsed = sscanf(ALLOWED_MAC_STR, "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx", 
                        &target_mac[0], &target_mac[1], &target_mac[2], 
                        &target_mac[3], &target_mac[4], &target_mac[5]);
        
    if (event_id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t* event = (wifi_event_ap_staconnected_t*) event_data;
        ESP_LOGI(TAG, "New Client joined Host! MAC: %02x:%02x:%02x:%02x:%02x:%02x",
                 event->mac[0], event->mac[1], event->mac[2], 
                 event->mac[3], event->mac[4], event->mac[5]);
        fps = 25;
        // 2. Tiến hành so sánh nếu ép kiểu chuỗi thành công
        if (parsed == 6)
        {
            if (memcmp(event->mac, target_mac, 6) == 0) {
                // Địa chỉ MAC trùng khớp -> Chấp nhận kết nối
                ESP_LOGI(TAG, "MAC Matched! Access Granted for client.");
            } else {
                // Địa chỉ MAC không trùng khớp -> Từ chối và ngắt kết nối
                ESP_LOGW(TAG, "MAC Mismatch! Access Denied. Disconnecting client...");
                
                // Gọi hàm deauth để ép buộc client này ngắt kết nối ngay lập tức
                esp_wifi_deauth_sta(event->aid);
            }
        } else {
            ESP_LOGE(TAG, "Failed to parse ALLOWED_MAC_STR. Blocked client by default for safety.");
            esp_wifi_deauth_sta(event->aid);
        }

    }
    else if (event_id == WIFI_EVENT_AP_STADISCONNECTED)
    {
        wifi_event_ap_stadisconnected_t* event = (wifi_event_ap_stadisconnected_t*) event_data;
        ESP_LOGE(TAG, "A Client left Host! MAC: %02x:%02x:%02x:%02x:%02x:%02x",
                 event->mac[0], event->mac[1], event->mac[2], 
                 event->mac[3], event->mac[4], event->mac[5]);
        fps = 1;
    }
}


// Khởi tạo ESP32 làm Wi-Fi Host (Access Point)
void wifi_init_softap(void) 
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    
    // Tạo giao tiếp mạng Soft-AP. 
    // Mặc định IP của ESP32 Host sẽ là: 192.168.4.1
    esp_netif_t *ap_netif = esp_netif_create_default_wifi_ap();

    // 2. DỪNG dịch vụ DHCP Server mặc định để cấu hình lại IP tĩnh cho Host
    ESP_ERROR_CHECK(esp_netif_dhcps_stop(ap_netif));

    // 3. Khai báo cấu hình IP tĩnh cố định theo nhu cầu của bạn
    esp_netif_ip_info_t ip_info;
    memset(&ip_info, 0, sizeof(esp_netif_ip_info_t));
    
    // Đặt IP tĩnh cho Host (Ví dụ đổi từ 4.1 sang cố định 192.168.9.1)
    esp_netif_set_ip4_addr(&ip_info.ip, 192, 168, 9, 1);       // Đặt IP tĩnh Server là 192.168.9.1
    esp_netif_set_ip4_addr(&ip_info.gw, 192, 168, 9, 1);       // Gateway trùng với IP Host
    esp_netif_set_ip4_addr(&ip_info.netmask, 255, 255, 255, 0); // Subnet mask tiêu chuẩn

    // Áp dụng cấu hình IP tĩnh vào hệ thống
    ESP_ERROR_CHECK(esp_netif_set_ip_info(ap_netif, &ip_info));

    // 4. KHỞI ĐỘNG LẠI DHCP Server để nó tự động cấp IP cho các thiết bị bắt vào (Dải 192.168.5.x)
    ESP_ERROR_CHECK(esp_netif_dhcps_start(ap_netif));

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));

    wifi_config_t wifi_config = {
        .ap = {
            .ssid = ESP_WIFI_AP_SSID,
            .ssid_len = strlen(ESP_WIFI_AP_SSID),
            .channel = ESP_WIFI_CHANNEL,
            .password = ESP_WIFI_AP_PASS,
            .max_connection = MAX_STA_CONN,
            .authmode = WIFI_AUTH_WPA2_PSK,
            .pmf_cfg = {
                .required = false,
            },
        },
    };

    if (strlen(ESP_WIFI_AP_PASS) == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    // In thông tin IP tĩnh của Host ra UART
    esp_netif_get_ip_info(ap_netif, &ip_info);
    ESP_LOGI(TAG, "Wi-Fi suscess! SSID: %s", ESP_WIFI_AP_SSID);
    ESP_LOGI(TAG, "IP Server Host: " IPSTR, IP2STR(&ip_info.ip));
    //uart_send_data(msg, len);
}

// 1. Handler xử lý Stream Video (MJPEG)
static esp_err_t stream_handler(httpd_req_t *req) {
    esp_err_t res = ESP_OK;
    char part_buf[64];
    ESP_LOGI(TAG, "HTTP Client started");
    res = httpd_resp_set_type(req, "multipart/x-mixed-replace;boundary=" PART_BOUNDARY);
    if(res != ESP_OK)
        return res;
    TickType_t  start_time = 0;
    int cnt = 0;
    while (true)
    {
        start_time = xTaskGetTickCount();
        if(shared_frame_buf != NULL && shared_frame_len > 0)
        {
            size_t hlen = snprintf(part_buf, sizeof(part_buf), "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n", shared_frame_len);
            res = httpd_resp_send_chunk(req, part_buf, hlen);
            if (res == ESP_OK &&
                xSemaphoreTake(s_frame_mutex, pdMS_TO_TICKS(1)) == pdTRUE) 
            {
                res = httpd_resp_send_chunk(req, (const char *)shared_frame_buf, shared_frame_len);
                xSemaphoreGive(s_frame_mutex);
            }
            if (res == ESP_OK)
                res = httpd_resp_send_chunk(req, "\r\n--" PART_BOUNDARY "\r\n", strlen("\r\n--" PART_BOUNDARY "\r\n"));
            if (res != ESP_OK) 
                break;
        }
        
        cnt++;
        if(cnt%25 == 0)
            ESP_LOGI(TAG, "Send frame in %d ms", pdTICKS_TO_MS(xTaskGetTickCount() - start_time));
        vTaskDelayUntil (&start_time, pdMS_TO_TICKS(40)); 
    }
    return res;
}

void start_host_web_server(void *pvParameters) 
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;
    static httpd_handle_t server = NULL;
    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_uri_t stream_uri = { 
            .uri = "/stream292", 
            .method = HTTP_GET, 
            .handler = stream_handler, 
            .user_ctx = NULL 
        };
        httpd_register_uri_handler(server, &stream_uri);
        ESP_LOGI(TAG, "HTTP Server started on port 80. Endpoint: /stream292");
    }
    
    while (1) 
    {
        vTaskDelay(pdMS_TO_TICKS(100)); 
    }
}
