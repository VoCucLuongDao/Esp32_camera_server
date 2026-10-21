#include "esp_network.h"
#include "esp_video_capture.h"


static const char *TAG = "NETWORK";

extern uint8_t *shared_frame_buf;
extern size_t shared_frame_len;
extern SemaphoreHandle_t s_frame_mutex;
extern uint8_t fps;

static EventGroupHandle_t s_wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

#define ALLOWED_MAC_STR     "4c:03:4f:db:2f:27"
extern volatile bool g_flash_write_requested;
extern uint16_t g_flash_write_buffer[12];

static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) 
{
    static uint8_t s_retry_num = 0;
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < 5) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGE(TAG, "Connection failed. Retrying (%d/%d)...", s_retry_num, 5);
            vTaskDelay(pdMS_TO_TICKS(1000));
        } else {
            // Chỉ kích hoạt FAIL_BIT khi đã thử hết số lần quy định
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
            if(fps > 1) 
                esp_restart();
        }
    } 
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
    else if (event_id == WIFI_EVENT_AP_STACONNECTED)
    {
        uint8_t target_mac[6];
        int parsed = sscanf(ALLOWED_MAC_STR, "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx", 
                            &target_mac[0], &target_mac[1], &target_mac[2], 
                            &target_mac[3], &target_mac[4], &target_mac[5]);

        wifi_event_ap_staconnected_t* event = (wifi_event_ap_staconnected_t*) event_data;
        ESP_LOGI(TAG, "New Client joined Host! MAC: %02x:%02x:%02x:%02x:%02x:%02x",
                 event->mac[0], event->mac[1], event->mac[2], 
                 event->mac[3], event->mac[4], event->mac[5]);
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
    wifi_init_softap();
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

void wifi_init_sta(void) 
{
    if(s_wifi_event_group == NULL)
        s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID_ACESS,
            .password = WIFI_PASS_ACESS,
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    // Tắt power saving để tăng độ ổn định kết nối
    esp_wifi_set_ps(WIFI_PS_NONE);

    esp_netif_ip_info_t ip_info;
    bool is_assigned_ip = false;

    while (!is_assigned_ip) {
        ESP_LOGI(TAG, "Awaiting valid IP assignment from Router DHCP...");

        /* Chờ tín hiệu kết nối được kích hoạt từ event handler */
        EventBits_t bits = xEventGroupWaitBits(
                s_wifi_event_group,
                WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                pdTRUE,    // Clear bits on exit
                pdFALSE,   // Wait for any bit
                portMAX_DELAY);

        if (bits & WIFI_CONNECTED_BIT) {
            // Lấy thông tin IP thực tế từ giao diện mạng
            if (esp_netif_get_ip_info(sta_netif, &ip_info) == ESP_OK) {
                
                // KIỂM TRA: Nếu IP nhận được vẫn là 0.0.0.0
                if (ip_info.ip.addr == 0) {
                    ESP_LOGW(TAG, "Warning: Connected to AP but IP is 0.0.0.0 (DHCP delay).");
                    ESP_LOGE(TAG, "Reconnecting to network in 3 seconds...");
                    
                    // Thực hiện ngắt kết nối và dọn dẹp để chuẩn bị kết nối lại
                    esp_wifi_disconnect();
                    vTaskDelay(pdMS_TO_TICKS(3000));
                    esp_wifi_connect();
                } 
                else {
                    // Nhận được IP hợp lệ hợp pháp (Khác 0.0.0.0) -> Thoát vòng lặp
                    is_assigned_ip = true; 
                }
            }
        } 
        else if (bits & WIFI_FAIL_BIT) {
            ESP_LOGE(TAG, "Failed to connect to AP hardware layer. Retrying in 3 seconds...");
            vTaskDelay(pdMS_TO_TICKS(3000));
            esp_wifi_connect();
        }
    }

    // --- CHỈ IN BẢNG NÀY KHI ĐÃ CÓ IP HỢP LỆ (KHÁC 0.0.0.0) ---
    ESP_LOGI(TAG, "==============================================");
    ESP_LOGI(TAG, " Wi-Fi Login and Verification Successful!");
    ESP_LOGI(TAG, "==============================================");
    ESP_LOGI(TAG, "ESP32 IPv4 Address : " IPSTR, IP2STR(&ip_info.ip));
    ESP_LOGI(TAG, "Subnet Mask        : " IPSTR, IP2STR(&ip_info.netmask));
    ESP_LOGI(TAG, "Gateway (Router)   : " IPSTR, IP2STR(&ip_info.gw));
    ESP_LOGI(TAG, "==============================================");
}

// Function to discover server IP via UDP request-response
static bool discover_server_ip(ip4_addr_t *target_ip) {
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0) {
        ESP_LOGE(TAG, "Failed to create UDP socket");
        return false;
    }

    // Enable broadcast to send discovery request
    int broadcast = 1;
    setsockopt(sock, SOL_SOCKET, SO_BROADCAST, &broadcast, sizeof(broadcast));

    ESP_LOGI(TAG, "Sending discovery request on UDP port %d...", UDP_BROADCAST_PORT);

    // Set timeout for receiving
    struct timeval timeout;
    timeout.tv_sec = 10;  // 10 seconds timeout
    timeout.tv_usec = 0;
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

    // Prepare discovery request
    const char *discovery_msg = "DISCOVER_SERVER";

    // Send broadcast discovery request
    struct sockaddr_in broadcast_addr;
    broadcast_addr.sin_family = AF_INET;
    broadcast_addr.sin_port = htons(UDP_BROADCAST_PORT);
    broadcast_addr.sin_addr.s_addr = INADDR_BROADCAST;

    int sent = sendto(sock, discovery_msg, strlen(discovery_msg), 0,
                     (struct sockaddr *)&broadcast_addr, sizeof(broadcast_addr));

    if (sent < 0) {
        ESP_LOGE(TAG, "Failed to send discovery request");
        close(sock);
        return false;
    }

    ESP_LOGI(TAG, "Discovery request sent. Waiting for response...");

    // Wait for response
    char rx_buffer[128];
    struct sockaddr_in from_addr;
    socklen_t from_len = sizeof(from_addr);

    int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0,
                      (struct sockaddr *)&from_addr, &from_len);

    if (len < 0) {
        ESP_LOGE(TAG, "UDP receive timeout or error");
        close(sock);
        return false;
    }

    rx_buffer[len] = '\0';
    ESP_LOGI(TAG, "Received UDP response from %s: %s",
             ip4addr_ntoa((const ip4_addr_t *)&from_addr.sin_addr), rx_buffer);

    // Parse the message format: "KEY-IP x.x.x.x"
    if (strncmp(rx_buffer, "KEY-IP ", 7) == 0) {
        char *ip_str = rx_buffer + 7;
        if (ip4addr_aton(ip_str, target_ip) != 0) {
            ESP_LOGI(TAG, "Server IP discovered: " IPSTR, IP2STR(target_ip));
            close(sock);
            return true;
        } else {
            ESP_LOGE(TAG, "Failed to parse IP from response message");
        }
    } else {
        ESP_LOGE(TAG, "Invalid response format");
    }

    close(sock);
    return false;
}

void video_stream_task(void *pvParameters)
{
    wifi_init_sta();

    struct sockaddr_in dest_addr;
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(SERVER_PORT_ACCESS);

    ip4_addr_t target_ip;

    fps = 2;
    // Discovery and connection loop
    while (1)
    {
        // Discover server IP via UDP broadcast
        ESP_LOGI(TAG, "Starting server IP discovery...");
        if (!discover_server_ip(&target_ip)) {
            ESP_LOGE(TAG, "Failed to discover server IP. Retrying in 3 seconds...");
            vTaskDelay(pdMS_TO_TICKS(3000));
            continue;
        }

        esp_wifi_set_max_tx_power(80); // Set max TX power to 80 (0-82 range for ESP32)
        dest_addr.sin_addr.s_addr = target_ip.addr;

        // Set Send and Receive timeouts to 3 seconds to prevent permanent hanging
        struct timeval timeout;
        timeout.tv_sec = 3;
        timeout.tv_usec = 0;
        while (1) 
        {
            int sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
            if (sock < 0) {
                ESP_LOGE(TAG, "Failed to create socket. Retrying in 3 seconds...");
                vTaskDelay(pdMS_TO_TICKS(3000));
                continue;
            }

            setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
            setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));

            if (connect(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr)) != 0) {
                ESP_LOGE(TAG, "Connection failed! Retrying in 3 seconds...");
                close(sock);
                vTaskDelay(pdMS_TO_TICKS(3000));
                continue;
            }

            ESP_LOGI(TAG, "TCP CONNECTED. Sending credentials for authentication...");

            char rx_buffer[128];
            memset(rx_buffer, 0, sizeof(rx_buffer));
            int len = recv(sock, rx_buffer, sizeof(rx_buffer) - 1, 0);

            if (len <= 0) {
                ESP_LOGE(TAG, "Authentication TIMEOUT or server not responding.");
                close(sock);
                vTaskDelay(pdMS_TO_TICKS(3000));
                continue;
            }

            if (strstr(rx_buffer, AUTH_PASSWORD) == NULL)
            {
                ESP_LOGE(TAG, "Authentication DENIED by host server. Server response: %s", rx_buffer);
                close(sock);
                vTaskDelay(pdMS_TO_TICKS(3000));
                continue;
            }
            else
            {
                // Mật khẩu đúng -> Phản hồi tín hiệu xanh cho Python chuyển trạng thái
                send(sock, "AUTH_OK\n", 8, 0);
                ESP_LOGI(TAG, "Authentication SUCCESSFUL!");

                // 2. 👉 PARSER BÓC TÁCH VÀ ĐỐI CHIẾU CHUỖI LỆNH RAW NỐI PHÍA SAU
                char *raw_cmd_ptr = strstr(rx_buffer, "SET_RAW:");
                if (raw_cmd_ptr != NULL && !g_flash_write_requested)
                {
                    uint16_t incoming_array[12];
                    int parsed = sscanf(raw_cmd_ptr + 8, "%hu,%hu,%hu,%hu,%hu,%hu,%hu,%hu,%hu,%hu,%hu,%hu",
                                        &incoming_array[0], &incoming_array[1], &incoming_array[2], 
                                        &incoming_array[3], &incoming_array[4], &incoming_array[5], 
                                        &incoming_array[6], &incoming_array[7], &incoming_array[8], 
                                        &incoming_array[9], &incoming_array[10], &incoming_array[11]);
                    
                    if (parsed == 12)
                    {
                        // Đối chiếu kiểm tra dải miền số an toàn [0 - 2000]
                        bool valid = true;
                        for (int i = 0; i < 12; i++)
                        {
                            if (incoming_array[i] > 2000)
                            {
                                valid = false;
                                break;
                            }
                        }

                        if(incoming_array[0] == 999)
                        {
                            esp_restart();
                        }

                        if (valid)
                        {
                            // Thỏa mãn toàn bộ điều kiện -> Gán vào bộ đệm và dựng cờ báo main loop xử lý
                            memcpy(g_flash_write_buffer, incoming_array, sizeof(g_flash_write_buffer));
                            g_flash_write_requested = true;
                            ESP_LOGI(TAG, "✓ [VALID CONFIG] Appended RAW params match format perfectly. Forwarded to main loop.");
                        }
                    }
                }
            }

            ESP_LOGI(TAG, "Authentication SUCCESSFUL! Initiating video stream from shared_frame_buf.");

            // --- STEP 2: VIDEO STREAM LOOP FROM SHARED MEMORY BUFFER ---
            TickType_t start_time = 0;
            int cnt = 0;
            int error_cnt = 0;  // Track consecutive errors
            int err = 0;
            fps = 20;  // Set to higher FPS after successful connection

            while (1)
            {
                start_time = xTaskGetTickCount();

                // Guard against unallocated or empty shared memory
                if (shared_frame_buf == NULL || shared_frame_len == 0) {
                    ESP_LOGI(TAG, "Shared frame buffer is empty or unallocated. Waiting...");
                    vTaskDelay(pdMS_TO_TICKS(1000));
                    continue;
                }

                // Safely lock the shared buffer using your external mutex
                if (shared_frame_buf != NULL &&
                    shared_frame_len > 0 &&
                    xSemaphoreTake(s_frame_mutex, pdMS_TO_TICKS(1)) == pdTRUE)
                {
                    uint32_t image_len = (uint32_t)shared_frame_len;

                    // 2.1 Send frame size header (4 bytes payload declaration)
                    err = send(sock, &image_len, sizeof(image_len), 0);
                    if (err >= 0)
                    {
                        // 2.2 Send binary payload data
                        size_t bytes_sent = 0;
                        while (bytes_sent < image_len)
                        {
                            int sent = send(sock, shared_frame_buf + bytes_sent, image_len - bytes_sent, 0);
                            if (sent <= 0)
                            {
                                ESP_LOGE(TAG, "Socket transmission failed during packet payload delivery.");
                                err = -1;
                                error_cnt++;
                                if (error_cnt > 10) {
                                    ESP_LOGE(TAG, "Too many consecutive errors (%d). Breaking connection.", error_cnt);
                                }
                                break;
                            }
                            bytes_sent += sent;
                        }
                    }
                    else
                    {
                        ESP_LOGE(TAG, "Socket transmission failed during packet size header delivery.");
                        error_cnt++;
                        if (error_cnt > 10) {
                            ESP_LOGE(TAG, "Too many consecutive errors (%d). Breaking connection.", error_cnt);
                            break;
                        }
                        break;
                    }

                    // Reset error counter on successful transmission
                    error_cnt = 0;

                    // Immediately release the mutex to let the frame producer write new frames
                    xSemaphoreGive(s_frame_mutex);
                    cnt++;
                    if(cnt%fps == 0)
                        ESP_LOGI(TAG, "Sent a frame in %d ms, len %d, speed %dfps", pdTICKS_TO_MS(xTaskGetTickCount() - start_time), image_len, fps);  
                    vTaskDelay(pdMS_TO_TICKS(1));
                    xTaskDelayUntil(&start_time, pdMS_TO_TICKS(1000/fps));
                }
                else
                {
                    vTaskDelay(pdMS_TO_TICKS(10));
                }
            }
            ESP_LOGE(TAG, "Connection lost! Closing socket descriptor and restarting connection sequence in 3 seconds...");
            xSemaphoreGive(s_frame_mutex);
            close(sock);
            vTaskDelay(pdMS_TO_TICKS(3000));
        }
    }
}