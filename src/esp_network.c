#include "esp_network.h"
#include "esp_video_capture.h"
#include "esp_heap_caps.h"
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <errno.h>


static const char *TAG = "NETWORK";

extern uint8_t *shared_frame_buf;
extern size_t shared_frame_len;
extern SemaphoreHandle_t s_frame_mutex;
extern uint8_t fps;

static EventGroupHandle_t s_wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

/* ---- Giám sát rớt WiFi ----
 * Chỉ reboot khi TRƯỚC ĐÓ đã kết nối thành công ít nhất một lần; nếu chưa từng
 * kết nối (SSID sai, router chưa bật) thì chờ mãi, tránh vòng lặp khởi động lại.
 */
#define WIFI_DROP_REBOOT_MS   5000   /* mất WiFi liên tục > 5 s -> reboot */
#define WIFI_GUARD_TICK_MS    500    /* chu kỳ kiểm tra */

static volatile bool s_wifi_ever_connected = false;  /* đã từng có IP */
static volatile bool s_wifi_link_up        = false;  /* đang có kết nối */
static volatile TickType_t s_wifi_drop_tick = 0;     /* lúc bắt đầu mất kết nối */

#define ALLOWED_MAC_STR     "4c:03:4f:db:2f:27"

/* Mã lý do ngắt kết nối của ESP-IDF, in ra log cho dễ chẩn đoán */
static const char *wifi_disc_reason_str(uint8_t reason)
{
    switch (reason) {
    case 2:   return "AUTH_EXPIRE - sai mật khẩu / hết thời gian xác thực";
    case 3:   return "AUTH_LEAVE";
    case 8:   return "ASSOC_LEAVE - AP chủ động ngắt";
    case 15:  return "4WAY_HANDSHAKE_TIMEOUT - sai mật khẩu WPA2";
    case 17:  return "IE_INVALID";
    case 201: return "NO_AP_FOUND - không tìm thấy SSID";
    case 202: return "AUTH_FAIL - sai mật khẩu";
    case 203: return "ASSOC_FAIL";
    case 204: return "HANDSHAKE_TIMEOUT";
    case 205: return "CONNECTION_FAIL - AP từ chối";
    default:  return "không rõ";
    }
}

static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) 
{
    static uint32_t s_retry_num = 0;
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        const wifi_event_sta_disconnected_t *ev =
            (const wifi_event_sta_disconnected_t *)event_data;
        uint8_t reason = ev ? ev->reason : 0;
        s_retry_num++;

        /* Đánh dấu mốc mất kết nối (chỉ lấy lần đầu của chuỗi rớt liên tiếp)
         * để task giám sát reboot sau WIFI_DROP_REBOOT_MS. */
        if (s_wifi_link_up) {
            s_wifi_link_up = false;
            s_wifi_drop_tick = xTaskGetTickCount();
        }

        // Log 3 lần đầu và sau đó mỗi 10 lần để không ngập UART
        if (s_retry_num <= 3 || (s_retry_num % 10) == 0) {
            ESP_LOGW(TAG, "WiFi ngắt kết nối (lần %u): reason=%u -> %s",
                     (unsigned)s_retry_num, (unsigned)reason,
                     wifi_disc_reason_str(reason));
        }

        // Không reboot ở đây: task giám sát chỉ reboot khi đã từng kết nối và
        // mất liên tục > 5 s; còn SSID sai/hotspot tắt thì vẫn thử lại mãi.
        vTaskDelay(pdMS_TO_TICKS(500));
        esp_wifi_connect();
    } 
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        s_retry_num = 0;
        s_wifi_ever_connected = true;   // đã từng kết nối -> bật cơ chế tự reboot
        s_wifi_link_up = true;
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
        if(cnt%200 == 0)
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

/* Task giám sát WiFi: đã từng kết nối thành công mà mất liên tục > 5 giây thì
 * khởi động lại ESP32 để tự phục hồi. Nếu CHƯA từng kết nối (SSID sai, router
 * chưa bật) thì không reboot — cứ chờ và thử lại cho đến khi có WiFi. */
static void wifi_guard_task(void *arg)
{
    (void)arg;

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(WIFI_GUARD_TICK_MS));

        if (!s_wifi_ever_connected || s_wifi_link_up) {
            continue;               // chưa từng kết nối, hoặc đang kết nối tốt
        }

        TickType_t down = (TickType_t)(xTaskGetTickCount() - s_wifi_drop_tick);
        if (down >= pdMS_TO_TICKS(WIFI_DROP_REBOOT_MS)) {
            ESP_LOGE(TAG, "Mat WiFi > %u s sau khi da ket noi -> khoi dong lai ESP32",
                     (unsigned)(WIFI_DROP_REBOOT_MS / 1000));
            vTaskDelay(pdMS_TO_TICKS(200));   // kịp đẩy log ra UART
            esp_restart();
        }
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
    esp_wifi_set_ps(WIFI_PS_MAX_MODEM);

    // Giám sát rớt WiFi: đã từng kết nối mà mất > 5 s -> reboot (xem wifi_guard_task)
    if (xTaskCreate(wifi_guard_task, "wifi_guard", 3072, NULL, 3, NULL) != pdPASS) {
        ESP_LOGE(TAG, "Khong tao duoc task wifi_guard");
    }

    esp_netif_ip_info_t ip_info;
    bool is_assigned_ip = false;
    uint32_t wait_rounds = 0;

    while (!is_assigned_ip) {
        /* Chờ tín hiệu từ event handler. Có timeout thay vì portMAX_DELAY để
         * task mạng không "chết lặng" khi router tắt: mỗi 10 giây in 1 dòng
         * cảnh báo, và vẫn tiếp tục chờ vô hạn (event handler tự gọi
         * esp_wifi_connect() lại). */
        EventBits_t bits = xEventGroupWaitBits(
                s_wifi_event_group,
                WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                pdTRUE,    // Clear bits on exit
                pdFALSE,   // Wait for any bit
                pdMS_TO_TICKS(10000));

        if ((bits & (WIFI_CONNECTED_BIT | WIFI_FAIL_BIT)) == 0) {
            if ((++wait_rounds % 3) == 1) {
                ESP_LOGW(TAG, "Chua co IP sau %u s - kiem tra SSID/mat khau/router (van dang thu lai)",
                         (unsigned)(wait_rounds * 10));
            }
            continue;
        }
        wait_rounds = 0;

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

/* ============================================================================
 * KÊNH ĐIỀU KHIỂN (TCP 1358): Python gửi lệnh dạng text, ESP32 trả lời OK/ERR
 * Tách riêng khỏi cổng 1357 vì cổng đó đang truyền frame nhị phân
 * [uint32 len][JPEG], chèn text vào sẽ làm hỏng bộ đếm khung của client.
 * ========================================================================== */
#define CTRL_RX_MAX 512      // đủ cho 1 dòng "CFG" chứa toàn bộ 25 tham số

static char s_ctrl_rx[CTRL_RX_MAX];
static int  s_ctrl_rx_len = 0;

/* Buffer gửi riêng của task mạng: copy frame trong lúc giữ mutex (~1 ms) rồi
 * send() ra ngoài mutex. Trước đây task mạng giữ mutex suốt quá trình send()
 * nên task capture bị chặn -> FPS sụt xuống đúng bằng tốc độ mạng. */
static uint8_t *s_tx_buf   = NULL;
static size_t   s_tx_cap   = 0;
static uint32_t s_last_seq = 0;
static uint32_t s_last_send_ms = 0;

typedef struct {
    const char *key;
    int min;
    int max;
    size_t offset;
} cam_cfg_field_t;

/* Bảng key -> trường trong cam_config_t, kèm miền giá trị hợp lệ */
static const cam_cfg_field_t s_cfg_fields[] = {
    {"brightness", -2,    2, offsetof(cam_config_t, brightness)},
    {"contrast",   -2,    2, offsetof(cam_config_t, contrast)},
    {"saturation", -2,    2, offsetof(cam_config_t, saturation)},
    {"sharpness",  -2,    2, offsetof(cam_config_t, sharpness)},
    {"ae_level",   -2,    2, offsetof(cam_config_t, ae_level)},
    {"denoise",     0,    8, offsetof(cam_config_t, denoise)},
    {"quality",     0,   63, offsetof(cam_config_t, quality)},
    {"framesize",   0,   16, offsetof(cam_config_t, framesize)},
    {"gainceiling", 0,    6, offsetof(cam_config_t, gainceiling)},
    {"wb_mode",     0,    4, offsetof(cam_config_t, wb_mode)},
    {"effect",      0,    6, offsetof(cam_config_t, effect)},
    {"hmirror",     0,    1, offsetof(cam_config_t, hmirror)},
    {"vflip",       0,    1, offsetof(cam_config_t, vflip)},
    {"aec",         0,    1, offsetof(cam_config_t, aec)},
    {"agc",         0,    1, offsetof(cam_config_t, agc)},
    {"aec_value",   0, 1200, offsetof(cam_config_t, aec_value)},
    {"agc_gain",    0,   30, offsetof(cam_config_t, agc_gain)},
    {"bpc",         0,    1, offsetof(cam_config_t, bpc)},
    {"wpc",         0,    1, offsetof(cam_config_t, wpc)},
    {"lenc",        0,    1, offsetof(cam_config_t, lenc)},
    {"raw_gma",     0,    1, offsetof(cam_config_t, raw_gma)},
    {"fps",         1,   30, offsetof(cam_config_t, fps)},
    /* Kích thước khung hình tuỳ chỉnh. width/height được làm tròn xuống số
     * chẵn và kẹp theo vùng cảm biến OV3660 (2048x1536) khi áp dụng. */
    {"custom",      0,    1, offsetof(cam_config_t, custom)},
    {"width",      96, 2048, offsetof(cam_config_t, width)},
    {"height",     96, 1536, offsetof(cam_config_t, height)},
};

#define CFG_FIELD_COUNT (sizeof(s_cfg_fields) / sizeof(s_cfg_fields[0]))

/* Cú pháp: "CFG key=value key=value ..."  (chỉ cần gửi các key muốn đổi) */
static bool ctrl_apply_cfg(char *line, bool *need_restart)
{
    cam_config_t cfg;
    camera_config_get(&cfg);

    char *saveptr = NULL;
    char *tok = strtok_r(line, " \t\r\n", &saveptr);
    if (tok == NULL || strcasecmp(tok, "CFG") != 0)
        return false;

    int fields = 0;
    while ((tok = strtok_r(NULL, " \t\r\n", &saveptr)) != NULL) {
        char *eq = strchr(tok, '=');
        if (eq == NULL)
            return false;
        *eq = '\0';

        const char *key = tok;
        const char *val = eq + 1;

        if (strcasecmp(key, "restart") == 0) {
            if (atoi(val) != 0) *need_restart = true;
            continue;
        }

        bool known = false;
        for (size_t i = 0; i < CFG_FIELD_COUNT; i++) {
            if (strcasecmp(key, s_cfg_fields[i].key) != 0)
                continue;

            int v = atoi(val);
            if (v < s_cfg_fields[i].min || v > s_cfg_fields[i].max) {
                ESP_LOGW(TAG, "CFG value out of range: %s=%d", key, v);
                return false;
            }
            *(int *)((uint8_t *)&cfg + s_cfg_fields[i].offset) = v;
            known = true;
            fields++;
            break;
        }
        if (!known) {
            ESP_LOGW(TAG, "CFG unknown key: %s", key);
            return false;
        }
    }

    if (fields > 0)
        camera_config_set(&cfg);   // task capture sẽ áp dụng vào sensor

    return (fields > 0) || *need_restart;
}

/* Gửi đủ len byte. send() trên socket block có thể trả về THIẾU byte khi WiFi
 * chậm; trước đây header 4 byte có thể chỉ đi 1-2 byte làm lệch hoàn toàn bộ
 * đếm frame phía client. Trả 0 nếu đủ, -1 nếu lỗi/timeout. */
static int send_all(int sock, const void *buf, size_t len)
{
    const uint8_t *p = (const uint8_t *)buf;
    size_t done = 0;
    while (done < len) {
        ssize_t n = send(sock, p + done, len - done, 0);
        if (n <= 0)
            return -1;
        done += (size_t)n;
    }
    return 0;
}

static void ctrl_send_str(int sock, const char *msg)
{
    if (sock >= 0) send_all(sock, msg, strlen(msg));
}

/* Trả về số liệu chẩn đoán của cả 2 phía: capture (fps, exposure, gain) và mạng
 * (thời gian send, RSSI). Dùng để phân biệt "FPS thấp do thiếu sáng" với
 * "FPS thấp do WiFi yếu". */
static void ctrl_send_stats(int sock)
{
    cam_stats_t  st;
    cam_config_t cfg;
    camera_stats_get(&st);
    camera_config_get(&cfg);

    int rssi = 0;
    wifi_ap_record_t ap;
    if (esp_wifi_sta_get_ap_info(&ap) == ESP_OK)
        rssi = ap.rssi;

    char buf[300];
    snprintf(buf, sizeof(buf),
             "STAT fps=%u.%u frames=%u bad=%u len=%u exp=%d gain=%d ae=%d send=%ums "
             "fs=%d q=%d gc=%d res=%dx%d custom=%d cam=%d rssi=%d heap=%u min=%u psram=%u stream=%d\n",
             (unsigned)(st.fps_x10 / 10), (unsigned)(st.fps_x10 % 10),
             (unsigned)st.frames, (unsigned)st.invalid, (unsigned)st.last_len,
             st.exposure, st.gain, st.ae_manual,
             (unsigned)s_last_send_ms, cfg.framesize, cfg.quality, cfg.gainceiling,
             st.out_w, st.out_h, cfg.custom, camera_is_ready() ? 1 : 0,
             rssi, (unsigned)st.heap_free, (unsigned)st.heap_min,
             (unsigned)st.psram_free, camera_stream_active() ? 1 : 0);
    ctrl_send_str(sock, buf);
}

/* Xử lý 1 dòng lệnh nhận được từ client */
static void ctrl_handle_line(int sock, char *line)
{
    if (strncmp(line, "CFG", 3) == 0 && (line[3] == ' ' || line[3] == '\0')) {
        bool restart = false;
        if (ctrl_apply_cfg(line, &restart)) {
            ctrl_send_str(sock, "OK\n");
            ESP_LOGI(TAG, "CFG accepted and queued for the capture task");
            if (restart) {
                ESP_LOGW(TAG, "Restart requested by client");
                vTaskDelay(pdMS_TO_TICKS(200));
                esp_restart();
            }
        } else {
            ctrl_send_str(sock, "ERR invalid\n");
        }
    } else if (strncmp(line, "PING", 4) == 0) {
        ctrl_send_str(sock, "PONG\n");
    } else if (strncmp(line, "STAT", 4) == 0) {
        ctrl_send_stats(sock);
    } else if (strncmp(line, "RESTART", 7) == 0) {
        ctrl_send_str(sock, "OK restart\n");
        ESP_LOGW(TAG, "Restart requested by client");
        vTaskDelay(pdMS_TO_TICKS(200));
        esp_restart();
    } else if (line[0] != '\0') {
        ctrl_send_str(sock, "ERR unknown\n");
    }
}

/* Tách 1 dòng '\n' khỏi buffer nhận. Trả về 1 nếu có dòng hoàn chỉnh. */
static int ctrl_extract_line(char *out, size_t out_max)
{
    char *nl = memchr(s_ctrl_rx, '\n', s_ctrl_rx_len);
    if (nl == NULL)
        return 0;

    size_t len = (size_t)(nl - s_ctrl_rx);
    size_t cp = (len < out_max - 1) ? len : out_max - 1;
    memcpy(out, s_ctrl_rx, cp);
    out[cp] = '\0';

    memmove(s_ctrl_rx, nl + 1, (size_t)s_ctrl_rx_len - len - 1);
    s_ctrl_rx_len -= (int)(len + 1);

    if (cp > 0 && out[cp - 1] == '\r')
        out[cp - 1] = '\0';

    return 1;
}

/* Đọc & xử lý toàn bộ dòng đang có sẵn (không block).
 * Trả về -1 khi kết nối đã đóng/lỗi.
 *
 * Giới hạn số byte xử lý mỗi lần gọi: nếu client (hoặc một bản UI thứ hai bị
 * kẹt) bắn dữ liệu liên tục thì vòng while(1) sẽ không bao giờ trả về, chiếm
 * trọn core 1 và có thể làm task IDLE đói -> TWDT reset. Xử lý tối đa 2 KB
 * rồi nhường CPU, phần còn lại để vòng stream sau. */
static int ctrl_poll(int sock)
{
    static char line[CTRL_RX_MAX];   // static để không tốn stack của task mạng
    int budget = 2048;

    while (budget > 0 && ctrl_extract_line(line, sizeof(line)))
        ctrl_handle_line(sock, line);

    while (budget > 0) {
        if (s_ctrl_rx_len >= (int)sizeof(s_ctrl_rx) - 1)
            s_ctrl_rx_len = 0;   // dòng quá dài -> bỏ

        int r = recv(sock, s_ctrl_rx + s_ctrl_rx_len,
                     sizeof(s_ctrl_rx) - 1 - s_ctrl_rx_len, MSG_DONTWAIT);
        if (r > 0) {
            budget -= r;
            s_ctrl_rx_len += r;
            while (ctrl_extract_line(line, sizeof(line)))
                ctrl_handle_line(sock, line);
            continue;
        }
        if (r == 0)
            return -1;                                  // server đóng kết nối
        if (errno == EAGAIN || errno == EWOULDBLOCK)
            return 0;                                   // chưa có dữ liệu
        return -1;
    }
    return 0;      // hết hạn mức -> để vòng sau xử lý tiếp
}

/* Mở kết nối tới kênh điều khiển trên PC (cùng IP, khác port) */
static int ctrl_socket_open(const struct sockaddr_in *video_addr)
{
    struct sockaddr_in addr = *video_addr;
    addr.sin_port = htons(CONTROL_PORT_ACCESS);

    int sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    if (sock < 0) {
        ESP_LOGE(TAG, "Control: cannot create socket");
        return -1;
    }

    struct timeval tv = { .tv_sec = 3, .tv_usec = 0 };
    setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));

    if (connect(sock, (struct sockaddr *)&addr, sizeof(addr)) != 0) {
        ESP_LOGW(TAG, "Control: connect to port %d failed (UI sẽ không chỉnh được thông số)",
                 CONTROL_PORT_ACCESS);
        close(sock);
        return -1;
    }

    s_ctrl_rx_len = 0;
    ctrl_send_str(sock, "CTRL_HELLO\n");
    ESP_LOGI(TAG, "Control channel connected on port %d", CONTROL_PORT_ACCESS);
    return sock;
}

void video_stream_task(void *pvParameters)
{
    wifi_init_sta();

    struct sockaddr_in dest_addr;
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(SERVER_PORT_ACCESS);

    ip4_addr_t target_ip;

    // KHÔNG gán fps ở đây: fps do cam_config_t quyết định (UI chỉnh được).
    // Trước đây code gán fps = 2 rồi fps = 20 sau khi kết nối, đè mất giá trị
    // người dùng đặt trên bảng điều khiển.

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

            int ctrl_sock = -1;
            TickType_t ctrl_last_try = 0;

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
                send_all(sock, "AUTH_OK\n", 8);
                ESP_LOGI(TAG, "Authentication SUCCESSFUL!");

                // 2. MỞ KÊNH ĐIỀU KHIỂN RIÊNG (TCP 1358) ĐỂ CHỈNH THÔNG SỐ CAMERA
                //    Tách khỏi socket video để không làm hỏng khung [len][JPEG]
                ctrl_sock = ctrl_socket_open(&dest_addr);
                ctrl_last_try = xTaskGetTickCount();
            }

            ESP_LOGI(TAG, "Authentication SUCCESSFUL! Initiating video stream from shared_frame_buf.");

            // Có client -> capture chạy đúng FPS đặt trên UI (mặc định 20)
            camera_set_stream_active(true);

            // --- STEP 2: VIDEO STREAM LOOP FROM SHARED MEMORY BUFFER ---
            int cnt = 0;
            int err = 0;
            uint32_t idle_ms = 5;   // nghỉ khi không có frame mới (tăng dần)

            while (1)
            {
                // 0. Phục vụ kênh điều khiển (không block, 1 lần mỗi vòng)
                if (ctrl_sock >= 0) {
                    if (ctrl_poll(ctrl_sock) < 0) {
                        ESP_LOGW(TAG, "Control channel closed. Sẽ thử kết nối lại...");
                        close(ctrl_sock);
                        ctrl_sock = -1;
                        s_ctrl_rx_len = 0;
                        ctrl_last_try = xTaskGetTickCount();
                    }
                } else if (pdTICKS_TO_MS(xTaskGetTickCount() - ctrl_last_try) >= 5000) {
                    ctrl_last_try = xTaskGetTickCount();
                    ctrl_sock = ctrl_socket_open(&dest_addr);
                }

                // Guard against unallocated or empty shared memory
                if (shared_frame_buf == NULL || shared_frame_len == 0) {
                    vTaskDelay(pdMS_TO_TICKS(idle_ms));
                    if (idle_ms < 50)
                        idle_ms += 5;
                    continue;
                }

                // 1. Copy frame ra buffer riêng trong lúc giữ mutex (vài trăm µs),
                //    chỉ khi capture vừa publish frame mới (s_frame_seq đổi).
                size_t   image_len = 0;
                if (xSemaphoreTake(s_frame_mutex, pdMS_TO_TICKS(5)) == pdTRUE)
                {
                    uint32_t seq = camera_frame_seq();
                    if (seq != s_last_seq && shared_frame_len > 0)
                    {
                        if (shared_frame_len > s_tx_cap)
                        {
                            uint8_t *tmp = (uint8_t *)heap_caps_realloc(
                                s_tx_buf, shared_frame_len, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
                            if (tmp == NULL)
                                tmp = (uint8_t *)heap_caps_realloc(
                                    s_tx_buf, shared_frame_len, MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
                            if (tmp != NULL)
                            {
                                s_tx_buf = tmp;
                                s_tx_cap = shared_frame_len;
                            }
                        }
                        if (s_tx_buf != NULL && s_tx_cap >= shared_frame_len)
                        {
                            memcpy(s_tx_buf, shared_frame_buf, shared_frame_len);
                            image_len = shared_frame_len;
                            s_last_seq = seq;
                        }
                    }
                    xSemaphoreGive(s_frame_mutex);
                }

                // Chưa có frame mới -> nhường CPU, không gửi lặp.
                // Nếu camera đang lỗi/treo, vòng này chạy mãi: tăng dần thời
                // gian nghỉ 5 -> 50 ms để không đốt CPU (vẫn đủ nhanh để trả
                // lời PING/CFG). Có frame mới thì trả về 5 ms.
                if (image_len == 0)
                {
                    vTaskDelay(pdMS_TO_TICKS(idle_ms));
                    if (idle_ms < 50)
                        idle_ms += 5;
                    continue;
                }
                idle_ms = 5;

                // 2. Gửi ra ngoài mutex: task capture không bị chặn bởi send()
                TickType_t send_t0 = xTaskGetTickCount();
                uint32_t   hdr = (uint32_t)image_len;
                size_t     bytes_sent = 0;

                // Gửi đủ header + payload (send() có thể ghi thiếu byte)
                err = send_all(sock, &hdr, sizeof(hdr));
                if (err == 0) {
                    err = send_all(sock, s_tx_buf, image_len);
                    if (err == 0)
                        bytes_sent = image_len;
                }

                s_last_send_ms = pdTICKS_TO_MS(xTaskGetTickCount() - send_t0);

                // Lỗi truyền -> thoát vòng stream để kết nối lại
                if (err < 0 || bytes_sent < image_len)
                {
                    ESP_LOGE(TAG, "Stream send failed (%u/%u bytes). Reconnecting...",
                             (unsigned)bytes_sent, (unsigned)image_len);
                    break;
                }

                cnt++;
                if ((cnt % 200) == 0)
                    ESP_LOGI(TAG, "Sent frame %u bytes in %u ms (rssi check via STAT)",
                             (unsigned)image_len, (unsigned)s_last_send_ms);
            }
            ESP_LOGE(TAG, "Connection lost! Closing socket and restarting sequence in 3 seconds...");

            // Không còn client -> capture về chế độ chờ 1 fps
            camera_set_stream_active(false);

            if (ctrl_sock >= 0) {
                close(ctrl_sock);
                ctrl_sock = -1;
                s_ctrl_rx_len = 0;
            }
            close(sock);
            vTaskDelay(pdMS_TO_TICKS(3000));
        }
    }
}