#include "esp_serial.h"
#include "esp_video_capture.h"
#include "esp_network.h"
#include "esp_flash.h"       // 👉 Thư viện driver Flash thế hệ mới của ESP-IDF v5.x
#include "nvs_flash.h"
#include <string.h>

static const char *TAG = "MAIN_APP";

SemaphoreHandle_t s_frame_mutex;

// MẢNG HOẠT ĐỘNG TOÀN CỤC (uint16_t - 24 bytes)
uint16_t g_cam_raw_params[12] = {0, 0, 0, 0, 0, 0, 1024, 728, 1024, 728, false, false};

void app_main(void)
{
    init_serial_debug();  
    ESP_LOGI(TAG, "Welcome");
    
    // Khởi động NVS hệ thống (vẫn cần cho Wi-Fi chạy)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    s_frame_mutex = xSemaphoreCreateMutex();

    // Khởi tạo kho cấu hình camera (mặc định + mutex) trước khi tạo task
    camera_config_defaults();

    // Khởi tạo các Task xử lý hình ảnh và mạng
    BaseType_t ok;

    // Task camera: tự thử lại nếu camera lỗi (không làm chết hệ thống)
    ok = xTaskCreatePinnedToCore(camera_capture_individual_task, "indiv_capture_task",
                                 4096, NULL, 10, NULL, 0);
    if (ok != pdPASS)
        ESP_LOGE(TAG, "Khong tao duoc task camera (thieu RAM?) - van chay task mang");

    // Task mạng: discovery + stream + kênh điều khiển
    ok = xTaskCreatePinnedToCore(video_stream_task, "video_stream_task",
                                 4096, NULL, 10, NULL, 1);
    if (ok != pdPASS)
        ESP_LOGE(TAG, "Khong tao duoc task mang (thieu RAM?)");

    while(1)
    {

        vTaskDelay(pdMS_TO_TICKS(1000)); // Quét định kỳ cờ hiệu sau mỗi 100ms
    }
}