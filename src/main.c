#include "esp_serial.h"
#include "esp_video_capture.h"
#include "esp_network.h"
#include "esp_flash.h"       // 👉 Thư viện driver Flash thế hệ mới của ESP-IDF v5.x
#include "nvs_flash.h"
#include <string.h>

static const char *TAG = "MAIN_APP";

SemaphoreHandle_t s_frame_mutex;

// MẢNG HOẠT ĐỘNG TOÀN CỤC (uint16_t - 24 bytes)
uint16_t g_cam_raw_params[12] = {0, 0, 0, 0, 0, 0, 640, 480, 640, 480, 0, 0};

volatile bool g_flash_write_requested = false;
uint16_t g_flash_write_buffer[12] = {0};

// Địa chỉ tuyệt đối mặc định (Góc cuối Flash 4MB)
#define TEST_FLASH_ABSOLUTE_ADDR   0x3F0000 

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

    // =========================================================================
    // 📥 ĐỌC DỮ LIỆU TỪ ĐỊA CHỈ MẶC ĐỊNH LÊN MẢNG HOẠT ĐỘNG KHI KHỞI ĐỘNG CHIP (CÚ PHÁP V5)
    // =========================================================================
    uint16_t stored_cfg[12];
    esp_err_t err = esp_flash_read(NULL, stored_cfg, TEST_FLASH_ABSOLUTE_ADDR, sizeof(stored_cfg));

    if (err == ESP_OK && stored_cfg[8] != 0 && stored_cfg[9] != 0) { 
        memcpy(g_cam_raw_params, stored_cfg, sizeof(g_cam_raw_params));
        ESP_LOGI(TAG, "Successfully read from 0x%X RAW: %u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u", TEST_FLASH_ABSOLUTE_ADDR,
                 g_cam_raw_params[0], g_cam_raw_params[1], g_cam_raw_params[2], g_cam_raw_params[3],
                 g_cam_raw_params[4], g_cam_raw_params[5], g_cam_raw_params[6], g_cam_raw_params[7],
                 g_cam_raw_params[8], g_cam_raw_params[9], g_cam_raw_params[10], g_cam_raw_params[11]);
    } else {
        ESP_LOGW(TAG, "⚠️ [MAIN] Address empty or invalid. System booted with factory default array parameters.");
    }

    s_frame_mutex = xSemaphoreCreateMutex();

    // Khởi tạo các Task xử lý hình ảnh và mạng
    xTaskCreatePinnedToCore(camera_capture_individual_task, "indiv_capture_task", 4096, NULL, 10, NULL, 0);
    xTaskCreatePinnedToCore(video_stream_task, "video_stream_task", 4096, NULL, 10, NULL, 1);

    // =========================================================================
    // 🔁 VÒNG LẶP WHILE(1) ĐÓN NHẬN SỰ KIỆN VÀ LỌC TRÙNG THAM SỐ RAW (CÚ PHÁP V5)
    // =========================================================================
     while(1)
    {
        if (g_flash_write_requested)
        {
            // Lọc dữ liệu trùng
            if (memcmp(g_flash_write_buffer, g_cam_raw_params, sizeof(g_cam_raw_params)) == 0)
            {
                ESP_LOGW(TAG, "⏭️ [WHILE-1] RAW parameters are IDENTICAL to current config. Flash write skipped.");
                g_flash_write_requested = false; 
            } 
            else 
            {
                ESP_LOGI(TAG, "⚡ [WHILE-1] New RAW data detected! Erasing flash sector at: 0x%X...", TEST_FLASH_ABSOLUTE_ADDR);
                esp_err_t erase_res = esp_flash_erase_region(NULL, TEST_FLASH_ABSOLUTE_ADDR, 4096);
                
                if (erase_res == ESP_OK)
                {
                    ESP_LOGI(TAG, "⚡ [WHILE-1] Writing 24 bytes of new array data...");
                    esp_err_t write_res = esp_flash_write(NULL, g_flash_write_buffer, TEST_FLASH_ABSOLUTE_ADDR, sizeof(g_flash_write_buffer));
                    
                    if (write_res == ESP_OK)
                    {
                        ESP_LOGI(TAG, "✓ [WHILE-1] Physical Flash updated seamlessly! Restarting chip now...");
                        vTaskDelay(pdMS_TO_TICKS(1000)); 
                        esp_restart(); 
                    }
                    else
                    {
                        ESP_LOGE(TAG, "❌ [WHILE-1] Write execution failed: 0x%x", write_res);
                    }
                } 
                else
                {
                    ESP_LOGE(TAG, "❌ [WHILE-1] Sector erase failed: 0x%x", erase_res);
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(1000)); // Quét định kỳ cờ hiệu sau mỗi 100ms
    }
}