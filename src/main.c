#include "esp_serial.h"
#include "esp_video_capture.h"
#include "esp_network.h"

static const char *TAG = "MAIN_APP";

SemaphoreHandle_t s_frame_mutex;

void app_main(void)
{

   // init_serial_debug();  
    ESP_LOGI(TAG, "Welcome");
    //
    // 1. Flash (NVS) is required for Wi-Fi
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    s_frame_mutex = xSemaphoreCreateMutex();
    
    xTaskCreatePinnedToCore(camera_capture_individual_task, "indiv_capture_task", 4096, NULL, 10, NULL, 0);
    //xTaskCreatePinnedToCore(start_host_web_server, "socket_server", 4096, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(video_stream_task, "video_stream_task", 4096, NULL, 5, NULL, 1);

    while(1)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}