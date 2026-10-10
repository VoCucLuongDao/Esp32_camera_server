#include "esp_video_capture.h"
#include "esp_log.h"

static const char* TAG = "VIDEO_CAP";

uint8_t *shared_frame_buf = NULL;
size_t shared_frame_len = 0;
uint8_t fps = 1;

extern SemaphoreHandle_t s_frame_mutex;

bool camera_init(esp_video_capture_t* cap)
{
    if (cap == NULL)
        return false;

    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,         // Tắt tính năng ngắt cho chân này
        .mode = GPIO_MODE_INPUT,               // Thiết lập chế độ ngõ ra
        .pin_bit_mask = (1ULL << FLASH_GPIO_NUM), // Đặt Bit Mask cho GPIO 4
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE
    };
    gpio_config(&io_conf);

    // Chân PWDN trên ESP32-CAM (GPIO 32) cần được kéo xuống Mức Thấp (0) để cấp điện
    gpio_config_t pwdn_conf = {
        .pin_bit_mask = (1ULL << PWDN_GPIO_NUM), // GPIO 32 (PWDN_GPIO_NUM)
        .mode = GPIO_MODE_OUTPUT,
        .intr_type = GPIO_INTR_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE
    };
    gpio_config(&pwdn_conf);

    gpio_set_level(PWDN_GPIO_NUM, 0); // Kéo xuống 0 để kích hoạt nguồn cấp cho camera
    vTaskDelay(pdMS_TO_TICKS(50)); 
    // Áp dụng cấu hình DMA và PSRAM do người dùng thiết lập
    cap->config.ledc_channel   = LEDC_CHANNEL_0;
    cap->config.ledc_timer     = LEDC_TIMER_0;
    cap->config.pin_d0         = Y2_GPIO_NUM;
    cap->config.pin_d1         = Y3_GPIO_NUM;
    cap->config.pin_d2         = Y4_GPIO_NUM;
    cap->config.pin_d3         = Y5_GPIO_NUM;
    cap->config.pin_d4         = Y6_GPIO_NUM;
    cap->config.pin_d5         = Y7_GPIO_NUM;
    cap->config.pin_d6         = Y8_GPIO_NUM;
    cap->config.pin_d7         = Y9_GPIO_NUM;
    cap->config.pin_xclk       = XCLK_GPIO_NUM;
    cap->config.pin_pclk       = PCLK_GPIO_NUM;
    cap->config.pin_vsync      = VSYNC_GPIO_NUM;
    cap->config.pin_href       = HREF_GPIO_NUM;
    cap->config.pin_sccb_sda   = SIOD_GPIO_NUM;
    cap->config.pin_sccb_scl   = SIOC_GPIO_NUM;
    cap->config.pin_pwdn       = PWDN_GPIO_NUM;
    cap->config.pin_reset      = RESET_GPIO_NUM;
    cap->config.xclk_freq_hz   = 24000000;          //24MHZ
    
    // Cấu hình định dạng và nén bằng phần cứng
    cap->config.pixel_format   = PIXFORMAT_JPEG;
    cap->config.frame_size     = FRAMESIZE_VGA;       // 1280x720
    cap->config.jpeg_quality   = 15;                 // Tối ưu cân bằng dung lượng/băng thông mạng
    
    // Chiến lược quản lý Bộ đệm DMA nâng cao trên PSRAM
    cap->config.fb_location    = CAMERA_FB_IN_PSRAM; // Ép buộc lưu trên PSRAM
    cap->config.fb_count       = 2;                  // Bật chế độ Ping-Pong DMA
    cap->config.grab_mode      = CAMERA_GRAB_LATEST;  // CHỈ LẤY KHUNG HÌNH MỚI NHẤT, vứt bỏ khung hình lỗi mất EOI

    // Khởi tạo driver camera phần cứng (Kích hoạt EDMA/LCD_CAM DMA)
    esp_err_t err = esp_camera_init(&cap->config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera initialization is failured: 0x%x", err);
        cap->is_opened = false;
        return false;
    }
sensor_t *s = esp_camera_sensor_get();
if (s != NULL) {
    // --- BƯỚC 1: ĐỊNH CẤU HÌNH CROP PHẦN CỨNG ---
    s->set_res_raw(s, 1, 0, 0, 0, 0, 0, 800, 600, 640, 480, false, false);

    // --- BƯỚC 2: TẮT CHẾ ĐỘ GIẢM FPS TỰ ĐỘNG THIẾU SÁNG ---
    s->set_reg(s, 0xff, 0xff, 0x01); // Vào Bank 1
    s->set_reg(s, 0x2d, 0xff, 0x00); // Khóa tính năng tự động drop FPS khi tối
    s->set_reg(s, 0x2e, 0xff, 0x00); 

  // --- BƯỚC 3: TẮT MÀN HÌNH TRẬP TỰ ĐỘNG (BẮT BUỘC) ---
    s->set_exposure_ctrl(s, 0);       

    // --- BƯỚC 4: TĂNG MẠNH GIÁ TRỊ PHƠI SÁNG THỦ CÔNG (MANUAL EXP) ---
    s->set_reg(s, 0xff, 0xff, 0x01); // Đảm bảo ở Bank 1
    
    // Tăng giá trị phơi sáng (AEC) lên mức cao để cảm biến kịp ăn sáng trong phòng
    s->set_reg(s, 0x04, 0xff, 0x03); // Tăng bit cao (AEC[15:10])
    s->set_reg(s, 0x10, 0xff, 0xff); // Đẩy bit trung tâm (AEC[9:2]) lên tối đa (0xFF)
    s->set_reg(s, 0x45, 0xff, 0xff); // Đẩy bit thấp (AEC[1:0]) lên tối đa

    // --- BƯỚC 5: ĐIỀU CHỈNH GAIN (ĐỘ KHUẾCH ĐẠI ĐIỆN TỬ) ĐỂ GIẢM SỌC ---
    // Do tắt AEC, ta nên thử tắt cả AGC (Tự động Gain) và đặt Gain cố định 
    // để triệt tiêu hiện tượng sọc ngang tự động nhảy trên màn hình.
    s->set_gain_ctrl(s, 0);           // Tắt tự động tăng Gain (AGC = 0)
    
    // Đặt giá trị Gain thủ công (Thanh ghi 0x00 ở Bank 1)
    // Giá trị từ 0x00 đến 0x3F. Thử mức trung bình-cao 0x20 để ảnh sáng mà ít nhiễu sọc.
    s->set_reg(s, 0x00, 0xff, 0x20); 

    // --- BƯỚC 6: GIỮ CÂN BẰNG TRẮNG VÀ BỘ LỌC NHIỄU ĐIỂM ---
    s->set_whitebal(s, 1);       
    s->set_awb_gain(s, 1);       
    s->set_dcw(s, 1);             
    s->set_lenc(s, 1);            
    s->set_bpc(s, 1);                
    s->set_wpc(s, 1);                
    s->set_brightness(s, 1);         // Tăng nhẹ độ sáng kỹ thuật số phần mềm lên +2

    vTaskDelay(pdMS_TO_TICKS(1200)); 
    ESP_LOGI(TAG, "Auto-Exposure (AEC) Disabled. Shutter speed forced to manual fast mode.");
}
    ESP_LOGI(TAG, "Camera initialization is sucessfull (Resolution: HD, Storge: PSRAM)");
    cap->is_opened = true;
    return true;
}

bool video_capture_is_opened(esp_video_capture_t* cap) {
    return (cap != NULL && cap->is_opened);
}

// Hàm giả lập cap.read(frame)
bool video_capture_read(esp_video_capture_t* cap, camera_fb_t** frame) {
    if (!video_capture_is_opened(cap) || frame == NULL) return false;

    // Hàm này sẽ block luồng cho đến khi bộ điều khiển DMA điền đầy 1 Frame Buffer trong PSRAM
    *frame = esp_camera_fb_get();
    if (*frame == NULL) {
        ESP_LOGE(TAG, "DMA lỗi hoặc không thể lấy Frame từ PSRAM Queue");
        return false;
    }
    return true;
}

// Giải phóng khung hình trả lại cho Pool của bộ điều khiển DMA phần cứng
void video_capture_release_frame(esp_video_capture_t* cap, camera_fb_t* frame) {
    if (frame != NULL) {
        esp_camera_fb_return(frame);
    }
}

void video_capture_close(esp_video_capture_t* cap) {
    if (cap != NULL && cap->is_opened) {
        esp_camera_deinit();
        cap->is_opened = false;
        ESP_LOGI(TAG, "Đã đóng VideoCapture, giải phóng tài nguyên DMA.");
    }
}

// ==================== [TASK CHỤP ẢNH ĐỘC LẬP - CHẠY TẠI CORE 0] ====================
void camera_capture_individual_task(void *pvParameters) 
{
    esp_video_capture_t cap = {0};
    if (!camera_init(&cap)) {
        ESP_LOGE("CAM_TASK", "Cannot initialize VideoCapture.");
        vTaskDelete(NULL);
        return;
    }

    camera_fb_t *fb = NULL;
    ESP_LOGI("CAM_TASK", "Individual Capture Task start working at Core 0.");
    TickType_t start_time = 0;
    int cnt = 0;
    while (1) 
    {
        start_time = xTaskGetTickCount();
        
        // 1. Chờ và lấy khung hình từ DMA hoán đổi tự động
        if (video_capture_read(&cap, &fb))
        {
            if(shared_frame_buf == NULL || (uint16_t)(shared_frame_len*1.1) < fb->len)
            {
                uint8_t* tmp = realloc(shared_frame_buf, (uint16_t)(fb->len*1.1));
                if(tmp != NULL)
                {
                    shared_frame_buf = tmp;
                    shared_frame_len = fb->len; 
                }
                else
                {
                    ESP_LOGE(TAG, "Cannot realloc memory");
                    video_capture_release_frame(&cap, fb);
                    vTaskDelay(pdMS_TO_TICKS(100));
                    continue;
                }
            }

            // 2. Chiếm quyền bảo vệ Mutex thật nhanh
            if (xSemaphoreTake(s_frame_mutex, pdMS_TO_TICKS(1)) == pdTRUE) 
            {
                memcpy(shared_frame_buf, fb->buf, fb->len);
                shared_frame_len = fb->len;
                xSemaphoreGive(s_frame_mutex);
            }
                
            cnt++;
            if(cnt%(10*fps) == 0)
                ESP_LOGI(TAG, "Frame captured in %d ms, len %d, speed %dfps", pdTICKS_TO_MS(xTaskGetTickCount() - start_time), shared_frame_len, fps);
            video_capture_release_frame(&cap, fb);
        }

        xTaskDelayUntil(&start_time, pdMS_TO_TICKS(1000/fps));
    }
}
