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
    cap->config.xclk_freq_hz   = 20000000;          //24MHZ

    // Cấu hình định dạng và nén bằng phần cứng
    cap->config.pixel_format   = PIXFORMAT_JPEG;
    cap->config.frame_size     = FRAMESIZE_XGA;       // 1280x720
    cap->config.jpeg_quality   = 8;                 // Tối ưu cân bằng dung lượng/băng thông mạng

    // Chiến lược quản lý Bộ đệm DMA nâng cao trên PSRAM
    cap->config.fb_location    = CAMERA_FB_IN_PSRAM; // Ép buộc lưu trên PSRAM
    cap->config.fb_count       = 1;                  // Bật chế độ Ping-Pong DMA
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
        // --- 1. GIỮ NGUYÊN CẤU HÌNH RAW VGA THEO YÊU CẦU CỦA BẠN ---
        s->set_res_raw(s, 1, 0, 0, 0, 0, 0, 800, 600, 640, 480, false, false);

        // --- 2. GIỮ CẤU HÌNH MÀU SẮC ĐẬM NÉT ---
        s->set_brightness(s, 1);     // Giữ độ sáng phần mềm tối đa (+2)
        s->set_contrast(s, 1);       
        s->set_saturation(s, 2);     // Giữ màu sắc đậm đà, không bị bệt khi nâng sáng
        s->set_special_effect(s, 0); 

        s->set_whitebal(s, 1);       
        s->set_awb_gain(s, 1);       
        s->set_wb_mode(s, 0);        

        // --- 3. ĐIỀU TỐC XUNG NHỊP CHỐNG NGHẼN BỘ ĐỆM (GIỮ NGUYÊN ĐỘ MƯỢT) ---
        s->set_reg(s, 0xff, 0xff, 0x00); // Vào Bank 0 để chỉnh luồng DSP
        s->set_reg(s, 0xd3, 0xff, 0x1F); // Giữ bộ chia xung nhịp ở mức tối đa để chống giật lag

        // --- 4. GIẢI PHÁP LÀM SÁNG BỪNG VÙNG NGƯỢC SÁNG VÀ DIỆT SỌC ĐÈN ---
        s->set_exposure_ctrl(s, 1);  // Bật tự động đo sáng
        s->set_aec2(s, 1);           // [BẬT LÊN 1]: Kích hoạt mạch AEC2 chuyên dụng để tự động nhận diện và cứu sáng vùng ngược sáng (khuôn mặt)
        s->set_ae_level(s, 2);       // [TĂNG LÊN 2]: Ép mức trần bù sáng tự động lên cao nhất để đẩy toàn bộ vùng tối sáng lên

        s->set_reg(s, 0xff, 0xff, 0x01); // Vào Bank 1
        s->set_reg(s, 0x11, 0xff, 0x01); // Giữ tỷ lệ phân tần nội bộ mượt với bộ đệm
        s->set_reg(s, 0x3d, 0x03, 0x01); // Bộ lọc tần số quét vật lý 50Hz phần cứng chống sọc đèn phòng
        
        // Cấu hình lại ma trận đo sáng thông minh sang chế độ tự động cân bằng dải sáng động (Auto WDR)
        s->set_reg(s, 0x13, 0xff, 0xc7); // [ĐỔI TỪ 0x07 LÊN 0xC7]: Cho phép chip tự động tính toán bù phơi sáng cho vùng tối sâu khi bị ngược sáng
        s->set_reg(s, 0x4d, 0xff, 0x40); // Khóa đỉnh dải quét bóng đèn, triệt tiêu dải sọc sậm màu ngang người

        // --- 5. TĂNG MẠNH TRẦN ĐỘ LỢI SÁNG (GAIN) ĐỂ CỨU CHI TIẾT NGƯỜI VÀ MẶT ---
        s->set_gain_ctrl(s, 1);      
        s->set_gainceiling(s, (gainceiling_t)4); // [NÂNG LÊN MỨC 4 - GAINCEILING_16X]: Đẩy trần kích sáng phần cứng giúp kéo rõ nét các chi tiết trong bóng râm

        // --- 6. GIỮ CÁC BỘ LỌC HOÀN THIỆN VÀ CHIỀU XOAY ẢNH CHUẨN ---
        s->set_bpc(s, 1);            
        s->set_wpc(s, 1);            
        s->set_raw_gma(s, 1);        
        s->set_lenc(s, 1);           
        s->set_dcw(s, 1);            
        s->set_colorbar(s, 0);       

        s->set_hmirror(s, 1);        
        s->set_vflip(s, 1);          

        vTaskDelay(pdMS_TO_TICKS(1500)); 
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
        ESP_LOGE(TAG, "DMA error can not retrieve a framefrom PSRAM Queue");
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
    uint16_t cnt = 0;
    uint16_t max_len = 0;
    uint16_t retry = 0;
    while (1)
    {
        // 1. Chờ và lấy khung hình từ DMA hoán đổi tự động
        if (video_capture_read(&cap, &fb))
        {
            start_time = xTaskGetTickCount();
            if(shared_frame_buf == NULL || max_len < fb->len)
            {
                max_len = fb->len;
                uint8_t* tmp = (uint8_t *)heap_caps_realloc(shared_frame_buf, max_len, MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
                if(tmp != NULL)
                {
                    shared_frame_buf = tmp;
                    shared_frame_len = max_len; 
                }
                else
                {
                    ESP_LOGE(TAG, "Cannot realloc memory");
                    video_capture_release_frame(&cap, fb);
                    vTaskDelay(pdMS_TO_TICKS(100));
                    retry++;
                    if(retry > 10)
                        esp_restart();
                    continue;
                }
            }
            retry = 0;

            // 2. Chiếm quyền bảo vệ Mutex thật nhanh
            if (xSemaphoreTake(s_frame_mutex, pdMS_TO_TICKS(1000)) == pdTRUE) 
            {
                memcpy(shared_frame_buf, fb->buf, fb->len);
                shared_frame_len = fb->len;
                xSemaphoreGive(s_frame_mutex);
                vTaskDelay(pdMS_TO_TICKS(1));
                xTaskDelayUntil(&start_time, pdMS_TO_TICKS(1000/fps));
            }
            else
            {
                vTaskDelay(pdMS_TO_TICKS(10));
            }
            cnt++;
            if(cnt%fps == 0)
                ESP_LOGI(TAG, "Frame captured in %d ms, len %d, speed %dfps", pdTICKS_TO_MS(xTaskGetTickCount() - start_time), shared_frame_len, fps);
            video_capture_release_frame(&cap, fb);
        }
        else
        {
            ESP_LOGE(TAG, "DMA error can not retrieve a frame from PSRAM Queue");
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
}
