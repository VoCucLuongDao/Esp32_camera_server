#ifndef ESP_VIDEO_CAPTURE_H
#define ESP_VIDEO_CAPTURE_H

#include <string.h>
#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "lwip/sockets.h"   
#include "lwip/netdb.h"  
#include "esp_system.h"
#include "esp_camera.h"

// ----- ĐỊNH NGHĨA CHÂN SỐ CHO BO MẠCH (VÍ DỤ CẤU HÌNH CHUẨN ESP32-CAM) -----
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

#define FLASH_GPIO_NUM     4
#define FRAME_READY_BIT    BIT0

// Định nghĩa cấu trúc đối tượng VideoCapture
typedef struct {
    bool is_opened;
    camera_config_t config;
} esp_video_capture_t;

// Khai báo các hàm tương tác chuẩn VideoCapture
bool video_capture_is_opened(esp_video_capture_t* cap);
bool video_capture_read(esp_video_capture_t* cap, camera_fb_t** frame);
void video_capture_release_frame(esp_video_capture_t* cap, camera_fb_t* frame);
void video_capture_close(esp_video_capture_t* cap);

bool camera_init(esp_video_capture_t* cap);
void camera_capture_individual_task(void *pvParameters) ;

#endif // ESP_VIDEO_CAPTURE_H