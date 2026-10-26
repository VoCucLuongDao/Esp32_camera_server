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

/* ============================================================================
 * CẤU HÌNH CAMERA CÓ THỂ CHỈNH TỪ XA (TCP control channel, lệnh "CFG k=v ...")
 * Mọi trường đều là int để có thể duyệt bảng key -> offset một cách tổng quát.
 * ========================================================================== */
typedef struct {
    int brightness;      // -2 .. 2
    int contrast;        // -2 .. 2
    int saturation;      // -2 .. 2
    int sharpness;       // -2 .. 2
    int ae_level;        // -2 .. 2
    int denoise;         //  0 .. 3
    int quality;         //  0 .. 63  (số càng nhỏ JPEG càng nét / càng nặng)
    int framesize;       //  0 .. 16  (framesize_t, xem bảng FRAMESIZE_*)
    int gainceiling;     //  0 .. 6   (gainceiling_t: 2X .. 128X)
    int wb_mode;         //  0 .. 4   (0 auto, 1 nắng, 2 mây, 3 văn phòng, 4 đèn)
    int effect;          //  0 .. 6   (0 none, 1 âm bản, 2 trắng đen, ...)
    int hmirror;         //  0 / 1
    int vflip;           //  0 / 1
    int aec;             //  1 = auto exposure, 0 = thủ công (dùng aec_value)
    int agc;             //  1 = auto gain,     0 = thủ công (dùng agc_gain)
    int aec_value;       //  0 .. 1200 (chỉ có hiệu lực khi aec = 0)
    int agc_gain;        //  0 .. 30   (chỉ có hiệu lực khi agc = 0)
    int bpc;             //  0 / 1  black pixel correction
    int wpc;             //  0 / 1  white pixel correction
    int lenc;            //  0 / 1  lens shading correction
    int raw_gma;         //  0 / 1  gamma RAW
    int fps;             //  1 .. 30 tốc độ gửi frame

    /* Kích thước khung hình tuỳ chỉnh (bỏ qua framesize khi custom = 1)
     *   width  : 96 .. 2048 (số chẵn)
     *   height : 96 .. 1536 (số chẵn)
     * Giới hạn trên là vùng cảm biến OV3660 (2048x1536). Kích thước thực tế
     * có thể bị thu nhỏ theo tỉ lệ khung gần nhất (xem ov3660_apply_custom_res). */
    int custom;          //  0 = dùng framesize, 1 = dùng width/height
    int width;
    int height;
} cam_config_t;

// Giá trị mặc định ban đầu (gọi 1 lần trong app_main trước khi tạo task)
void camera_config_defaults(void);

// Lấy bản copy cấu hình hiện tại (thread-safe)
void camera_config_get(cam_config_t *dst);

// Ghi cấu hình mới + đánh dấu "dirty"; task capture sẽ tự áp dụng vào sensor
void camera_config_set(const cam_config_t *src);

// Task capture gọi hàm này mỗi vòng lặp: trả về true nếu có cấu hình mới cần áp dụng
bool camera_config_take_pending(cam_config_t *dst);

/* Số liệu đo được ở task capture (dùng cho lệnh "STAT" của kênh điều khiển).
 * Cho biết thời gian phơi sáng thật của cảm biến: khi trời tối, AEC tự tăng
 * exposure lên hàng trăm ms -> FPS tụt mạnh dù mạng vẫn nhanh. */
typedef struct {
    uint32_t frames;      // tổng số frame đã capture từ lúc boot
    uint32_t invalid;     // số frame bị bỏ vì không có marker JPEG SOI (0xFFD8)
    uint32_t fps_x10;     // fps capture đo trong 1 giây gần nhất, nhân 10
    uint32_t last_len;    // kích thước JPEG của frame gần nhất (byte)
    int      exposure;    // 0x3501/0x3502 (đơn vị dòng)
    int      gain;        // 0x3508/0x3509 (thô, 1x = 16)
    int      ae_manual;   // giá trị thanh ghi 0x3503
    int      out_w;       // chiều rộng khung đang áp dụng (sau khi kẹp tỉ lệ)
    int      out_h;       // chiều cao khung đang áp dụng
    uint32_t heap_free;
    uint32_t heap_min;
    uint32_t psram_free;
} cam_stats_t;

void camera_stats_get(cam_stats_t *dst);

// Task mạng dùng để biết có frame mới (tránh gửi lặp lại 1 frame)
uint32_t camera_frame_seq(void);

/* Báo cho task capture biết có client đang xem hay không:
 *   active = true  -> chạy đúng FPS đặt trên UI
 *   active = false -> chạy 1 fps (tiết kiệm điện, giảm nóng cảm biến) */
void camera_set_stream_active(bool active);
bool camera_stream_active(void);

/* Camera đã khởi tạo thành công chưa. Task capture tự thử lại mỗi 5 giây nếu
 * lỗi, nên mạng/UI vẫn chạy bình thường khi camera hỏng hoặc chưa cắm. */
bool camera_is_ready(void);

// Khai báo các hàm tương tác chuẩn VideoCapture
bool video_capture_is_opened(esp_video_capture_t* cap);
bool video_capture_read(esp_video_capture_t* cap, camera_fb_t** frame);
void video_capture_release_frame(esp_video_capture_t* cap, camera_fb_t* frame);
void video_capture_close(esp_video_capture_t* cap);

bool camera_init(esp_video_capture_t* cap);
void camera_capture_individual_task(void *pvParameters) ;

#endif // ESP_VIDEO_CAPTURE_H