#include "esp_video_capture.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include <stdlib.h>

static const char* TAG = "VIDEO_CAP";

uint8_t *shared_frame_buf = NULL;
size_t shared_frame_len = 0;
uint8_t fps = 1;

/* Số thứ tự frame đã publish; task mạng so sánh để không gửi lặp 1 frame */
static volatile uint32_t s_frame_seq = 0;

uint32_t camera_frame_seq(void)
{
    return s_frame_seq;
}

/* Số liệu chẩn đoán do task capture cập nhật */
static cam_stats_t s_stats;

/* Có client đang xem không. Task mạng bật/tắt, task capture đọc để chọn nhịp. */
static volatile bool s_stream_active  = false;
static volatile bool s_stream_changed = false;

void camera_set_stream_active(bool active)
{
    if (s_stream_active != active) {
        s_stream_active  = active;
        s_stream_changed = true;   // capture reset lại mốc thời gian ngay
    }
}

bool camera_stream_active(void)
{
    return s_stream_active;
}

/* Camera đã init được chưa (task capture retry tới khi thành công) */
static volatile bool s_cam_ok = false;

bool camera_is_ready(void)
{
    return s_cam_ok;
}

void camera_stats_get(cam_stats_t *dst)
{
    if (dst != NULL)
        *dst = s_stats;
}

extern SemaphoreHandle_t s_frame_mutex;
extern uint16_t g_cam_raw_params[12];

/* ================= CẤU HÌNH THANH GHI OV3660 (SCCB 16-bit) =================
 * OV3660 dùng địa chỉ thanh ghi 16-bit trực tiếp (SCCB_Read16 / SCCB_Write16).
 * KHÔNG có cơ chế chọn bank qua thanh ghi 0xFF như OV2640/OV7670, nên các lệnh
 * kiểu "set_reg(0xFF, 0x01)" rồi "set_reg(0x13, ...)" là vô nghĩa trên chip này.
 * Tất cả thanh ghi dưới đây được truy cập qua sensor_t->get_reg / set_reg.
 * ========================================================================= */
#define OV3660_REG_PID_H              0x300A
#define OV3660_REG_PID_L              0x300B
#define OV3660_REG_SYSTEM_CTROL0      0x3008
#define OV3660_REG_SC_PLLS_CTRL0      0x303A
#define OV3660_REG_SC_PLLS_CTRL1      0x303B
#define OV3660_REG_SC_PLLS_CTRL2      0x303C
#define OV3660_REG_SC_PLLS_CTRL3      0x303D
#define OV3660_REG_SCCB_ID            0x3100
#define OV3660_REG_AEC_PK_MANUAL      0x3503
#define OV3660_REG_AEC_EXPO_20        0x3500   /* AEC[19:16] (4 bit thấp) */
#define OV3660_REG_AEC_PK_EXPO_H      0x3501   /* AEC[15:8]  */
#define OV3660_REG_AEC_PK_EXPO_L      0x3502   /* AEC[7:4]   */
#define OV3660_REG_AEC_PK_GAIN_H2     0x350A   /* gain[9:8]  (OV3660) */
#define OV3660_REG_AEC_PK_GAIN_L2     0x350B   /* gain[7:0]  (OV3660) */
/* 0x3508/0x3509 là gain của OV2640 — KHÔNG dùng cho OV3660 */
#define OV3660_REG_LIGHT_DETECT0      0x3C1B
#define OV3660_REG_LIGHT_DETECT1      0x3C1C
#define OV3660_REG_LIGHT_DETECT2      0x3C1D
#define OV3660_REG_TIMING_TC_REG20    0x3820
#define OV3660_REG_TIMING_TC_REG21    0x3821
#define OV3660_REG_X_ADDR_ST_H        0x3800
#define OV3660_REG_X_ADDR_END_H       0x3802
#define OV3660_REG_X_OFFSET_H         0x3804
#define OV3660_REG_X_TOTAL_SIZE_H     0x3806
#define OV3660_REG_X_OUTPUT_SIZE_H    0x3808
#define OV3660_REG_X_INCREMENT        0x3814
#define OV3660_REG_Y_INCREMENT        0x3815
#define OV3660_REG_COMPRESSION_CTRL07 0x4407
#define OV3660_REG_VFIFO_CTRL02       0x4602
#define OV3660_REG_CLOCK_POL_CONTROL  0x4740
#define OV3660_REG_FORMAT_CTRL        0x501F
#define OV3660_REG_FORMAT_CTRL00      0x4300
#define OV3660_REG_ISP_CONTROL_01     0x5001
#define OV3660_REG_PRE_ISP_TEST_1     0x503D
#define OV3660_REG_SCALE_CTRL_01      0x5601

/* ----- Ghi chú: các giá trị cụ thể (flip, mirror, exposure, gain...) lấy từ
 * cam_config_t do người dùng chỉnh trên UI, không hard-code trong file này. ----- */

/* Đọc và in giá trị 1 thanh ghi ra UART để kiểm chứng cấu hình */
static bool s_ov3660_dump_enabled = true;

static void ov3660_dump_reg(sensor_t *s, uint16_t reg, const char *name)
{
    if (!s_ov3660_dump_enabled)
        return;

    int v = s->get_reg(s, reg, 0xFF);
    if (v < 0)
        ESP_LOGW(TAG, "  %-26s [0x%04X] = <read error %d>", name, reg, v);
    else
        ESP_LOGI(TAG, "  %-26s [0x%04X] = 0x%02X", name, reg, v);
}

/* Đọc exposure/gain đúng layout OV3660 để hiển thị trên STAT:
 *   exposure = {0x3500[3:0], 0x3501[7:0], 0x3502[7:0]}  (đơn vị dòng)
 *   gain     = {0x350A[1:0], 0x350B[7:0]}               (thô, 1x = 16) */
static int ov3660_read_exposure(sensor_t *s)
{
    int a = s->get_reg(s, OV3660_REG_AEC_EXPO_20, 0xFF);
    int b = s->get_reg(s, OV3660_REG_AEC_PK_EXPO_H, 0xFF);
    int c = s->get_reg(s, OV3660_REG_AEC_PK_EXPO_L, 0xFF);
    if (a < 0 || b < 0 || c < 0)
        return -1;
    return ((a & 0x0F) << 12) | ((b & 0xFF) << 4) | ((c >> 4) & 0x0F);
}

static int ov3660_read_gain_x16(sensor_t *s)
{
    int a = s->get_reg(s, OV3660_REG_AEC_PK_GAIN_H2, 0xFF);
    int b = s->get_reg(s, OV3660_REG_AEC_PK_GAIN_L2, 0xFF);
    if (a < 0 || b < 0)
        return -1;
    return ((a & 0x03) << 8) | (b & 0xFF);
}

/* Ghi 1 giá trị 16-bit thành 2 lần ghi 8-bit liên tiếp (reg, reg+1) */
__attribute__((unused))
static int ov3660_write_u16(sensor_t *s, uint16_t reg, uint16_t value)
{
    int ret = s->set_reg(s, reg,     0xFF, (value >> 8) & 0xFF);
    if (ret == 0)
        ret = s->set_reg(s, reg + 1, 0xFF, value & 0xFF);
    return ret;
}

/* ================== KÍCH THƯỚC KHUNG HÌNH TUỲ CHỈNH ======================
 * OV3660 chỉ có 1 vùng cảm biến 2048x1536. Muốn ra kích thước bất kỳ, ta chọn
 * tỉ lệ khung gần nhất trong bảng ratio_table của driver (để không bị kéo giãn
 * hình), rồi để ISP thu nhỏ (scale) hoặc gộp điểm ảnh (binning) về w x h.
 * Bảng dưới đây sao chép từ sensors/private_include/ov3660_settings.h.
 * ========================================================================= */
typedef struct {
    uint16_t mw, mh;          // kích thước tối đa của tỉ lệ này
    uint16_t sx, sy;          // cửa sổ crop bắt đầu
    uint16_t ex, ey;          // cửa sổ crop kết thúc
    uint16_t ox, oy;          // offset
    uint16_t tx, ty;          // tổng kích thước (HTS/VTS)
    uint8_t  ar_x, ar_y;      // tỉ lệ (dùng để chọn khung gần nhất)
} ov3660_ratio_t;

static const ov3660_ratio_t s_ratio_table[] = {
    { 2048, 1536,   0,   0, 2079, 1547, 16, 6, 2300, 1564,  4,  3 }, // 4:3
    { 1920, 1280,  64, 128, 2015, 1419, 16, 6, 2172, 1436,  3,  2 }, // 3:2
    { 2048, 1280,   0, 128, 2079, 1419, 16, 6, 2300, 1436, 16, 10 }, // 16:10
    { 1920, 1152,  64, 192, 2015, 1355, 16, 6, 2172, 1372,  5,  3 }, // 5:3
    { 1920, 1080,  64, 242, 2015, 1333, 16, 6, 2172, 1322, 16,  9 }, // 16:9
    { 2048,  880,   0, 328, 2079, 1219, 16, 6, 2300, 1236, 21,  9 }, // 21:9
    { 1920, 1536,  64,   0, 2015, 1547, 16, 6, 2172, 1564,  5,  4 }, // 5:4
    { 1536, 1536, 256,   0, 1823, 1547, 16, 6, 2044, 1564,  1,  1 }, // 1:1
    {  864, 1536, 592,   0, 1487, 1547, 16, 6, 2044, 1564,  9, 16 }, // 9:16
};
#define RATIO_TABLE_COUNT (sizeof(s_ratio_table) / sizeof(s_ratio_table[0]))

#define CUSTOM_W_MIN   96
#define CUSTOM_W_MAX 2048
#define CUSTOM_H_MIN   96
#define CUSTOM_H_MAX 1536

/* Áp kích thước tuỳ chỉnh.
 *
 * Các bước:
 *   1. Kẹp W/H vào vùng cảm biến, làm tròn số chẵn.
 *   2. Chọn tỉ lệ khung gần nhất trong bảng (9 tỉ lệ của driver).
 *   3. Thu hẹp cửa sổ crop của tỉ lệ đó cho ĐÚNG tỉ lệ người dùng nhập, nhờ vậy
 *      hình không bị kéo giãn. Cửa sổ luôn được canh GIỮA cảm biến bằng
 *      start_x/start_y (offset), chỉ thu hẹp (không mở rộng) nên HTS/VTS lấy
 *      từ bảng vẫn luôn hợp lệ.
 *   4. Thu nhỏ (ISP scale) TOÀN BỘ cửa sổ về đúng W×H -> luôn giữ góc nhìn
 *      rộng như khung gốc, kể cả khi W×H nhỏ hơn mặc định 1024×768.
 * Binning được bật khi W×H <= 1/2 vùng tối đa (giống set_framesize của
 * driver) để tăng độ nhạy sáng cho ảnh nhỏ.
 * Kích thước thực tế trả về qua out_w/out_h (có thể nhỏ hơn yêu cầu vài pixel
 * do làm tròn, hoặc bị kẹp nếu vượt quá cửa sổ crop). */
static void ov3660_apply_custom_res(sensor_t *s, int w, int h, int *out_w, int *out_h)
{
    const int req_w = w;
    const int req_h = h;

    /* 1. Kẹp vào giới hạn cảm biến + làm tròn về số chẵn */
    w = (w < CUSTOM_W_MIN) ? CUSTOM_W_MIN : (w > CUSTOM_W_MAX ? CUSTOM_W_MAX : w);
    h = (h < CUSTOM_H_MIN) ? CUSTOM_H_MIN : (h > CUSTOM_H_MAX ? CUSTOM_H_MAX : h);
    w &= ~1;
    h &= ~1;

    /* 2. Chọn tỉ lệ khung gần nhất. Sai số = |w/h - ar_x/ar_y| * 1000
     *    (chuẩn hoá để so sánh được giữa các tỉ lệ khác mẫu số). */
    size_t best = 0;
    int best_err = 0x7FFFFFFF;
    for (size_t i = 0; i < RATIO_TABLE_COUNT; i++) {
        const ov3660_ratio_t *r = &s_ratio_table[i];
        int num = abs(w * (int)r->ar_y - h * (int)r->ar_x);
        int den = h * (int)r->ar_y;
        int err = (den > 0) ? (num * 1000 / den) : 0x7FFFFFFF;
        if (err < best_err) {
            best_err = err;
            best = i;
        }
    }
    const ov3660_ratio_t *r = &s_ratio_table[best];

    /* 3. Cửa sổ crop gốc của tỉ lệ này (toạ độ tuyệt đối trên cảm biến) */
    int win_w = (int)r->ex - (int)r->sx + 1;
    int win_h = (int)r->ey - (int)r->sy + 1;

    /* Nếu cửa sổ gốc đã đúng tỉ lệ (lệch < 1%) thì giữ nguyên giá trị đã được
     * kiểm chứng của driver; chỉ cắt bớt khi tỉ lệ người dùng nhập nằm giữa
     * hai tỉ lệ trong bảng (ví dụ 3:4, 7:5, 2:1...). Chỉ cắt bớt nên HTS/VTS
     * lấy từ bảng vẫn luôn hợp lệ. */
    long long diff = (long long)win_w * h - (long long)win_h * w;
    if (diff < 0) diff = -diff;

    if (diff * 100 >= (long long)win_h * w) {
        if ((long long)win_w * h > (long long)win_h * w)
            win_w = win_h * w / h;      // cửa sổ quá rộng
        else
            win_h = win_w * h / w;      // cửa sổ quá cao

        win_w &= ~3;                    // cảm biến yêu cầu bội số 4 cho cửa sổ JPEG
        win_h &= ~3;
        if (win_w < 64) win_w = 64;
        if (win_h < 64) win_h = 64;
    }

    /* 4. Canh GIỮA cửa sổ trong vùng tỉ lệ: đây là "offset" quyết định khung
     *    hình lấy từ trung tâm cảm biến, không lệch về góc. */
    int start_x = (((int)r->sx + (int)r->ex + 1 - win_w) / 2) & ~3;
    int start_y = (((int)r->sy + (int)r->ey + 1 - win_h) / 2) & ~3;

    /* Canh lại nếu việc làm tròn 4 pixel đẩy mép phải/vượt quá cửa sổ gốc */
    if (start_x + win_w - 1 > (int)r->ex) start_x = ((int)r->ex + 1 - win_w) & ~3;
    if (start_y + win_h - 1 > (int)r->ey) start_y = ((int)r->ey + 1 - win_h) & ~3;
    if (start_x < 0) start_x = 0;
    if (start_y < 0) start_y = 0;

    int end_x = start_x + win_w - 1;
    int end_y = start_y + win_h - 1;

    /* 5. ISP chỉ thu nhỏ được -> output không lớn hơn cửa sổ crop */
    if (w > win_w) w = win_w & ~1;
    if (h > win_h) h = win_h & ~1;

    bool binning = (w <= (int)r->mw / 2 && h <= (int)r->mh / 2);
    bool scale   = !((w == (int)r->mw && h == (int)r->mh) ||
                     (w == (int)r->mw / 2 && h == (int)r->mh / 2));

    uint16_t total_y = binning ? (uint16_t)((r->ty / 2) + 1) : r->ty;
    uint16_t off_x   = binning ? 8 : 16;
    uint16_t off_y   = binning ? 2 : 6;

    int ret = s->set_res_raw(s, start_x, start_y, end_x, end_y,
                             off_x, off_y, r->tx, total_y,
                             w, h, scale, binning);

    ESP_LOGI(TAG, "Custom %dx%d -> %dx%d (ratio %u:%u win %d,%d..%d,%d scale=%d binning=%d) ret=%d",
             req_w, req_h, w, h, (unsigned)r->ar_x, (unsigned)r->ar_y,
             start_x, start_y, end_x, end_y, scale ? 1 : 0, binning ? 1 : 0, ret);

    if (out_w) *out_w = w;
    if (out_h) *out_h = h;
}

/* Áp dụng bảng thanh ghi OV3660 theo 6 nhóm chức năng.
 * dump == true: đọc và in toàn bộ thanh ghi chính ra log (dùng ở lần init đầu). */
static void ov3660_apply_config(sensor_t *s, const cam_config_t *c, bool dump)
{
    s_ov3660_dump_enabled = dump;

    if (dump)
        ESP_LOGI(TAG, "Applying OV3660 register configuration...");

    /* --- 1. ĐIỀU KHIỂN HỆ THỐNG (0x3008, 0x303A-0x303D, 0x3100) ---
     * Chỉ ĐỌC để xác nhận: PLL do driver tính từ xclk_freq_hz, ghi đè sẽ làm
     * sai tần số PCLK và hỏng luồng DMA. */
    ov3660_dump_reg(s, OV3660_REG_PID_H,            "PID high (expect 0x36)");
    ov3660_dump_reg(s, OV3660_REG_PID_L,            "PID low  (expect 0x60)");
    ov3660_dump_reg(s, OV3660_REG_SYSTEM_CTROL0,    "SYSTEM_CTROL0");
    ov3660_dump_reg(s, OV3660_REG_SC_PLLS_CTRL0,    "PLLS_CTRL0 (bypass)");
    ov3660_dump_reg(s, OV3660_REG_SC_PLLS_CTRL1,    "PLLS_CTRL1 (multiplier)");
    ov3660_dump_reg(s, OV3660_REG_SC_PLLS_CTRL2,    "PLLS_CTRL2 (sys div)");
    ov3660_dump_reg(s, OV3660_REG_SC_PLLS_CTRL3,    "PLLS_CTRL3 (pre-div)");
    ov3660_dump_reg(s, OV3660_REG_SCCB_ID,          "SCCB ID (0x3100)");

    /* --- 2. ĐỊNH DẠNG ĐẦU RA (0x501F, 0x4300) ---
     * Driver đã đặt theo PIXFORMAT_JPEG; chỉ log để đối chiếu. */
    ov3660_dump_reg(s, OV3660_REG_FORMAT_CTRL,      "FORMAT_CTRL (0x501F)");
    ov3660_dump_reg(s, OV3660_REG_FORMAT_CTRL00,    "FORMAT_CTRL00 (0x4300)");

    /* --- 3. KHUNG HÌNH / THỜI GIAN (0x3820, 0x3821, 0x4740) ---
     * Dùng API set_hmirror/set_vflip: chúng ghi đúng cặp bit [2:1] của
     * TIMING_TC_REG20/21 và tự tính lại binning qua set_image_options(). */
    if (c != NULL) {
        s->set_hmirror(s, c->hmirror ? 1 : 0);
        s->set_vflip  (s, c->vflip   ? 1 : 0);
    }
    ov3660_dump_reg(s, OV3660_REG_TIMING_TC_REG20,  "TIMING_TC_REG20 (vflip)");
    ov3660_dump_reg(s, OV3660_REG_TIMING_TC_REG21,  "TIMING_TC_REG21 (hmirror)");
    ov3660_dump_reg(s, OV3660_REG_CLOCK_POL_CONTROL,"CLOCK_POL_CONTROL");

    /* --- 4. KÍCH THƯỚC / THU PHÓNG (0x3800-0x3815, 0x5601) ---
     * Vùng cửa sổ do set_framesize() tính toán -> tuyệt đối không ghi đè,
     * chỉ log để kiểm tra khớp với FRAMESIZE_XGA. */
    ov3660_dump_reg(s, OV3660_REG_X_ADDR_ST_H,      "X_ADDR_ST_H (crop start)");
    ov3660_dump_reg(s, OV3660_REG_X_ADDR_END_H,     "X_ADDR_END_H (crop end)");
    ov3660_dump_reg(s, OV3660_REG_X_OFFSET_H,       "X_OFFSET_H");
    ov3660_dump_reg(s, OV3660_REG_X_TOTAL_SIZE_H,   "X_TOTAL_SIZE_H");
    ov3660_dump_reg(s, OV3660_REG_X_OUTPUT_SIZE_H,  "X_OUTPUT_SIZE_H (W/H)");
    ov3660_dump_reg(s, OV3660_REG_X_INCREMENT,      "X_INCREMENT (subsample)");
    ov3660_dump_reg(s, OV3660_REG_Y_INCREMENT,      "Y_INCREMENT (subsample)");
    ov3660_dump_reg(s, OV3660_REG_SCALE_CTRL_01,    "SCALE_CTRL_01");

    /* --- 5. AEC / AGC / ISP (0x3503, 0x5001, 0x503D, 0x3C1B-0x3C1D) ---
     * 0x3503 bit0 = AEC manual, bit1 = AGC manual. Khi khoá thủ công thì ghi
     * trực tiếp cặp thanh ghi 16-bit: exposure 0x3501/0x3502, gain 0x3508/0x3509. */
    if (c != NULL) {
        uint8_t manual = (c->aec ? 0x00 : 0x01) | (c->agc ? 0x00 : 0x02);
        s->set_reg(s, OV3660_REG_AEC_PK_MANUAL, 0x03, manual);

        /* OV3660 KHÁC OV2640 — layout thanh ghi thật (xem ov3660_regs.h):
         *   exposure = {0x3500[3:0], 0x3501[7:0], 0x3502[7:0]}
         *   gain     = {0x350A[1:0], 0x350B[7:0]} / 16
         * Driver đã ghi đúng layout này trong set_aec_value()/set_agc_gain()
         * (camera_apply_cfg gọi sẵn). Ghi tay 0x3501/0x3508 như OV2640 sẽ
         * làm exposure/gain sai hoàn toàn, nên ở đây không ghi nữa. */
    }

    s->set_colorbar(s, 0);

    ov3660_dump_reg(s, OV3660_REG_AEC_PK_MANUAL,    "AEC_PK_MANUAL");
    ov3660_dump_reg(s, OV3660_REG_ISP_CONTROL_01,   "ISP_CONTROL_01");
    ov3660_dump_reg(s, OV3660_REG_PRE_ISP_TEST_1,   "PRE_ISP_TEST_1");
    ov3660_dump_reg(s, OV3660_REG_LIGHT_DETECT0,    "LIGHT_DETECT0");
    ov3660_dump_reg(s, OV3660_REG_LIGHT_DETECT1,    "LIGHT_DETECT1");
    ov3660_dump_reg(s, OV3660_REG_LIGHT_DETECT2,    "LIGHT_DETECT2");

    /* --- 6. FIFO / NÉN (0x4407, 0x4602) ---
     * Chất lượng JPEG nằm ở 6 bit thấp của 0x4407, driver quản lý qua
     * config.jpeg_quality -> chỉ log, không ghi trực tiếp. */
    ov3660_dump_reg(s, OV3660_REG_COMPRESSION_CTRL07, "COMPRESSION_CTRL07 (QS)");
    ov3660_dump_reg(s, OV3660_REG_VFIFO_CTRL02,       "VFIFO_CTRL02");

    if (dump)
        ESP_LOGI(TAG, "OV3660 config applied: mirror=%d flip=%d aec_manual=%d agc_manual=%d",
                 c ? c->hmirror : 0, c ? c->vflip : 0,
                 c ? (c->aec ? 0 : 1) : 0, c ? (c->agc ? 0 : 1) : 0);
}

/* ================== KHO CẤU HÌNH DÙNG CHUNG (THREAD-SAFE) ==================
 * Task mạng chỉ ghi vào kho này (nhanh, không chạm phần cứng), task capture
 * đọc ra và áp dụng vào sensor. Nhờ vậy mọi truy cập SCCB vẫn nằm trong 1 task.
 * ========================================================================= */
static sensor_t *s_sensor = NULL;
static cam_config_t s_cfg;
static SemaphoreHandle_t s_cfg_mutex = NULL;
static volatile bool s_cfg_dirty = false;

/* ---------------------------------------------------------------------------
 * GIỚI HẠN GIÁ TRỊ: mọi cấu hình (từ UI, từ mặc định, từ bất kỳ đâu) đều phải
 * đi qua hàm này trước khi chạm vào cảm biến. Nhờ vậy một dòng CFG sai/thiếu
 * dấu, một giá trị tràn số hay một struct bị copy dở dang cũng không thể làm
 * ghi thanh ghi sai hoặc chia cho 0.
 * ------------------------------------------------------------------------- */
static int clamp_int(int v, int lo, int hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static void cam_config_sanitize(cam_config_t *c)
{
    if (c == NULL) return;

    c->brightness  = clamp_int(c->brightness, -2, 2);
    c->contrast    = clamp_int(c->contrast, -2, 2);
    c->saturation  = clamp_int(c->saturation, -2, 2);
    c->sharpness   = clamp_int(c->sharpness, -2, 2);
    c->ae_level    = clamp_int(c->ae_level, -2, 2);
    c->denoise     = clamp_int(c->denoise, 0, 8);   // driver hỗ trợ 0..8
    c->quality     = clamp_int(c->quality, 0, 63);
    c->framesize   = clamp_int(c->framesize, 0, FRAMESIZE_FHD);
    c->gainceiling = clamp_int(c->gainceiling, 0, 6);
    c->wb_mode     = clamp_int(c->wb_mode, 0, 4);
    c->effect      = clamp_int(c->effect, 0, 6);
    c->hmirror     = clamp_int(c->hmirror, 0, 1);
    c->vflip       = clamp_int(c->vflip, 0, 1);
    c->aec         = clamp_int(c->aec, 0, 1);
    c->agc         = clamp_int(c->agc, 0, 1);
    c->aec_value   = clamp_int(c->aec_value, 0, 1200);
    c->agc_gain    = clamp_int(c->agc_gain, 0, 30);
    c->bpc         = clamp_int(c->bpc, 0, 1);
    c->wpc         = clamp_int(c->wpc, 0, 1);
    c->lenc        = clamp_int(c->lenc, 0, 1);
    c->raw_gma     = clamp_int(c->raw_gma, 0, 1);
    c->fps         = clamp_int(c->fps, 1, 30);
    c->custom      = clamp_int(c->custom, 0, 1);
    c->width       = clamp_int(c->width, CUSTOM_W_MIN, CUSTOM_W_MAX) & ~1;
    c->height      = clamp_int(c->height, CUSTOM_H_MIN, CUSTOM_H_MAX) & ~1;
}

void camera_config_defaults(void)
{
    if (s_cfg_mutex == NULL)
        s_cfg_mutex = xSemaphoreCreateMutex();

    cam_config_t d = {
        .brightness   = 0,
        .contrast     = 1,              // vừa phải: tránh cắt mất chi tiết vùng tối/sáng
        .saturation   = 1,
        .sharpness    = 1,              // 1: nét vừa phải (2..3 khuếch đại hạt nhiễu)
        .ae_level     = 0,              // đo sáng trung tính
        .denoise      = 2,              // khử nhiễu mạnh hơn (0..8, 0 = tắt)
        .quality      = 6,              // JPEG nét hơn (số nhỏ = chất lượng cao)
        .framesize    = FRAMESIZE_XGA,  // 1024x768
        .gainceiling  = 2,              // Gain 8X - giới hạn nhiễu
        .wb_mode      = 0,              // auto white balance
        .effect       = 0,              // không hiệu ứng
        .hmirror      = 0,
        .vflip        = 0,
        .aec          = 1,              // auto exposure
        .agc          = 1,              // auto gain
        .aec_value    = 400,
        .agc_gain     = 0,
        .bpc          = 1,              // sửa điểm ảnh chết
        .wpc          = 1,              // sửa điểm ảnh trắng
        .lenc         = 1,              // bù sáng viền ống kính
        .raw_gma      = 1,
        .fps          = 15,             // 15 fps: phơi sáng lâu hơn -> gain thấp -> ít nhiễu
        .custom       = 0,              // tắt: dùng bảng độ phân giải chuẩn
        .width        = 1024,           // chỉ dùng khi custom = 1
        .height       = 768,
    };

    if (s_cfg_mutex != NULL) {
        xSemaphoreTake(s_cfg_mutex, portMAX_DELAY);
        s_cfg = d;
        cam_config_sanitize(&s_cfg);
        s_cfg_dirty = true;
        xSemaphoreGive(s_cfg_mutex);
    } else {
        s_cfg = d;
        cam_config_sanitize(&s_cfg);
        s_cfg_dirty = true;
    }
}

void camera_config_get(cam_config_t *dst)
{
    if (dst == NULL) return;
    if (s_cfg_mutex == NULL) {
        *dst = s_cfg;
        return;
    }

    if (xSemaphoreTake(s_cfg_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        *dst = s_cfg;
        xSemaphoreGive(s_cfg_mutex);
    } else {
        /* Không lấy được mutex: KHÔNG để dst chứa rác (trước đây caller dùng
         * biến chưa khởi tạo -> ghi thanh ghi cảm biến bằng giá trị rác).
         * Đọc "best effort" là đủ an toàn vì mọi trường đều là int và luôn
         * nằm trong miền hợp lệ. */
        ESP_LOGW(TAG, "camera_config_get: mutex timeout, dùng bản sao không khoá");
        *dst = s_cfg;
    }
}

void camera_config_set(const cam_config_t *src)
{
    if (src == NULL || s_cfg_mutex == NULL) return;
    if (xSemaphoreTake(s_cfg_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        cam_config_t tmp = *src;
        cam_config_sanitize(&tmp);      // chặn giá trị ngoài miền ngay từ cửa vào
        s_cfg = tmp;
        s_cfg_dirty = true;
        xSemaphoreGive(s_cfg_mutex);
    } else {
        ESP_LOGW(TAG, "camera_config_set: mutex timeout, bỏ lần cập nhật này");
    }
}

bool camera_config_take_pending(cam_config_t *dst)
{
    if (dst == NULL || s_cfg_mutex == NULL) return false;

    bool changed = false;
    if (xSemaphoreTake(s_cfg_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        if (s_cfg_dirty) {
            *dst = s_cfg;
            s_cfg_dirty = false;
            changed = true;
        }
        xSemaphoreGive(s_cfg_mutex);
    }
    return changed;
}

/* Áp toàn bộ cấu hình vào sensor. Chỉ được gọi từ task capture.
 * Chỉ ghi lại các trường thực sự thay đổi để không làm giật hình khi kéo slider. */
static cam_config_t s_applied_cfg;
static bool s_applied_valid = false;

static void camera_apply_cfg(sensor_t *s, const cam_config_t *c)
{
    if (s == NULL || c == NULL) return;

    /* Bản sao đã kẹp miền: không bao giờ ghi giá trị lạ vào cảm biến */
    cam_config_t safe = *c;
    cam_config_sanitize(&safe);
    c = &safe;

    const cam_config_t *p = s_applied_valid ? &s_applied_cfg : NULL;
#define CFG_CHANGED(field) (p == NULL || p->field != c->field)

    ESP_LOGI(TAG, "Apply config: fs=%d q=%d b=%d c=%d s=%d ae=%d sh=%d dn=%d gc=%d wb=%d fx=%d hm=%d vf=%d aec=%d agc=%d fps=%d custom=%d %dx%d",
             c->framesize, c->quality, c->brightness, c->contrast, c->saturation,
             c->ae_level, c->sharpness, c->denoise, c->gainceiling, c->wb_mode,
             c->effect, c->hmirror, c->vflip, c->aec, c->agc, c->fps,
             c->custom, c->width, c->height);

    // 1. Kích thước khung hình: hoặc theo bảng chuẩn (framesize), hoặc theo
    //    kích thước tuỳ chỉnh (width x height) do người dùng nhập trên UI.
    //    Đổi custom/width/height cũng phải áp lại nên gộp chung điều kiện.
    if (CFG_CHANGED(framesize) || CFG_CHANGED(custom) ||
        CFG_CHANGED(width)     || CFG_CHANGED(height))
    {
        if (c->custom)
        {
            ov3660_apply_custom_res(s, c->width, c->height,
                                    &s_stats.out_w, &s_stats.out_h);
        }
        else
        {
            int fs = (c->framesize >= 0 && c->framesize < FRAMESIZE_INVALID)
                     ? c->framesize : FRAMESIZE_XGA;
            s->set_framesize(s, (framesize_t)fs);
            s_stats.out_w = resolution[fs].width;
            s_stats.out_h = resolution[fs].height;
            ESP_LOGI(TAG, "Frame size preset #%d -> %dx%d",
                     fs, s_stats.out_w, s_stats.out_h);
        }
    }

    // 2. Chất lượng nén JPEG (0x4407 bit[5:0])
    if (CFG_CHANGED(quality))
        s->set_quality(s, c->quality);

    // 3. Khối ISP: màu sắc, tương phản, độ nét, khử nhiễu
    if (CFG_CHANGED(brightness))  s->set_brightness(s, c->brightness);
    if (CFG_CHANGED(contrast))    s->set_contrast(s, c->contrast);
    if (CFG_CHANGED(saturation))  s->set_saturation(s, c->saturation);
    if (CFG_CHANGED(sharpness))   s->set_sharpness(s, c->sharpness);
    if (CFG_CHANGED(denoise))     s->set_denoise(s, c->denoise);
    if (CFG_CHANGED(effect))      s->set_special_effect(s, c->effect);
    if (CFG_CHANGED(wb_mode))     s->set_wb_mode(s, c->wb_mode);

    // 4. Các bộ hiệu chỉnh điểm ảnh / ống kính
    if (CFG_CHANGED(bpc))      s->set_bpc(s, c->bpc);
    if (CFG_CHANGED(wpc))      s->set_wpc(s, c->wpc);
    if (CFG_CHANGED(lenc))     s->set_lenc(s, c->lenc);
    if (CFG_CHANGED(raw_gma))  s->set_raw_gma(s, c->raw_gma);

    // 5. Lật / gương (0x3820, 0x3821)
    if (CFG_CHANGED(hmirror))  s->set_hmirror(s, c->hmirror);
    if (CFG_CHANGED(vflip))    s->set_vflip(s, c->vflip);

    // 6. AEC / AGC: bật auto thì bỏ qua giá trị thủ công
    if (CFG_CHANGED(ae_level))     s->set_ae_level(s, c->ae_level);
    if (CFG_CHANGED(gainceiling))  s->set_gainceiling(s, (gainceiling_t)c->gainceiling);
    if (CFG_CHANGED(aec))
        s->set_exposure_ctrl(s, c->aec ? 1 : 0);
    if (!c->aec && CFG_CHANGED(aec_value))
        s->set_aec_value(s, c->aec_value);      // 0x3501/0x3502
    if (CFG_CHANGED(agc))
        s->set_gain_ctrl(s, c->agc ? 1 : 0);
    if (!c->agc && CFG_CHANGED(agc_gain))
        s->set_agc_gain(s, c->agc_gain);        // 0x3508/0x3509

    // 7. Ghi trực tiếp thanh ghi 0x3501/0x3502, 0x3508/0x3509 khi khoá thủ công
    //    + đọc lại toàn bộ thanh ghi chính ra log để kiểm chứng
    ov3660_apply_config(s, c, p == NULL);

    // 8. Tốc độ gửi frame (dùng chung cho task capture và task mạng)
    fps = (uint8_t)((c->fps > 0 && c->fps <= 30) ? c->fps : 20);

    s_applied_cfg = *c;
    s_applied_valid = true;
#undef CFG_CHANGED

    ESP_LOGI(TAG, "Camera config applied.");
}

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
    cap->config.xclk_freq_hz   = 12000000;          //24MHZ
    
    // Cấu hình định dạng và nén bằng phần cứng
    cap->config.pixel_format   = PIXFORMAT_JPEG;
    /* Kích thước lúc init quyết định KÍCH THƯỚC BUFFER của driver:
     *   cam_hal.c -> recv_size = width * height / 5 (JPEG), fb_size = recv_size
     * Vì UI cho phép đặt kích thước tuỳ chỉnh (tối đa 2048x1536), phải init ở
     * QXGA để 2 frame buffer đủ chỗ cho khung lớn nhất (2 x 629 KB PSRAM).
     * Kích thước thật sẽ được đặt lại ngay sau đó bởi camera_apply_cfg()
     * (mặc định XGA 1024x768). */
    cap->config.frame_size     = FRAMESIZE_QXGA;      // 2048x1536 - mức buffer tối đa
    cap->config.jpeg_quality   = 8;                 // Tối ưu cân bằng dung lượng/băng thông mạng

    // Chiến lược quản lý Bộ đệm DMA nâng cao trên PSRAM
    cap->config.fb_location    = CAMERA_FB_IN_PSRAM; // Ép buộc lưu trên PSRAM
    cap->config.fb_count       = 2;                  // Bật chế độ Ping-Pong DMA
    cap->config.grab_mode      = CAMERA_GRAB_LATEST;  // CHỈ LẤY KHUNG HÌNH MỚI NHẤT, vứt bỏ khung hình lỗi mất EOI

    // Khởi tạo driver camera phần cứng (Kích hoạt EDMA/LCD_CAM DMA)
    esp_err_t err = esp_camera_init(&cap->config);
    if (err != ESP_OK) {
        /* QXGA cần 2 x ~629 KB PSRAM. Board thiếu PSRAM / PSRAM lỗi thì lùi về
         * XGA để camera vẫn chạy. Khi đó khung tuỳ chỉnh lớn hơn buffer sẽ bị
         * driver bỏ (không crash) - xem giải thích ở camera_init(). */
        ESP_LOGW(TAG, "Init FRAMESIZE_QXGA that bai (0x%x) - thu lai voi XGA", err);
        cap->config.frame_size = FRAMESIZE_XGA;
        err = esp_camera_init(&cap->config);
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera initialization is failured: 0x%x", err);
        s_cam_ok = false;
        cap->is_opened = false;
        return false;
    }
    s_cam_ok = true;

   sensor_t *s = esp_camera_sensor_get();
    if (s != NULL) {

                // 1. BẬT TỰ ĐỘNG ĐỂ CAMERA TỰ CÂN BẰNG ĐỘ SÁNG THEO MÔI TRƯỜNG
        s->set_exposure_ctrl(s, 1);  
        s->set_gain_ctrl(s, 1);      
        s->set_aec2(s, 1);           

        // 2. ĐO SÁNG TRUNG TÍNH + GIỚI HẠN GAIN ĐỂ GIẢM NHIỄU
        //    Trần gain 8X: đủ sáng cho phòng tiêu chuẩn mà không nổi hạt tím.
        s->set_ae_level(s, 0);       
        s->set_gainceiling(s, (gainceiling_t)2); // Gain 8X

        // 3. CHUYỂN SANG ĐO SÁNG TOÀN KHUNG HÌNH (AVERAGE METERING)
        // 👉 OV3660 KHÔNG có bank 0xFF/0x13 như OV2640. Việc đo sáng được điều
        //    khiển qua ae_level + khối AEC và các thanh ghi 0x3C1B-0x3C1D.
        //    Toàn bộ cấu hình thanh ghi nằm trong ov3660_apply_config().

        // 4. ĐỘ NÉT VỪA + KHỬ NHIỄU MẠNH (ưu tiên ảnh sạch hạt)
        s->set_whitebal(s, 1);       
        s->set_wb_mode(s, 0);        
        s->set_brightness(s, 0);     
        s->set_contrast(s, 1);       // vừa phải, tránh bệt vùng tối
        s->set_saturation(s, 1);     
        s->set_sharpness(s, 1);      // 1: nét nhưng không khuếch đại hạt nhiễu
        s->set_denoise(s, 2);        // khử nhiễu mạnh hơn (0..8)

        vTaskDelay(pdMS_TO_TICKS(1500)); 

        // 5. ÁP DỤNG CẤU HÌNH TỪ KHO CHUNG (mặc định, có thể chỉnh từ xa qua TCP)
        cam_config_t cfg;
        camera_config_get(&cfg);
        camera_apply_cfg(s, &cfg);
        camera_config_take_pending(&cfg);   // đã áp dụng xong -> xoá cờ dirty

        // 6. Giữ con trỏ sensor cho task capture dùng lại khi có cấu hình mới
        s_sensor = s;
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
        s_cam_ok = false;
        s_sensor = NULL;          // con trỏ sensor không còn hợp lệ sau deinit
        ESP_LOGI(TAG, "Đã đóng VideoCapture, giải phóng tài nguyên DMA.");
    }
}

// ==================== [TASK CHỤP ẢNH ĐỘC LẬP - CHẠY TẠI CORE 0] ====================
void camera_capture_individual_task(void *pvParameters)
{
    esp_video_capture_t cap = {0};

    /* Camera lỗi/ chưa cắm KHÔNG được làm chết task này: thử lại mỗi 5 giây để
     * khi phần cứng ổn định trở lại là có hình ngay. Task mạng và kênh điều
     * khiển vẫn chạy độc lập nên UI vẫn kết nối và xem được STAT. */
    while (!camera_init(&cap)) {
        ESP_LOGE("CAM_TASK", "Khoi tao camera that bai - thu lai sau 5 giay (mang van chay)");
        vTaskDelay(pdMS_TO_TICKS(5000));
    }

    camera_fb_t *fb = NULL;
    ESP_LOGI("CAM_TASK", "Individual Capture Task start working at Core 0.");
    TickType_t latest_time = xTaskGetTickCount();
    TickType_t frame_tick  = xTaskGetTickCount();

    // Buffer đích dùng chung với task mạng: cấp phát động, KHÔNG dùng biến 16-bit
    // cho kích thước (frame XGA có thể > 65535 byte -> trước đây bị cắt cụt JPEG).
    size_t max_len = 0;
    shared_frame_buf = NULL;
    shared_frame_len = 0;

    uint32_t cnt = 0;
    uint32_t read_fail = 0;
    cam_config_t pending;
    while (1)
    {
        // 0. Có cấu hình mới từ kênh TCP? Áp dụng ngay tại đây để mọi truy cập
        //    SCCB/sensor đều nằm trong task capture (không tranh chấp với task mạng)
        if (s_sensor != NULL && camera_config_take_pending(&pending))
        {
            camera_apply_cfg(s_sensor, &pending);
            frame_tick = xTaskGetTickCount();
        }

        // 0b. Đổi chế độ chờ/hoạt động -> reset mốc nhịp để phản ứng ngay
        if (s_stream_changed)
        {
            s_stream_changed = false;
            frame_tick = xTaskGetTickCount();
        }

        // 1. Chờ và lấy khung hình từ DMA hoán đổi tự động
        if (video_capture_read(&cap, &fb))
        {
            read_fail = 0;
            // 1.1 Frame phải bắt đầu bằng SOI (FF D8). Ngay sau khi đổi
            //     framesize/JPEG quality, driver có thể trả buffer chưa kịp
            //     mã hoá ("NO-SOI") -> gửi lên client sẽ chỉ thấy ảnh đen.
            if (fb->len < 2 || fb->buf[0] != 0xFF || fb->buf[1] != 0xD8)
            {
                s_stats.invalid++;
                ESP_LOGW("CAM_TASK", "Frame không có SOI (%u byte), bỏ", (unsigned)fb->len);
                video_capture_release_frame(&cap, fb);
                fb = NULL;
                vTaskDelay(pdMS_TO_TICKS(10));
                continue;
            }

            // 2. Giãn buffer đích. Ưu tiên PSRAM (4 MB) để không ăn RAM nội bộ;
            //    chỉ khi PSRAM hết mới rơi về vùng DMA-capable trong RAM nội.
            if (fb->len > max_len)
            {
                uint8_t *tmp = (uint8_t *)heap_caps_realloc(
                    shared_frame_buf, fb->len, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
                if (tmp == NULL)
                    tmp = (uint8_t *)heap_caps_realloc(
                        shared_frame_buf, fb->len, MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
                if (tmp != NULL)
                {
                    shared_frame_buf = tmp;
                    max_len = fb->len;
                    ESP_LOGI("CAM_TASK", "Frame buffer enlarged to %u bytes", (unsigned)max_len);
                }
            }

            if (shared_frame_buf == NULL || fb->len > max_len)
            {
                // Không đủ chỗ: bỏ hẳn frame này. Trước đây code cắt bớt JPEG
                // rồi vẫn gửi -> client nhận ảnh rách/hỏng.
                ESP_LOGW("CAM_TASK", "No buffer for %u-byte frame, dropped", (unsigned)fb->len);
            }
            else if (xSemaphoreTake(s_frame_mutex, pdMS_TO_TICKS(100)) == pdTRUE)
            {
                memcpy(shared_frame_buf, fb->buf, fb->len);
                shared_frame_len = fb->len;
                s_frame_seq++;
                xSemaphoreGive(s_frame_mutex);
                cnt++;
            }
            else
            {
                ESP_LOGW("CAM_TASK", "Frame mutex timeout, dropped");
            }

            s_stats.frames++;
            s_stats.last_len = (uint32_t)fb->len;

            video_capture_release_frame(&cap, fb);
            fb = NULL;

            // 3. Nhịp chạy: có client -> đúng FPS trên UI, không có -> 1 fps.
            //    xTaskDelayUntil tự bỏ qua nếu capture đã chậm hơn nhịp này.
            uint32_t period_ms = s_stream_active
                                 ? (1000u / (fps ? fps : 20u))
                                 : 1000u;

            if (!s_stream_active)
            {
                // Chờ theo từng bước 50 ms để phản ứng ngay khi client kết nối
                for (uint32_t waited = 0; waited < period_ms && !s_stream_changed; waited += 50)
                    vTaskDelay(pdMS_TO_TICKS(50));
                frame_tick = xTaskGetTickCount();
            }
            else
            {
                xTaskDelayUntil(&frame_tick, pdMS_TO_TICKS(period_ms));
            }
        }
        else
        {
            /* Chỉ log lần đầu và mỗi 10 lần để UART không bị ngập (log chậm
             * cũng có thể làm lệch nhịp các task khác). */
            if (read_fail == 0 || (read_fail % 10) == 9)
                ESP_LOGE(TAG, "DMA error can not retrieve a frame from PSRAM Queue");
            /* fb_get() trả NULL (driver/DMA treo) chứ không block: sau ~3 giây
             * liên tiếp thất bại thì deinit + init lại ngay trong task này
             * (task duy nhất chạm sensor nên an toàn), thay vì chạy vô ích. */
            if (++read_fail >= 30)
            {
                read_fail = 0;
                ESP_LOGW("CAM_TASK", "Mat frame 30 lan lien tiep - khoi tao lai camera");
                video_capture_close(&cap);
                shared_frame_len = 0;
                while (!camera_init(&cap))
                {
                    ESP_LOGE("CAM_TASK", "Khoi tao lai camera that bai - thu lai sau 5 giay");
                    vTaskDelay(pdMS_TO_TICKS(5000));
                }
                frame_tick = xTaskGetTickCount();
                continue;
            }
            vTaskDelay(pdMS_TO_TICKS(100));
        }

        // 4. Cập nhật số liệu chẩn đoán mỗi giây + đọc exposure/gain thật của sensor
        uint32_t elapsed = pdTICKS_TO_MS(xTaskGetTickCount() - latest_time);
        if (elapsed >= 10000)
        {
            latest_time = xTaskGetTickCount();
            s_stats.fps_x10 = (uint32_t)(cnt * 10000ULL / elapsed);

            if (s_sensor != NULL)
            {
                s_stats.exposure  = ov3660_read_exposure(s_sensor);
                s_stats.gain      = ov3660_read_gain_x16(s_sensor);
                s_stats.ae_manual = s_sensor->get_reg(s_sensor, OV3660_REG_AEC_PK_MANUAL, 0xFF);
            }
            s_stats.heap_free  = esp_get_free_heap_size();
            s_stats.heap_min   = esp_get_minimum_free_heap_size();
            s_stats.psram_free = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);

            ESP_LOGI(TAG, "Capture %u frame/s, len %u, exp %d, gain %d, heap %u",
                     (unsigned)(s_stats.fps_x10 / 10), (unsigned)shared_frame_len,
                     s_stats.exposure, s_stats.gain, (unsigned)s_stats.heap_free);            cnt = 0;
        }
    }
}
