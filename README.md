# Esp32_camera_server

ESP32-CAM (OV3660) làm **server camera**, PC chạy Python làm **client + bảng điều khiển**
để chỉnh thông số cảm biến theo thời gian thực.

## Kiến trúc

| Kênh | Hướng | Cổng | Nội dung |
|------|-------|------|----------|
| UDP discovery | ESP32 → PC | 5005 | ESP32 broadcast `DISCOVER_SERVER`, PC trả `KEY-IP <ip>` |
| TCP video | ESP32 → PC | 1357 | `AUTH_OK` rồi stream frame `[uint32 len][JPEG]` |
| TCP control | PC → ESP32 | 1358 | Lệnh text `CFG key=value ...`, phản hồi `OK` / `ERR` |

| Tên lệnh | Phản hồi | Ý nghĩa |
|---|---|---|
| `CFG key=value ...` | `OK` / `ERR invalid` | Đổi thông số camera (chỉ cần gửi key muốn đổi) |
| `PING` | `PONG` | Kiểm tra kênh điều khiển |
| `STAT` | `STAT fps=.. exp=.. gain=.. cam=1 rssi=..` | Số liệu chẩn đoán (xem dưới) |
| `RESTART` | `OK restart` | Khởi động lại ESP32 |

Tách riêng cổng control vì cổng 1357 đang mang dữ liệu nhị phân — chèn text vào sẽ
làm hỏng bộ đếm khung của client.

### Đọc `STAT` để biết vì sao FPS thấp

| Trường | Ý nghĩa |
|---|---|
| `fps` | Số frame capture thật ở task capture (không phải tốc độ mạng) |
| `len` | Kích thước JPEG frame gần nhất |
| `res` | Kích thước khung **đang thực sự áp dụng** (sau khi kẹp tỉ lệ), ví dụ `res=1280x1024` |
| `custom` | `1` = đang dùng kích thước tuỳ chỉnh, `0` = dùng `framesize` |
| `cam` | `1` = camera đã khởi tạo, `0` = **camera lỗi/chưa cắm** (firmware đang tự thử lại mỗi 5 s) |
| `exp` | Giá trị phơi sáng `0x3501/0x3502`. **≥ 1000 dòng nghĩa là trời tối, cảm biến phơi sáng gần 1 giây → FPS thấp là do thiếu sáng, không phải lỗi mạng** |
| `gain` | Gain `0x3508/0x3509` (thô, 1x = 16) |
| `send` | Thời gian đẩy 1 frame qua TCP |
| `rssi` | Cường độ WiFi; dưới −75 dBm là yếu |
| `heap` / `min` / `psram` | RAM nội bộ còn trống / thấp nhất / PSRAM còn trống |

Nút **Đọc trạng thái** trên UI gửi `STAT` và in kết quả kèm gợi ý nguyên nhân.

### Nhịp chạy (tiết kiệm điện)

- **Không có client** (Python chưa kết nối): camera chạy **1 fps** để giảm nóng, giảm dòng tiêu thụ.
- **Có client** đã xác thực: chạy đúng **FPS đặt trên UI** (mặc định 15).
- Khi client kết nối/ngắt giữa lúc đang chờ, capture phản ứng trong ~50 ms.

## Chạy

```bash
# 1. Nạp firmware
pio run -t upload

# 2. Chạy client trên PC (cùng mạng Wi-Fi với ESP32)
python record.py
```

Cửa sổ Tkinter chứa toàn bộ tham số; video hiển thị ở cửa sổ OpenCV "ESP32-CAM Stream".
Nhấn `q` ở cửa sổ video hoặc đóng cửa sổ điều khiển để thoát.

## Thông số chỉnh được từ UI

**Tab Hình ảnh:** độ sáng, tương phản, bão hoà, độ nét, khử nhiễu, bù sáng AE,
trần gain, chất lượng JPEG (0x4407), phơi sáng thủ công (0x3501/02), gain thủ công
(0x3508/09), FPS, độ phân giải (framesize_t), **kích thước tuỳ chỉnh W × H** và bộ
lọc hiển thị phía PC.

**Tab Cảm biến OV3660:** lật ngang (0x3821), lật dọc (0x3820), auto exposure
(0x3503 bit0), auto gain (0x3503 bit1), BPC/WPC/LENC/gamma RAW (0x5000), cân bằng
trắng và hiệu ứng màu.

**Gửi tự động:** mọi thay đổi trên UI (kéo thanh trượt, bấm checkbox, chọn
combobox, gõ W × H rồi Enter/rời ô) được gửi xuống ESP32 **sau ~0,15 giây**, và
**hai lần gửi liên tiếp luôn cách nhau ít nhất 1 giây** — thay đổi trong khoảng
chờ được gộp vào lần gửi kế tiếp (lấy giá trị cuối), nên kéo thanh trượt liên tục
không dội lệnh `CFG` xuống ESP32. Không cần bấm nút. Nút **Gửi lại** chỉ dùng khi
muốn đẩy lại toàn bộ cấu hình (ví dụ sau khi ESP32 khởi động lại), **Mặc định**
khôi phục bộ giá trị gốc rồi gửi luôn, **Đọc trạng thái** gửi `STAT` (xem mục
dưới), **Khởi động lại ESP32** reset board. Tab chỉnh sáng/tương phản/độ nét...,
tab còn lại chỉnh cảm biến OV3660.

### Giới hạn giá trị

UI kẹp mọi giá trị trước khi gửi, firmware **kẹp lại lần nữa** trong
`cam_config_sanitize()` trước khi ghi thanh ghi. Nhờ vậy một ô nhập trống, số lạ
hay dòng `CFG` sai định dạng cũng không thể tạo ra giá trị ngoài miền.

| Tham số | Miền | Tham số | Miền |
|---|---|---|---|
| `brightness` / `contrast` / `saturation` / `sharpness` / `ae_level` | −2 … 2 | `aec` / `agc` | 0 … 1 |
| `denoise` | 0 … 8 | `aec_value` (phơi sáng thủ công) | 0 … 1200 |
| `quality` (JPEG) | 0 … 63 | `agc_gain` | 0 … 30 |
| `framesize` | 0 … 16 (`FRAMESIZE_FHD`) | `bpc` / `wpc` / `lenc` / `raw_gma` | 0 … 1 |
| `gainceiling` | 0 … 6 | `fps` | 1 … 30 |
| `wb_mode` | 0 … 4 | `custom` | 0 … 1 |
| `effect` | 0 … 6 | `width` | 96 … 2048 |
| `hmirror` / `vflip` | 0 … 1 | `height` | 96 … 1536 |

Giá trị sai kiểu (chữ, dấu phẩy, ô trống) được thay bằng mặc định; số lẻ ở
`width`/`height` được làm tròn xuống số chẵn.

Giá trị mặc định mới (nút **Mặc định** khôi phục đúng bộ này) — tối ưu cho
**ảnh sạch hạt, nét vừa phải** trong điều kiện thiếu sáng:

| Tham số | Mặc định | Ghi chú |
|---|---|---|
| độ nét (`sharpness`) | **1** | nét nhưng không khuếch đại hạt nhiễu (2–3 làm hạt nổi rõ) |
| khử nhiễu (`denoise`) | **2** | miền 0…8; tăng nếu còn hạt, giảm nếu ảnh bị bệt |
| tương phản (`contrast`) | **1** | vừa phải, tránh bệt vùng tối |
| độ bão hoà (`saturation`) | **1** | tránh nhiễu màu ở vùng tối |
| bù sáng AE (`ae_level`) | **0** | đo sáng trung tính |
| trần gain | 2 (8×) | giới hạn gain — nguồn nhiễu chính khi thiếu sáng |
| chất lượng JPEG | **6** | số nhỏ = ít khối vuông hơn |
| độ phân giải | XGA 1024×768 | |
| FPS | **15** | phơi sáng lâu hơn 20 fps ⇒ gain thấp hơn ⇒ ít nhiễu hơn |

Nếu ảnh vẫn còn hạt: tăng **khử nhiễu** lên 3–4, hoặc giảm **trần gain** xuống 1
(4×, ảnh tối hơn nhưng sạch hơn), hoặc giảm FPS còn 10. Nếu ảnh bị nhoè mất chi
tiết: giảm **khử nhiễu** về 1 và tăng **độ nét** lên 2.

Ảnh mờ do **mất nét quang học** (ống kính bẩn/lệch tiêu cự) thì không tham số nào
sửa được — cần lau ống kính hoặc chỉnh tiêu cự; các giá trị trên chỉ tối ưu phần
xử lý ảnh của cảm biến.

### Khi camera lỗi hoặc mạng kém

- **Camera hỏng / chưa cắm:** task capture **không chết** mà in log và thử lại
  `esp_camera_init()` mỗi 5 giây. Task mạng vẫn chạy, UI vẫn kết nối được và
  `STAT` trả `cam=0`; UI hiện cảnh báo "CAMERA LỖI/CHƯA KHỞI TẠO".
- **Thiếu PSRAM:** nếu init ở `QXGA` thất bại, firmware tự lùi về `XGA` để camera
  vẫn có hình (khung tuỳ chỉnh lớn hơn buffer sẽ bị driver bỏ, không crash).
- **ESP32 mất điện / mất WiFi:** client tự đóng kênh video sau 15 s không có dữ
  liệu và kênh điều khiển sau 20 s không phản hồi, rồi chờ ESP32 nối lại. Khi
  nối lại, UI tự đẩy lại toàn bộ cấu hình. Kênh điều khiển được giữ sống bằng
  `PING` mỗi 5 s khi im lặng.
- **Camera treo / không có frame mới:** task mạng không quay vòng liên tục mà
  **giãn nhịp nghỉ 5 → 50 ms** (vẫn trả lời `PING`/`CFG` kịp), nên không chiếm
  CPU và không làm đói task IDLE (tránh TWDT reset).
- **Kênh điều khiển bị bắn dữ liệu liên tục:** mỗi lần chỉ xử lý tối đa 2 KB rồi
  nhường CPU, phần còn lại để vòng sau — một client lỗi không thể chiếm trọn core.
- **Truyền frame:** header 4 byte và payload được gửi đủ byte bằng vòng lặp
  (`send_all`), không còn nguy cơ header bị ghi thiếu làm lệch bộ đếm frame.
- **Cổng 1357/1358/5005 bị chương trình khác chiếm:** client báo lỗi và thử mở
  lại mỗi 5 s, không thoát.
- **Khung JPEG hỏng:** firmware kiểm tra SOI (`FFD8`) trước khi gửi và bỏ khung
  lỗi; client bỏ qua frame không giải mã được và ghi log, không dừng vòng cập nhật.
- **Đóng cửa sổ video giữa chừng:** client tự tạo lại cửa sổ và tiếp tục hiển thị.
- **Mất WiFi:** nếu ESP32 **đã từng kết nối thành công** (đã có IP) mà mất WiFi
  liên tục **quá 5 giây** → firmware **tự khởi động lại** để phục hồi. Nếu **chưa
  từng kết nối** (SSID sai, router chưa bật) thì **không reboot**, chỉ in lý do
  ngắt kết nối và thử lại vô hạn cho đến khi có WiFi — tránh vòng lặp khởi động
  lại. Ngưỡng 5 s nằm ở `WIFI_DROP_REBOOT_MS` trong `src/esp_network.c`.

Giới hạn đã biết: nếu cảm biến treo **trong lúc đang chờ DMA** (`esp_camera_fb_get()`
block), task capture không thể tự phục hồi. Khi đó bấm **Khởi động lại ESP32**
(hoặc `RESTART`). Nếu `fb_get()` trả `NULL` (DMA lỗi) thì sau ~3 giây firmware
tự deinit và init lại camera ngay trong task capture.

### Kích thước khung hình tuỳ chỉnh

Tích **Kích thước tuỳ chỉnh (W × H)** rồi nhập chiều rộng/cao (UI gửi `custom=1
width=... height=...`). Khi bật, ô "Độ phân giải" bị khoá vì chỉ một trong hai
kiểu kích thước có hiệu lực.

| Giới hạn | Giá trị |
|---|---|
| Nhỏ nhất | 96 × 96 |
| Lớn nhất | 2048 × 1536 (vùng cảm biến OV3660) |
| Bước | 2 pixel khi nhập; cửa sổ crop canh giữa, làm tròn bội số 4 |

Cách firmware xử lý (`ov3660_apply_custom_res()` trong `esp_video_capture.c`):

1. Chọn **tỉ lệ khung gần nhất** trong 9 tỉ lệ của driver (4:3, 3:2, 16:10, 5:3,
   16:9, 21:9, 5:4, 1:1, 9:16) để lấy cửa sổ crop và thông số HTS/VTS đã được
   kiểm chứng.
2. Nếu tỉ lệ bạn nhập lệch khỏi tỉ lệ đó **≥ 1%** (ví dụ 3:4, 7:5, 2:1), firmware
   thu hẹp cửa sổ crop cho đúng tỉ lệ của bạn → **hình không bị kéo giãn**. Cửa
   sổ luôn được canh **giữa** cảm biến bằng start/end (offset), không lệch góc.
3. **Mọi kích thước** (kể cả nhỏ hơn mặc định 1024 × 768) đều **thu nhỏ toàn bộ
   cửa sổ** bằng ISP (`scale`) → giữ nguyên góc nhìn rộng của khung gốc, không
   zoom/cắt bớt vùng nhìn.
4. Với ảnh nhỏ (W×H ≤ ½ vùng tối đa) firmware bật **binning** để tăng độ nhạy
   sáng, giống `set_framesize()` của driver.
5. Nếu W/H vượt quá cửa sổ crop (ví dụ `2048×1152` → `1952×1092`) thì output bị
   kẹp lại. Kích thước thật xem ở `res=` trong `STAT` (ô log của UI cũng in ra).
6. Gọi `sensor->set_res_raw(...)` — logic sao chép từ `set_framesize()` của
   driver, nên vẫn đi qua `sensor_t`.

Lưu ý: chế độ tuỳ chỉnh chỉ thay đổi cửa sổ crop / tỉ lệ scale, **không** đổi
PLL, nên FPS tối đa vẫn như chế độ `framesize` tương ứng.

Buffer của driver (`cam_hal.c`: `recv_size = w*h/5` cho JPEG) được cấp phát một
lần lúc `esp_camera_init()`, nên firmware **init ở `FRAMESIZE_QXGA`** để 2 frame
buffer đủ chỗ cho khung lớn nhất (2 × 629 KB PSRAM ≈ 1.26 MB), rồi mới đặt lại
kích thước thật (mặc định XGA 1024×768). Nếu lần init QXGA thất bại (thiếu PSRAM),
firmware tự thử lại với XGA.

## Cấu hình trong firmware

`src/esp_video_capture.c`:

- `OV3660_REG_*` — bảng địa chỉ thanh ghi 16-bit (OV3660 **không** có bank select
  qua `0xFF` như OV2640).
- `ov3660_apply_config()` — ghi/đọc thanh ghi theo 6 nhóm: system, format, timing,
  size, AEC/AGC/ISP, FIFO/compression. Chỉ log chi tiết ở lần khởi tạo.
- `camera_config_defaults()` — giá trị mặc định, sửa tại đây nếu muốn đổi gốc.
- `cam_config_sanitize()` — kẹp mọi trường vào miền hợp lệ; được gọi ở mặc định,
  lúc nhận `CFG` và ngay trước khi ghi thanh ghi.
- `camera_apply_cfg()` — chỉ ghi lại trường thực sự thay đổi, tránh giật hình.
- `ov3660_apply_custom_res()` — bảng tỉ lệ khung + `set_res_raw()` cho kích thước
  tuỳ chỉnh; `s_ratio_table` sao chép từ `ov3660_settings.h` của driver.

Task capture (core 0) là nơi duy nhất chạm vào `sensor_t`, task mạng (core 1) chỉ
ghi cấu hình vào kho chung có mutex. Vì vậy không có tranh chấp SCCB giữa 2 task.
