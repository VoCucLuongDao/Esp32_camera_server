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
| `STAT` | `STAT fps=.. exp=.. maxexp=.. gain=.. cam=1 rssi=.. txp=..` | Số liệu chẩn đoán (xem dưới) |
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
| `exp` | Giá trị phơi sáng `0x3500/0x3501/0x3502` (dòng). So với `maxexp`: chạm trần nghĩa là AEC đang dùng hết phơi sáng cho phép |
| `maxexp` | Trần phơi sáng `0x3A02/0x3A03` do thanh trượt **FPS** đặt (`PCLK/(HTS×fps)`). **Giảm FPS để tăng `maxexp` → sáng hơn; tăng FPS → phơi ngắn, FPS cao hơn** |
| `gain` | Gain `0x350A/0x350B` (thô, 1x = 16) |
| `send` | Thời gian đẩy 1 frame qua TCP |
| `rssi` | Cường độ WiFi; dưới −75 dBm là yếu |
| `txp` | Công suất phát đang cấu hình, đơn vị 0,25 dBm: **`80` = 20 dBm = mức tối đa**. Nếu nhỏ hơn 80, xem gợi ý bên dưới |
| `heap` / `min` / `psram` | RAM nội bộ còn trống / thấp nhất / PSRAM còn trống |

The **Read status** button sends `STAT` and prints the result with diagnostic hints.

### Nhịp chạy (tiết kiệm điện)

- **Không có client** (Python chưa kết nối): camera chạy **1 fps** để giảm nóng, giảm dòng tiêu thụ.
- **Có client** đã xác thực: chạy đúng **FPS đặt trên UI** (mặc định 10).
- Khi client kết nối/ngắt giữa lúc đang chờ, capture phản ứng trong ~50 ms.

## Chạy

```bash
# 1. Nạp firmware
pio run -t upload

# 2. Chạy client trên PC (cùng mạng Wi-Fi với ESP32)
python record.py
```

The Tkinter window contains all settings. Click **Show video** to open the OpenCV
"ESP32-CAM Stream" window. Press `q` in the video window or close the control
panel to exit.

## UI controls

**Image tab:** brightness, contrast, saturation, sharpness, denoise, AE
compensation, gain ceiling, JPEG quality (0x4407), manual exposure
(0x3500/01/02), manual gain (0x350A/0B), FPS, resolution (framesize_t),
**custom width × height**, and the PC display filter.

**OV3660 sensor tab:** horizontal mirror (0x3821), vertical flip (0x3820), auto
exposure (0x3503 bit0), auto gain (0x3503 bit1), BPC/WPC/LENC/RAW gamma (0x5000),
white balance, and color effects.

**Automatic updates:** UI changes (sliders, checkboxes, comboboxes, or entering
width × height) are sent to the ESP32 after ~0.15 seconds. Consecutive sends are
at least 1 second apart; pending changes are combined using the latest values.
**Send** resends the full configuration, **Defaults** restores and sends the
default values, **Read status** sends `STAT`, and **Restart ESP32** reboots the
board.

### MP4 recording

The **● Record MP4** / **■ Stop recording** button saves the **unfiltered**
video stream to an `.mp4` file in `RECORD_DIR`.

```python
# record.py (đầu file)
RECORD_DIR = r"D:\Book"          # đổi thư mục lưu tại đây
RECORD_SEGMENT_SECONDS = 300     # chia file mỗi 5 phút (0 = không chia)
```

- Có thể ghi đè thư mục mà không sửa code: đặt biến môi trường `ESP32CAM_RECORD_DIR`.
- **Chia đoạn 5 phút:** cứ đủ ~300 giây video, chương trình chốt file hiện tại và
  mở đoạn kế tiếp cùng thư mục, cùng kích thước. Nhờ vậy một file lỗi/hỏng (mất
  điện, ghi đĩa lỗi) chỉ mất tối đa 5 phút, không mất toàn bộ bản ghi.
  Tên file: `esp32cam_YYYYmmdd_HHMMSS_mmm.mp4`, đoạn sau thêm `_p2`, `_p3`...
  Đổi `RECORD_SEGMENT_SECONDS` (ví dụ 600 = 10 phút) hoặc đặt `0` để ghi 1 file.
- Không ghi đè file cũ (trùng tên sẽ tự thêm `_2`, `_3`...).
- FPS của file = FPS **đo được** khi bắt đầu ghi (nếu chưa ổn định thì lấy FPS
  đang đặt trên UI) để video phát đúng tốc độ thực.
- **Đổi độ phân giải giữa lúc ghi** (hay ESP32 reboot rồi nối lại với cỡ khung
  khác) chỉ làm frame đó được **resize** cho khớp cỡ file (báo 1 dòng log),
  **không** sinh thêm file rác.
- Bấm ghi rồi dừng khi chưa nhận được frame nào → file rỗng được **xoá**, không
  để lại clip 0 giây.
- Đóng chương trình / nhấn `q` khi đang ghi → file được chốt an toàn.
- Codec thử lần lượt `mp4v` → `avc1` → `H264`; nếu máy thiếu cả ba, UI báo lỗi và
  ghi chi tiết vào ô log (cần bản OpenCV có FFmpeg).
- Thư mục `D:\Book` được tự tạo nếu chưa có; tạo thất bại thì báo lỗi, không ghi.

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
| `gainceiling` | 0 … 6 (0=2×, 4=32×, 5/6≈64×) | `fps` | 1 … 30 |
| `wb_mode` | 0 … 4 | `custom` | 0 … 1 |
| `effect` | 0 … 6 | `width` | 96 … 2048 |
| `hmirror` / `vflip` | 0 … 1 | `height` | 96 … 1536 |

Giá trị sai kiểu (chữ, dấu phẩy, ô trống) được thay bằng mặc định; số lẻ ở
`width`/`height` được làm tròn xuống số chẵn.

Giá trị mặc định mới (nút **Defaults** khôi phục đúng bộ này) — tối ưu cho
**cảnh thiếu sáng / ban đêm**: ưu tiên nhìn rõ hơn là sạch hạt tuyệt đối.

| Tham số | Mặc định | Ghi chú |
|---|---|---|
| độ sáng (`brightness`) | **1** | nâng nhẹ vùng tối |
| tương phản (`contrast`) | **1** | khối mặt rõ hơn, không bệt |
| độ bão hoà (`saturation`) | **1** | tránh nhiễu màu ở vùng tối |
| độ nét (`sharpness`) | **2** | làm nét mạnh (driver cho −3…3) để lấy lại chi tiết mặt/mép |
| khử nhiễu (`denoise`) | **3** | 4–8 làm nhoè cả khuôn mặt; 1–2 nét hơn nhưng hạt hơn |
| bù sáng AE (`ae_level`) | **2** | đẩy mục tiêu đo sáng lên → ảnh sáng hơn |
| trần gain | 5 (~64×) | **thay đổi quan trọng nhất**: trần 8× cũ làm ảnh ban đêm gần như đen |
| chất lượng JPEG | **10** | số nhỏ = nét hơn nhưng file lớn hơn; ảnh gain cao rất nhiễu nên q=6 làm frame phình to, nghẽn WiFi |
| độ phân giải | XGA 1024×768 | binning 2×2 → nhạy sáng gấp 4 lần |
| FPS | **10** | vừa là tốc độ gửi frame, vừa là **FPS tối thiểu** của cảm biến (xem mục dưới) |

The **Image** tab provides quick presets:

| Button | Main settings |
|---|---|
| **Low light** | low-light settings |
| **Face detail** | `denoise=1`, `sharpness=2`, `contrast=1`, gain ceiling 5 (~64×) — sharper detail, but a noisier image |
| **Daylight** | default daylight settings |
| **Advanced** | wall / horizontal-slot preset (640×360) |

Selecting a preset updates the settings and sends the configuration.

Nếu ảnh ban đêm vẫn tối: **giảm FPS** (5–8) để AEC được phơi sáng lâu hơn —
đây là cách tăng sáng mạnh nhất còn lại; **Bù sáng AE** đã ở mức cao nhất UI cho
phép (2) và trần gain đã ở mức tối đa cảm biến. Kiểm tra `AGC_GAIN_CEILING H/L`
trong log lúc khởi động phải là `0x03/0xFF` (xem mục dưới). Nếu ảnh quá nhiễu:
giảm **trần gain** về 4 (32×), **tăng FPS** (phơi ngắn hơn) và tăng **khử nhiễu**
lên 5–6. Nếu ảnh bị nhoè mất chi tiết: bấm **Face detail** (giảm khử nhiễu, tăng độ
nét) hoặc giảm **khử nhiễu** về 1–2 và tăng **độ nét** lên 2.

**Lưu ý quan trọng về trần gain (đã sửa trong firmware):** driver `ov3660.c`
của esp32-camera ghi thẳng chỉ số enum 0–6 vào thanh ghi `0x3A18/0x3A19`, nhưng
OV3660 nhận gain thật dạng 6.4 fixed-point (`0x10` = 1×, 10 bit → tối đa ~64×).
Hệ quả: đặt "trần gain 64×" trên UI trước đây thực chất chỉ cho AGC lên khoảng
**1×**, nên ảnh ban đêm gần như đen dù mọi thông số khác đúng. Firmware hiện map
lại đúng (`gainceiling_code()` trong `esp_video_capture.c`): 0→2×, 1→4×, 2→8×,
3→16×, 4→32×, 5/6→~64× (mức tối đa thanh ghi 10 bit). Log khởi động in
`AGC_GAIN_CEILING H/L` để kiểm chứng.

### Vì sao trước đây chỉ được 1–5 fps (đã sửa)

FPS thật của cảm biến do xung nhịp PCLK quyết định:
`fps = PCLK / (HTS × VTS)`. Driver `ov3660.c` tính PLL theo xclk **20 MHz**, nhưng
firmware cũ đặt xclk **12 MHz** → PCLK chỉ còn **6 MHz**, nên XGA (HTS 2300,
VTS 783) tối đa khoảng **3,3 fps**. Tệ hơn, AEC luôn ưu tiên tăng phơi sáng trước
khi tăng gain; trần phơi sáng mặc định 2352 dòng (gấp ~3 lần một frame) kéo FPS
xuống **~1 fps** khi thiếu sáng — đúng hiện tượng "1→5 fps".

Firmware hiện:
1. Đặt `xclk = 20 MHz` và **để driver tự chọn PLL** → **PCLK 10 MHz**, XGA đạt
   khoảng **5,5 fps**. Với xclk 12 MHz trước đây driver chỉ cho PCLK 6 MHz
   (~3,3 fps), nên đổi xclk 20 MHz đã tăng FPS ~1,7 lần mà không phải ghi đè PLL.
2. **Không đổi độ phân giải/PLL khi camera đang stream.** `frame_size` được đặt
   đúng ngay trong `esp_camera_init()`; khi người dùng đổi độ phân giải trên UI,
   firmware **deinit + init lại camera** thay vì gọi `set_framesize()` giữa frame.
   (Đây chính là nguyên nhân gốc của NO-SOI / 0 frame / "restart liên tục" —
   xem mục dưới.)
3. Giới hạn **AEC max exposure** (`0x3A02/0x3A03`) theo thanh trượt FPS:
   `max_dòng = PCLK / (HTS × fps)`. Nhờ vậy thanh trượt **FPS** giờ điều khiển
   thật: **giảm FPS → phơi sáng lâu hơn → sáng hơn nhưng dễ nhoè**; **tăng FPS →
   phơi ngắn, AGC bù bằng gain tới ~64× → nhanh nhưng hạt hơn**.
   `STAT` in thêm `maxexp=` để biết trần phơi sáng hiện tại. `fps` bị chặn trên
   theo FPS khả thi của cảm biến; còn `maxexp` **được phép vượt VTS** (tối đa
   2352 dòng ≈ 0,54 s ở PCLK 10 MHz): OV3660 tự kéo dài frame khi exposure >
   VTS, nên đặt FPS 2–4 mới thực sự sáng hơn (trước đây bị chặn ở `VTS-4` nên
   giảm FPS mà ảnh không sáng thêm).

Gợi ý chọn FPS: trong nhà thiếu sáng 3–5 fps; ban ngày 5 fps; WiFi yếu 4–5 fps.
XGA đạt tối đa khoảng **5,5 fps** ở PCLK 10 MHz; đặt cao hơn chỉ làm phơi sáng
ngắn lại chứ không tăng FPS cảm biến. Muốn FPS cao hơn thì **giảm độ phân giải**
(VGA/QVGA dùng binning).

### Vì sao không được đổi framesize/PLL khi đang stream (lỗi NO-SOI / 0 frame / restart)

PCLK vừa quyết định FPS vừa quyết định nhịp DMA. `set_framesize()` của driver
ghi lại **toàn bộ timing + PLL** (HTS/VTS, subsample, PCLK). Nếu gọi nó trong
lúc DMA đang chạy, xung PCLK đổi **giữa frame** → khối JPEG của cảm biến và bộ
nhận I2S/DMA mất đồng bộ. Trên board thật, tuỳ mức PCLK, lỗi biểu hiện khác nhau:

```
PCLK 20 MHz: NO-SOI - JPEG start marker missing   (frame rác liên tục)
PCLK 12,5 MHz: Failed to get frame: timeout       (không frame nào)
PCLK 13,3 MHz: 1 frame "HD" cũ rồi đứng           (watchdog init lại liên tục)
```

Đó là lý do firmware hiện:
- init driver **đúng framesize ngay từ đầu** (không còn frame HD "cũ" bị gửi
  nhầm sau khi đổi sang XGA),
- đổi độ phân giải trên UI = **init lại camera** (mất ~2 s, nhưng hình ổn định),
- **không ghi đè PLL** sau `set_framesize()` nữa.

Nếu vẫn thấy `NO-SOI` / `NO-DATA` / `timeout` sau thay đổi này, gửi log dòng
`Failed to get frame: timeout (vsync=… eof=… queued=… nodata=… soi_fail=…)` —
các bộ đếm cho biết frame rơi ở tầng nào.

**Vì sao mặt vẫn có thể nhoè dù đã tăng nét:** khi FPS đặt thấp, cảm biến phơi
sáng lâu (xem `exp`/`maxexp` trong `STAT`; 2352 dòng ≈ 0,54 giây/frame). Người di
chuyển trong khoảng đó sẽ bị **nhoè chuyển động** — không tham số ISP nào sửa
được, chỉ giảm được bằng cách **tăng FPS** (phơi ngắn lại, gain bù) hoặc **thêm
đèn/nguồn sáng gần mặt** (đèn hồng ngoại nếu ống kính không có filter IR). Nếu
**cả khung hình** đều mờ kể cả vật đứng yên thì là **mất nét quang học**: xoay nhẹ
thấu kính để lấy nét lại và lau ống kính.

### Khi camera lỗi hoặc mạng kém

- **Camera hỏng / chưa cắm:** task capture **không chết** mà in log và thử lại
  `esp_camera_init()` mỗi 5 giây. Task mạng vẫn chạy, UI vẫn kết nối được và
  `STAT` trả `cam=0`; UI hiện cảnh báo "CAMERA LỖI/CHƯA KHỞI TẠO".
- **Thiếu PSRAM:** firmware cấp buffer JPEG tối đa (~629 KB cho 1 frame) để chứa
  được khung 2048×1536; nếu cấp phát thất bại (board thiếu PSRAM), firmware tự lùi
  về buffer mặc định của `XGA` để camera vẫn có hình (khung lớn hơn buffer sẽ bị
  driver bỏ, không crash).
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
- **Đóng cửa sổ video giữa chừng:** click **Show video** on the control panel to reopen it.
- **WiFi "yếu" sau khi tăng gain 64×:** RSSI là mức thu từ router nên gần như
  không đổi theo tham số camera; cái tăng là **dung lượng frame**. Ảnh gain cao bị
  nhiễu, JPEG nén kém → frame có thể to gấp 3–5 lần, chiếm hết airtime WiFi nên
  cảm giác như mạng yếu (xem `len=` và `send=` trong `STAT`). Cách giảm tải, theo
  thứ tự hiệu quả:
  1. Tăng **JPEG quality** lên 12–16 (số lớn hơn = file nhỏ hơn).
  2. Giảm **FPS gửi frame** xuống 6–8.
  3. Giảm **độ phân giải** (XGA → VGA 640×480) hoặc dùng **Kích thước tuỳ chỉnh**.
  4. Nếu phải giữ 64×: giảm **trần gain** về 4 (32×) — ảnh hạt hơn nhưng frame nhỏ
     hơn và WiFi ổn hơn.
  Firmware đã đặt **TX power 20 dBm** và **băng thông 20 MHz** một lần lúc
  `wifi_init_sta()` (`esp_network.c`); `WIFI_PS_NONE` được bật nên không có hiện
  tượng ngủ làm tăng ping. Cụ thể `wifi_apply_max_tx_power()`:
  - Ghi `esp_wifi_set_max_tx_power(80)` = **20 dBm** (đơn vị 0,25 dBm), rồi
    **đọc lại** bằng `esp_wifi_get_max_tx_power()` và log mức thực tế.
  - Gọi lại sau mỗi lần **nhận IP** (`IP_EVENT_STA_GOT_IP`) để không bị hạ mức
    sau khi bắt tay.
  - `sdkconfig.esp32cam` đã có `CONFIG_ESP_PHY_MAX_WIFI_TX_POWER=20` và
    `CONFIG_ESP_PHY_REDUCE_TX_POWER` **tắt** → 20 dBm là mức cao nhất phần cứng
    cho phép; đặt cao hơn cũng bị kẹp về 20 dBm.
  Kiểm tra `txp=` trong `STAT`: phải là **80** (20 dBm). Nếu nhỏ hơn, log UART
  sẽ cảnh báo "TX power bi gioi han" — kiểm tra nguồn 5 V/2 A và cáp USB ngắn
  (camera ăn dòng nhiều khi ảnh phức tạp, nguồn yếu làm công suất phát RF giảm).
- **Đặt ESP32 trong tường / bắt sóng kém:** đây là giới hạn vật lý — mỗi lớp
  bê tông/gạch suy hao khoảng 10–20 dB, firmware không bù được. Thứ tự xử lý:
  1. Đưa camera ra ngoài tường, hướng phần ăng-ten (PCB) về phía router, tránh
     ống kim loại/tủ điện.
  2. Dùng preset **Advanced** để chọn khung hình 640×360 và giảm bitrate khi
     lắp trong tường / khe ngang; với RSSI thấp, hãy đặt router hoặc repeater gần hơn.
  3. Dùng **repeater/AP** đặt ngoài tường gần camera, hoặc chuyển router sang
     kênh 1/6/11 và băng thông 20 MHz (ít nhiễu hơn).
  4. Board có chân **IPEX/u.FL**: hàn chuyển trở 0 Ω sang anten ngoài 3–5 dBi;
     anten PCB tích hợp rất kém khi bị che.
  5. Xem `rssi=` trong `STAT`: −50 dBm tốt, −70 dBm yếu, ≤ −80 dBm rất yếu —
     mức này chỉ giảm bitrate mới cứu được. Tránh đặt camera sát nguồn tỏa nhiệt
     và dây điện lớn.
- **Mất WiFi:** nếu ESP32 **đã từng kết nối thành công** (đã có IP) mà mất WiFi
  liên tục **quá 5 giây** → firmware **tự khởi động lại** để phục hồi. Nếu **chưa
  từng kết nối** (SSID sai, router chưa bật) thì **không reboot**, chỉ in lý do
  ngắt kết nối và thử lại vô hạn cho đến khi có WiFi — tránh vòng lặp khởi động
  lại. Ngưỡng 5 s nằm ở `WIFI_DROP_REBOOT_MS` trong `src/esp_network.c`.

Giới hạn đã biết: nếu cảm biến treo **trong lúc đang chờ DMA** (`esp_camera_fb_get()`
block), task capture không thể tự phục hồi. Khi đó bấm **Khởi động lại ESP32**
(hoặc `RESTART`). Nếu `fb_get()` trả `NULL` (DMA lỗi) thì sau ~12 giây (3 lần
thất bại, mỗi lần driver chờ 4 s) firmware tự deinit và init lại camera ngay
trong task capture.

### Gửi frame khi WiFi yếu

TCP tự truyền lại gói mất nên frame **vẫn đến** khi sóng yếu — cái giảm là thông
lượng/FPS chứ không phải tính toàn vẹn. Firmware được tối ưu để không tự ngắt oan:

- `sdkconfig.esp32cam`: `CONFIG_LWIP_TCP_SND_BUF_DEFAULT` và
  `CONFIG_LWIP_TCP_WND_DEFAULT` = **16384** (mặc định 5760). Buffer lớn hơn giữ
  được nhiều gói đang bay hơn → thông lượng cao hơn khi RTT lớn/nhiều retransmit.
- `CONFIG_LWIP_TCP_OOSEQ_MAX_PBUFS=8`, `OOSEQ_TIMEOUT=10`: chịu được nhiều gói
  đến sai thứ tự (hay gặp khi sóng yếu).
- Socket video bật `TCP_NODELAY`, `SO_KEEPALIVE` và `SO_SNDTIMEO = 10 s` (thay vì
  3 s) để frame lớn gửi chậm không bị coi là đứt.
- Khi `send()` thật sự lỗi, firmware **đóng kết nối** để client nối lại và gửi
  frame mới nguyên vẹn (không gửi lại nửa frame đã ghi → tránh hỏng JPEG).
- Phía PC: `TCP_NODELAY`, `SO_KEEPALIVE`, `SO_RCVBUF = 512 KB`, timeout chờ video
  15 s.

Nếu sóng quá yếu để kịp frame lớn, dùng preset **Advanced** (640×360) để giảm
dung lượng frame. TCP không thể tạo thêm băng thông — giảm bitrate là cách duy
nhất để frame vẫn đi đều.

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

Buffer của driver (`cam_hal.c`: `recv_size = jpeg_buffer_size` nếu khác 0, ngược
lại `w*h/5` cho JPEG) được cấp phát một lần lúc `esp_camera_init()`. Firmware đặt
`jpeg_buffer_size = 2048×1536/5 ≈ 629 KB` cho frame buffer duy nhất
(`fb_count=1`), nên khung tuỳ chỉnh lớn nhất không bị driver cắt dù
`frame_size` lúc init chỉ là `HD 1280×720` (buffer mặc định chỉ ~184 KB). Sau đó
`camera_apply_cfg()` mới đặt kích thước thật (mặc định XGA 1024×768). Nếu lần
init với buffer tối đa thất bại (thiếu PSRAM), firmware tự thử lại với buffer mặc
định của XGA.

Ở khung lớn nhất (2048×1536), nên giữ **JPEG quality ≥ 10** (mặc định). Đặt quá
thấp (0–5) làm file JPEG vượt ~629 KB, có thể bị driver cắt/bỏ khung; hạ độ phân
giải hoặc tăng số chất lượng sẽ ổn định hơn.

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
