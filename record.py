"""
ESP32-CAM client and control panel (Tkinter).

Two-channel TCP architecture:
  - Port 1357: ESP32 -> PC, binary frames [uint32 len][JPEG]
  - Port 1358: PC -> ESP32, sends "CFG key=value ..." commands and receives OK/ERR

Run:  python record.py
Exit: press 'q' in the video window or close the control panel.
"""

import os
import queue
import socket
import struct
import threading
import time
import tkinter as tk
from tkinter import ttk

import cv2
import numpy as np

# --------------------------------------------------------------------------
# Cấu hình mạng (phải khớp với include/esp_network.h)
# --------------------------------------------------------------------------
SERVER_IP = "0.0.0.0"
SERVER_PORT = 1357            # video
CONTROL_PORT = 1358           # điều khiển thông số
UDP_BROADCAST_PORT = 5005     # khám phá server
AUTH_PASSWORD = "pass187631"

# Trần kích thước 1 frame hợp lệ. Firmware cấp buffer JPEG ~629 KB/frame
# (OV3660_JPEG_BUF_MAX) nên frame thật không vượt ngưỡng này; giá trị lớn hơn
# chỉ để chặn header hỏng/gói rác làm client cấp phát vô hạn.
MAX_FRAME_SIZE = 1024 * 1024
VIDEO_WINDOW = "ESP32-CAM Stream"

# ---- Ghi video MP4 ----
# Thư mục lưu file. Sửa trực tiếp hằng số này, hoặc đặt biến môi trường
# ESP32CAM_RECORD_DIR để ghi đè mà không cần sửa code.
RECORD_DIR = r"D:\Book"
RECORD_DIR = os.environ.get("ESP32CAM_RECORD_DIR", RECORD_DIR)
# Codec thử lần lượt (máy nào thiếu cái trước thì dùng cái sau).
RECORD_CODECS = ("mp4v", "avc1", "H264")
# Chia video thành nhiều file, mỗi file ~RECORD_SEGMENT_SECONDS giây, để lỗi/hỏng
# một file không mất toàn bộ bản ghi. Đặt 0 để tắt chia đoạn.
RECORD_SEGMENT_SECONDS = 300      # 5 phút

FILTER_MODES = ["none", "gray", "blur", "sharp", "vintage"]

# framesize_t của esp32-camera
FRAME_SIZES = [
    ("96x96", 0), ("160x120", 1), ("128x128", 2), ("176x144", 3),
    ("240x176", 4), ("240x240", 5), ("320x240", 6), ("320x320", 7),
    ("400x296", 8), ("480x320", 9), ("640x480 VGA", 10), ("800x600 SVGA", 11),
    ("1024x768 XGA", 12), ("1280x720 HD", 13), ("1280x1024 SXGA", 14),
    ("1600x1200 UXGA", 15), ("1920x1080 FHD", 16),
]

WB_MODES = [("Auto", 0), ("Sunny", 1), ("Cloudy", 2), ("Office", 3), ("Incandescent", 4)]
EFFECTS = [("None", 0), ("Negative", 1), ("Grayscale", 2), ("Red tint", 3),
           ("Green tint", 4), ("Blue tint", 5), ("Sepia", 6)]

# name, nhãn hiển thị, min, max, mặc định
# Mặc định ưu tiên giảm nhiễu trong ánh sáng bình thường; các giá trị phải
# khớp camera_config_defaults() trong firmware.
SLIDERS = [
    ("brightness", "Brightness", -2, 2, 0),
    ("contrast", "Contrast", -2, 2, 0),
    ("saturation", "Saturation", -2, 2, 1),
    ("sharpness", "Sharpness", -2, 2, 0),
    ("denoise", "Denoise (0-8)", 0, 8, 5),
    ("ae_level", "AE compensation", -2, 2, 0),
    ("gainceiling", "Gain ceiling (0-6)", 0, 6, 0),
    ("quality", "JPEG quality (0 = best)", 0, 63, 12),
    ("aec_value", "Manual exposure", 0, 1200, 400),
    ("agc_gain", "Manual gain", 0, 30, 0),
    ("fps", "FPS (stream + max exposure)", 1, 30, 5),
]

DEFAULTS = {name: default for name, _, _, _, default in SLIDERS}
DEFAULTS.update({
    "framesize": 12, "wb_mode": 0, "effect": 0,
    "hmirror": 0, "vflip": 0, "aec": 1, "agc": 1,
    "bpc": 1, "wpc": 1, "lenc": 1, "raw_gma": 1,
    # kích thước khung hình tuỳ chỉnh (bỏ qua framesize khi custom = 1)
    "custom": 0, "width": 1024, "height": 768,
})

# Mỗi mode đặt lại toàn bộ thông số ảnh dùng chung để không sót giá trị từ mode
# trước hoặc từ thao tác tay. Độ phân giải, lật, WB và hiệu ứng vẫn giữ theo lựa
# chọn của người dùng.
MODE_BASE_PRESET = {
    **{name: DEFAULTS[name] for name, *_ in SLIDERS},
    **{name: DEFAULTS[name] for name in
       ("aec", "agc", "bpc", "wpc", "lenc", "raw_gma")},
}

# Dùng trong cảnh tối; gain cao hơn mode ban ngày nên ảnh có thể nhiễu hơn.
LOW_LIGHT_PRESET = {
    **MODE_BASE_PRESET,
    "brightness": 1, "contrast": 1, "sharpness": 2,
    "denoise": 4, "ae_level": 2, "gainceiling": 2,
    "aec": 1, "agc": 1,
}
# Ưu tiên chi tiết mặt/chuyển động; gain cao và ít khử nhiễu nên ảnh hạt hơn.
FACE_PRESET = {
    **MODE_BASE_PRESET,
    "brightness": 1, "contrast": 1, "sharpness": 2,
    "denoise": 1, "ae_level": 2, "gainceiling": 5,
    "aec": 1, "agc": 1,
}
DAYLIGHT_PRESET = {
    **MODE_BASE_PRESET,
    # XGA ở PCLK 10 MHz đạt tối đa ~5,5 fps; đặt cao hơn không nhanh thêm.
    "aec": 1, "agc": 1,
}
# Lắp trong tường, nhìn qua khe ngang: crop 16:9 và giảm bitrate để Wi-Fi yếu
# vẫn truyền được. Gain 4x giữ sáng vừa phải mà không đẩy nhiễu quá cao.
WALL_SLIT_PRESET = {
    **MODE_BASE_PRESET,
    "custom": 1, "width": 640, "height": 360,
    "quality": 20, "denoise": 4, "gainceiling": 1, "fps": 5,
    "aec": 1, "agc": 1,
}

# Giới hạn kích thước tuỳ chỉnh = vùng cảm biến OV3660 (phải khớp firmware)
CUSTOM_W_MIN, CUSTOM_W_MAX = 96, 2048
CUSTOM_H_MIN, CUSTOM_H_MAX = 96, 1536

# Miền giá trị hợp lệ của MỌI tham số gửi xuống ESP32.
# Phải khớp bảng s_cfg_fields trong src/esp_network.c: client kẹp lại lần nữa
# để một lỗi UI (ô nhập trống, số lạ...) không bao giờ gửi giá trị sai.
SEND_LIMITS = {
    "brightness": (-2, 2), "contrast": (-2, 2), "saturation": (-2, 2),
    "sharpness": (-2, 2), "ae_level": (-2, 2), "denoise": (0, 8),
    "quality": (0, 63), "framesize": (0, 16), "gainceiling": (0, 6),
    "wb_mode": (0, 4), "effect": (0, 6), "hmirror": (0, 1), "vflip": (0, 1),
    "aec": (0, 1), "agc": (0, 1), "aec_value": (0, 1200), "agc_gain": (0, 30),
    "bpc": (0, 1), "wpc": (0, 1), "lenc": (0, 1), "raw_gma": (0, 1),
    "fps": (1, 30), "custom": (0, 1),
    "width": (CUSTOM_W_MIN, CUSTOM_W_MAX),
    "height": (CUSTOM_H_MIN, CUSTOM_H_MAX),
}

CTRL_LINE_MAX = 500        # CTRL_RX_MAX bên firmware là 512
CTRL_IDLE_PING = 5.0       # giây không nhận gì -> gửi PING
CTRL_DEAD_AFTER = 20.0     # giây không nhận gì -> coi như kênh đã chết
VIDEO_STALL_TIMEOUT = 15.0 # giây không có byte nào -> ngắt để ESP32 nối lại
FRAME_QUEUE_MAX = 3

# Gửi cấu hình: gom thay đổi trong SEND_DEBOUNCE_MS, và 2 lần gửi liên tiếp
# phải cách nhau ít nhất SEND_MIN_INTERVAL giây để không dội lệnh CFG xuống ESP32
# khi người dùng kéo thanh trượt liên tục.
SEND_DEBOUNCE_MS = 150
SEND_MIN_INTERVAL = 1.0


# --------------------------------------------------------------------------
# Xử lý ảnh phía client
# --------------------------------------------------------------------------
def apply_filter(frame, mode):
    if mode == "gray":
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    if mode == "blur":
        return cv2.GaussianBlur(frame, (5, 5), 0)
    if mode == "sharp":
        kernel = np.array([[0, -1, 0], [-1, 5, -1], [0, -1, 0]])
        return cv2.filter2D(frame, -1, kernel)
    if mode == "vintage":
        kernel = np.array([[0.272, 0.534, 0.131],
                           [0.349, 0.686, 0.168],
                           [0.393, 0.769, 0.189]])
        out = cv2.transform(frame, kernel)
        return np.clip(out, 0, 255).astype(np.uint8)
    return frame


# --------------------------------------------------------------------------
# Tầng mạng: UDP discovery + TCP video + TCP control
# --------------------------------------------------------------------------
class CameraLink:
    def __init__(self, status_cb):
        self.frames = queue.Queue(maxsize=FRAME_QUEUE_MAX)
        self.status_cb = status_cb        # callback(text) cập nhật thanh trạng thái
        self.ctrl_sock = None
        self.ctrl_lock = threading.Lock()
        self.video_connected = False
        self.ctrl_connected = False
        self.need_sync = False        # True khi cần đẩy lại toàn bộ cấu hình
        self.on_ctrl_line = None      # callback(text) cho mỗi dòng ESP32 trả về
        self.last_ctrl_rx = 0.0       # thời điểm nhận dữ liệu cuối trên kênh 1358
        self._ping_sent = 0.0

    # -- tiện ích --
    def _status(self, text):
        if self.status_cb:
            try:
                self.status_cb(text)
            except Exception:       # callback lỗi không được làm chết thread mạng
                pass

    def ctrl_idle_seconds(self):
        return time.time() - self.last_ctrl_rx if self.last_ctrl_rx else 1e9

    def ctrl_abort(self):
        """Đóng kênh điều khiển khi nghi đã chết (ESP32 mất điện / mất WiFi)."""
        with self.ctrl_lock:
            sock, self.ctrl_sock = self.ctrl_sock, None
        self.ctrl_connected = False
        if sock is not None:
            try:
                sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            try:
                sock.close()
            except OSError:
                pass

    @staticmethod
    def _get_local_ip():
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(("8.8.8.8", 80))
            ip = s.getsockname()[0]
            s.close()
            return ip
        except OSError:
            return "127.0.0.1"

    @staticmethod
    def _recv_exact(conn, n):
        buf = bytearray()
        while len(buf) < n:
            chunk = conn.recv(min(65536, n - len(buf)))
            if not chunk:
                return None
            buf += chunk
        return bytes(buf)

    def start(self):
        threading.Thread(target=self._udp_discovery, daemon=True).start()
        threading.Thread(target=self._video_server, daemon=True).start()
        threading.Thread(target=self._ctrl_server, daemon=True).start()

    # -- 1. UDP discovery: trả lời "KEY-IP <ip>" cho ESP32 --
    def _udp_discovery(self):
        local_ip = self._get_local_ip()
        while True:
            sock = None
            try:
                sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                sock.bind(("0.0.0.0", UDP_BROADCAST_PORT))
                sock.settimeout(5.0)
                self._status(f"UDP discovery :{UDP_BROADCAST_PORT} — IP server {local_ip}")

                while True:
                    try:
                        data, addr = sock.recvfrom(1024)
                    except socket.timeout:
                        continue
                    except OSError as exc:
                        self._status(f"UDP discovery error: {exc}")
                        time.sleep(1.0)
                        break
                    if data.decode("utf-8", errors="ignore") == "DISCOVER_SERVER":
                        try:
                            sock.sendto(f"KEY-IP {local_ip}".encode(), addr)
                        except OSError:
                            continue
                        self._status(f"ESP32 {addr[0]} requested IP; replied with {local_ip}")
            except OSError as exc:
                self._status(f"Cannot open UDP port {UDP_BROADCAST_PORT}: {exc} "
                             f"— retrying in 5 s")
                time.sleep(5.0)
            finally:
                if sock is not None:
                    try:
                        sock.close()
                    except OSError:
                        pass

    # -- 2. TCP video --
    def _video_server(self):
        while True:
            srv = None
            try:
                srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                srv.bind((SERVER_IP, SERVER_PORT))
                srv.listen(1)
                self._status(f"Waiting for video on port {SERVER_PORT}...")
            except OSError as exc:
                self._status(f"Cannot open video port {SERVER_PORT}: {exc} "
                             f"(is another program using it?) — retrying in 5 s")
                if srv is not None:
                    srv.close()
                time.sleep(5.0)
                continue

            while True:
                try:
                    conn, addr = srv.accept()
                except OSError as exc:
                    self._status(f"Video accept error: {exc}")
                    break
                threading.Thread(target=self._video_session, args=(conn, addr),
                                 daemon=True).start()

            srv.close()
            time.sleep(2.0)

    def _video_session(self, conn, addr):
        try:
            # Tối ưu socket nhận cho WiFi yếu:
            #  - TCP_NODELAY: ESP32 đẩy frame ngay, không chờ gom gói.
            #  - SO_KEEPALIVE: phát hiện peer chết.
            #  - SO_RCVBUF lớn: đủ chỗ đệm 1 frame JPEG vài trăm KB khi link chậm.
            try:
                conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                conn.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
                conn.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 512 * 1024)
            except OSError:
                pass

            # Nếu ESP32 mất điện giữa khung hình, recv sẽ treo vô hạn nếu không
            # có timeout -> đặt timeout để tự ngắt và chờ kết nối lại.
            conn.settimeout(VIDEO_STALL_TIMEOUT)

            # Bắt tay: gửi mật khẩu, chờ AUTH_OK
            conn.sendall((AUTH_PASSWORD + "\n").encode())
            auth = b""
            while b"\n" not in auth:
                chunk = conn.recv(64)
                if not chunk:
                    return
                auth += chunk
                if len(auth) > 64:
                    break

            if b"AUTH_OK" not in auth:
                self._status(f"Authentication failed: {auth!r}")
                return

            self.video_connected = True
            self._status(f"Video connected from {addr[0]}")

            while True:
                header = self._recv_exact(conn, 4)
                if header is None:
                    break
                size = struct.unpack("<I", header)[0]
                if size == 0 or size > MAX_FRAME_SIZE:
                    self._status(f"Invalid frame size ({size}); disconnecting")
                    break
                payload = self._recv_exact(conn, size)
                if payload is None:
                    break

                try:
                    self.frames.put_nowait(payload)
                except queue.Full:
                    # Bỏ frame cũ nhất rồi thử lại; vẫn đầy thì bỏ frame mới
                    try:
                        self.frames.get_nowait()
                        self.frames.put_nowait(payload)
                    except (queue.Empty, queue.Full):
                        pass
        except socket.timeout:
            self._status(f"No video data for {VIDEO_STALL_TIMEOUT:.0f} s — "
                         f"disconnecting so ESP32 can reconnect")
        except OSError as exc:
            self._status(f"Video session error: {exc}")
        except Exception as exc:        # lỗi lạ không được làm chết tiến trình
            self._status(f"Unknown video session error: {exc}")
        finally:
            self.video_connected = False
            try:
                conn.close()
            except OSError:
                pass
            self._status("Video disconnected; waiting for ESP32 to reconnect...")

    # -- 3. TCP control --
    def _ctrl_server(self):
        while True:
            srv = None
            try:
                srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                srv.bind((SERVER_IP, CONTROL_PORT))
                srv.listen(1)
            except OSError as exc:
                self._status(f"Cannot open control port {CONTROL_PORT}: {exc} "
                             f"(is another program using it?) — retrying in 5 s")
                if srv is not None:
                    srv.close()
                time.sleep(5.0)
                continue

            while True:
                try:
                    conn, addr = srv.accept()
                except OSError as exc:
                    self._status(f"Control connection accept error: {exc}")
                    break
                threading.Thread(target=self._ctrl_session, args=(conn, addr),
                                 daemon=True).start()

            srv.close()
            time.sleep(2.0)

    def _ctrl_session(self, conn, addr):
        with self.ctrl_lock:
            self.ctrl_sock = conn
        self.ctrl_connected = True
        self.last_ctrl_rx = time.time()
        self.need_sync = True     # ESP32 vừa (kết nối lại) -> đồng bộ toàn bộ tham số
        self._status(f"Control channel connected from {addr[0]}:{CONTROL_PORT}")

        # Không đặt timeout cứng: kênh này im lặng khi không ai chỉnh gì. Việc
        # phát hiện "đã chết" do watchdog PING ở vòng UI đảm nhiệm.
        conn.settimeout(None)
        buf = b""
        try:
            while True:
                chunk = conn.recv(256)
                if not chunk:
                    break
                self.last_ctrl_rx = time.time()
                buf += chunk
                if len(buf) > 4096:          # dòng rác quá dài -> bỏ
                    buf = b""
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    text = line.decode("utf-8", errors="ignore").strip()
                    if text:
                        if self.on_ctrl_line:
                            try:
                                self.on_ctrl_line(text)
                            except Exception:
                                pass
        except OSError as exc:
            self._status(f"Control channel error: {exc}")
        finally:
            with self.ctrl_lock:
                if self.ctrl_sock is conn:
                    self.ctrl_sock = None
            self.ctrl_connected = False
            try:
                conn.close()
            except OSError:
                pass
            self._status("Control channel closed")

    # -- API cho UI --
    @staticmethod
    def _clamp_params(params):
        """Kẹp mọi tham số vào miền hợp lệ trước khi gửi (chống lỗi UI)."""
        safe = {}
        for key, value in params.items():
            limits = SEND_LIMITS.get(key)
            if limits is None:
                continue
            try:
                ivalue = int(value)
            except (TypeError, ValueError):
                continue
            safe[key] = max(limits[0], min(limits[1], ivalue))
        return safe

    def send_params(self, params):
        safe = self._clamp_params(params)
        if not safe:
            return False
        line = "CFG " + " ".join(f"{k}={v}" for k, v in safe.items()) + "\n"
        if len(line) > CTRL_LINE_MAX:
            self._status("Configuration is too long; not sent (internal error)")
            return False

        with self.ctrl_lock:
            if self.ctrl_sock is None:
                return False
            try:
                self.ctrl_sock.sendall(line.encode())
                self.last_ctrl_rx = time.time()
                return True
            except OSError as exc:
                self._status(f"Command send failed: {exc}")
                return False

    def send_raw(self, text):
        with self.ctrl_lock:
            if self.ctrl_sock is None:
                return False
            try:
                self.ctrl_sock.sendall((text.strip() + "\n").encode())
                self.last_ctrl_rx = time.time()
                return True
            except OSError as exc:
                self._status(f"Command send failed: {exc}")
                return False

    def ctrl_watchdog(self):
        """Gửi PING khi kênh im lặng; cắt kênh nếu mất phản hồi quá lâu.
        Gọi từ main thread (vòng _pump)."""
        if not self.ctrl_connected:
            return
        idle = self.ctrl_idle_seconds()
        if idle >= CTRL_DEAD_AFTER:
            self._status("Control channel is unresponsive — closing it so ESP32 can reconnect")
            self.ctrl_abort()
        elif idle >= CTRL_IDLE_PING and time.time() - self._ping_sent >= CTRL_IDLE_PING:
            self._ping_sent = time.time()
            self.send_raw("PING")


# --------------------------------------------------------------------------
# Giao diện Tkinter
# --------------------------------------------------------------------------
class ControlPanel:
    def __init__(self, root, link):
        self.root = root
        self.link = link
        self.vars = {}
        self.combos = {}
        self.checks = {}
        self.scales = {}
        self.value_labels = {}
        self.status_queue = queue.Queue()
        self.log_queue = queue.Queue()
        self.last_frame = None
        self.filter_mode = "sharp"
        self.frame_count = 0
        self.fps_measured = 0.0
        self._fps_t0 = time.time()
        self._send_job = None
        self._last_send = 0.0     # time.monotonic() của lần gửi CFG gần nhất
        self._building = True     # đang dựng UI: chưa gửi gì
        self._closing = False
        self._video_window_open = False

        # trạng thái ghi video
        self.record_writer = None
        self.record_size = None       # (w, h) của file đang ghi
        self.record_fps = 0.0
        self.record_path = ""
        self.record_frames = 0
        self.record_t0 = 0.0
        self._size_warned = False
        self.record_part = 1          # số thứ tự đoạn trong 1 lần bấm ghi
        self.record_seg_frames = 0    # frame đã ghi vào đoạn hiện tại
        self.record_seg_limit = 0     # số frame tối đa mỗi đoạn
        self.record_base = ""         # tên gốc dùng chung cho mọi đoạn

        self.status_var = tk.StringVar(value="Starting...")

        self._build_ui()
        self._waiting_frame = np.zeros((360, 640, 3), dtype=np.uint8)
        self._building = False    # từ đây mọi thay đổi mới được gửi đi
        self.link.status_cb = self._set_status_async
        self.link.on_ctrl_line = self.log_queue.put
        self.link.start()
        self.root.after(30, self._pump)

    # ---------- dựng giao diện ----------
    def _build_ui(self):
        self.root.title("ESP32-CAM — OV3660 Control Panel")
        self.root.geometry("1180x760")

        main = ttk.Frame(self.root, padding=8)
        main.pack(fill="both", expand=True)

        # ---- bên trái: thông tin video ----
        left = ttk.Frame(main)
        left.pack(side="left", fill="both", expand=True)

        box = ttk.LabelFrame(left, text="Video (OpenCV window)", padding=8)
        box.pack(fill="both", expand=True)
        self.info = tk.Text(box, height=12, width=44, state="disabled",
                            font=("Consolas", 9))
        self.info.pack(fill="both", expand=True)
        ttk.Label(left, text="Press 'q' in the video window to exit.",
                  foreground="#555").pack(anchor="w", pady=(6, 0))

        # ---- bên phải: bảng điều khiển ----
        right = ttk.Frame(main, width=460)
        right.pack(side="right", fill="y", padx=(10, 0))
        right.pack_propagate(False)

        ttk.Label(right, text="Status:").pack(anchor="w", pady=(0, 2))
        ttk.Label(right, textvariable=self.status_var, wraplength=410,
                  foreground="#0a5").pack(anchor="w", pady=(0, 6))
        ttk.Separator(right).pack(fill="x", pady=(0, 6))

        notebook = ttk.Notebook(right)
        notebook.pack(fill="both", expand=True)

        self._build_image_tab(notebook)
        self._build_sensor_tab(notebook)

        # ---- hàng nút ----
        buttons = ttk.Frame(right)
        buttons.pack(fill="x", pady=(8, 2))

        # Mọi thay đổi được gửi tự động (~150 ms) nên nút này chỉ là "gửi lại
        # toàn bộ" khi cần (ví dụ sau khi ESP32 vừa khởi động lại).
        ttk.Button(buttons, text="Show video",
                   command=self._open_video_window).pack(side="left")
        ttk.Button(buttons, text="Send", command=self.apply).pack(side="left")
        ttk.Button(buttons, text="Defaults", command=self.reset_defaults).pack(side="left", padx=4)
        ttk.Button(buttons, text="Read status", command=self.read_stats).pack(side="left", padx=4)
        self.record_btn = ttk.Button(buttons, text="● Record MP4",
                                     command=self.toggle_record)
        self.record_btn.pack(side="left", padx=(4, 0))

        ttk.Label(right, text="Changes are sent automatically — no button needed "
                              "(up to once per second).",
                  foreground="#666", wraplength=410).pack(anchor="w", pady=(0, 2))

        ttk.Button(right, text="Restart ESP32",
                   command=self.restart_esp).pack(fill="x", pady=(2, 4))

    def _build_image_tab(self, notebook):
        tab = ttk.Frame(notebook, padding=8)
        notebook.add(tab, text="Image")

        # Quick presets: change the full group of settings with one click.
        row = ttk.Frame(tab)
        row.pack(fill="x", pady=(0, 6))
        ttk.Label(row, text="Mode", width=10).pack(side="left")
        ttk.Button(row, text="Low light",
                   command=lambda: self.apply_preset(LOW_LIGHT_PRESET,
                                                    "Low light")).pack(side="left")
        ttk.Button(row, text="Face detail",
                   command=lambda: self.apply_preset(FACE_PRESET,
                                                    "Face detail")).pack(side="left", padx=4)
        ttk.Button(row, text="Daylight",
                   command=lambda: self.apply_preset(DAYLIGHT_PRESET,
                                                    "Daylight")).pack(side="left")
        ttk.Button(row, text="Advanced",
                   command=lambda: self.apply_preset(WALL_SLIT_PRESET,
                                                    "Wall / horizontal slot")).pack(side="left", padx=4)

        for name, label, lo, hi, default in SLIDERS:
            row = ttk.Frame(tab)
            row.pack(fill="x", pady=2)
            ttk.Label(row, text=label, width=27).pack(side="left")
            var = tk.IntVar(value=default)
            self.vars[name] = var
            value_lbl = ttk.Label(row, text=str(default), width=5, anchor="e")
            value_lbl.pack(side="right")
            self.value_labels[name] = value_lbl

            def on_move(val, v=var, lbl=value_lbl):
                v.set(int(round(float(val))))
                lbl.config(text=str(v.get()))
                self._schedule_send()

            scale = ttk.Scale(row, from_=lo, to=hi, orient="horizontal",
                              command=on_move)
            scale.set(default)
            scale.pack(side="left", fill="x", expand=True, padx=6)
            # Lưới an toàn: thả chuột / nhả phím cũng gửi ngay, kể cả khi nền
            # tảng không phát `command` cho lần thay đổi cuối.
            scale.bind("<ButtonRelease-1>", lambda _e: self._schedule_send())
            scale.bind("<KeyRelease>", lambda _e: self._schedule_send())
            self.scales[name] = scale

        # Standard resolution.
        row = ttk.Frame(tab)
        row.pack(fill="x", pady=(10, 2))
        ttk.Label(row, text="Resolution", width=27).pack(side="left")
        self.combos["framesize"] = ttk.Combobox(
            row, state="readonly", width=20,
            values=[f"{n}  (#{i})" for n, i in FRAME_SIZES])
        self.combos["framesize"].current(12)
        self.combos["framesize"].pack(side="left")
        self.combos["framesize"].bind("<<ComboboxSelected>>", lambda _e: self._schedule_send())

        # Custom size: when enabled, the Resolution field above is ignored.
        row = ttk.Frame(tab)
        row.pack(fill="x", pady=(8, 2))
        self.custom_var = tk.IntVar(value=DEFAULTS["custom"])
        ttk.Checkbutton(row, text="Custom size (W × H)",
                        variable=self.custom_var,
                        command=self._on_custom_toggle).pack(side="left")

        row = ttk.Frame(tab)
        row.pack(fill="x", pady=2)
        ttk.Label(row, text="Width × Height", width=27).pack(side="left")
        self.width_var = tk.IntVar(value=DEFAULTS["width"])
        self.height_var = tk.IntVar(value=DEFAULTS["height"])
        self.width_spin = ttk.Spinbox(row, from_=CUSTOM_W_MIN, to=CUSTOM_W_MAX,
                                      increment=2, width=6,
                                      textvariable=self.width_var,
                                      command=self._schedule_send)
        self.width_spin.pack(side="left")
        ttk.Label(row, text=" × ").pack(side="left")
        self.height_spin = ttk.Spinbox(row, from_=CUSTOM_H_MIN, to=CUSTOM_H_MAX,
                                       increment=2, width=6,
                                       textvariable=self.height_var,
                                       command=self._schedule_send)
        self.height_spin.pack(side="left")
        ttk.Label(row, text="  pixels", foreground="#666").pack(side="left")

        # gõ tay số rồi Enter / rời ô cũng gửi đi
        for spin in (self.width_spin, self.height_spin):
            spin.bind("<Return>", lambda _e: self._schedule_send())
            spin.bind("<FocusOut>", lambda _e: self._schedule_send())

        ttk.Label(tab, justify="left", wraplength=390, foreground="#666",
                  text="Dimensions must be even, up to 2048 × 1536 (OV3660 sensor "
                       "area). The full sensor frame is scaled to preserve the "
                       "wide field of view; the window is centered. Check the "
                       "log (STAT res=...) for the actual size.").pack(anchor="w", pady=(2, 0))

        self._update_res_state()

        # Client-side display filter.
        row = ttk.Frame(tab)
        row.pack(fill="x", pady=2)
        ttk.Label(row, text="Display filter (PC)", width=27).pack(side="left")
        self.filter_combo = ttk.Combobox(row, state="readonly", width=20,
                                         values=FILTER_MODES)
        self.filter_combo.set(self.filter_mode)
        self.filter_combo.pack(side="left")
        self.filter_combo.bind("<<ComboboxSelected>>",
                               lambda _e: setattr(self, "filter_mode", self.filter_combo.get()))

    def _build_sensor_tab(self, notebook):
        tab = ttk.Frame(notebook, padding=8)
        notebook.add(tab, text="OV3660 sensor")

        # Flip / mirror.
        for name, label in (("hmirror", "Horizontal mirror (0x3821)"),
                            ("vflip", "Vertical flip (0x3820)")):
            var = tk.IntVar(value=DEFAULTS[name])
            self.checks[name] = var
            ttk.Checkbutton(tab, text=label, variable=var,
                            command=self._schedule_send).pack(anchor="w", pady=2)

        # Auto exposure / gain.
        ttk.Separator(tab).pack(fill="x", pady=6)
        self.checks["aec"] = tk.IntVar(value=DEFAULTS["aec"])
        ttk.Checkbutton(tab, text="Auto Exposure (AEC)", variable=self.checks["aec"],
                        command=self._schedule_send).pack(anchor="w", pady=2)
        self.checks["agc"] = tk.IntVar(value=DEFAULTS["agc"])
        ttk.Checkbutton(tab, text="Auto Gain (AGC)", variable=self.checks["agc"],
                        command=self._schedule_send).pack(anchor="w", pady=2)

        # Pixel correction.
        ttk.Separator(tab).pack(fill="x", pady=6)
        for name, label in (("bpc", "BPC — black pixel correction"),
                            ("wpc", "WPC — white pixel correction"),
                            ("lenc", "LENC — lens shading correction"),
                            ("raw_gma", "Gamma RAW")):
            var = tk.IntVar(value=DEFAULTS[name])
            self.checks[name] = var
            ttk.Checkbutton(tab, text=label, variable=var,
                            command=self._schedule_send).pack(anchor="w", pady=2)

        # White balance + effects.
        ttk.Separator(tab).pack(fill="x", pady=6)
        row = ttk.Frame(tab)
        row.pack(fill="x", pady=2)
        ttk.Label(row, text="White balance", width=18).pack(side="left")
        self.combos["wb_mode"] = ttk.Combobox(row, state="readonly", width=18,
                                              values=[f"{n}" for n, _ in WB_MODES])
        self.combos["wb_mode"].current(0)
        self.combos["wb_mode"].pack(side="left")
        self.combos["wb_mode"].bind("<<ComboboxSelected>>", lambda _e: self._schedule_send())

        row = ttk.Frame(tab)
        row.pack(fill="x", pady=2)
        ttk.Label(row, text="Color effect", width=18).pack(side="left")
        self.combos["effect"] = ttk.Combobox(row, state="readonly", width=18,
                                             values=[f"{n}" for n, _ in EFFECTS])
        self.combos["effect"].current(0)
        self.combos["effect"].pack(side="left")
        self.combos["effect"].bind("<<ComboboxSelected>>", lambda _e: self._schedule_send())

        # Gửi tự động: bất kỳ thay đổi nào của checkbox (kể cả do code đặt giá
        # trị, không chỉ khi bấm chuột) đều lên lịch gửi cấu hình.
        for var in self.checks.values():
            var.trace_add("write", lambda *_a: self._schedule_send())

        ttk.Label(tab, justify="left", wraplength=390, foreground="#666",
                  text="Note: These settings write directly to OV3660 registers "
                       "(0x3503, 0x3500/01/02, 0x350A/0B, 0x3820/21, 0x4407...). "
                       "Changes are applied by the capture task, so the video "
                       "may skip 1–2 frames.").pack(anchor="w", pady=(12, 0))

    # ---------- thu thập / gửi tham số ----------
    @staticmethod
    def _clamp_even(var, default, lo, hi):
        """Đọc spinbox -> số nguyên chẵn trong [lo, hi]; gõ sai thì lấy mặc định."""
        try:
            value = int(float(var.get()))
        except (tk.TclError, ValueError):
            return default
        value = max(lo, min(hi, value))
        return value - (value % 2)

    def _update_res_state(self):
        """Bật/tắt ô nhập theo checkbox: chỉ một trong hai kiểu kích thước có hiệu lực."""
        custom = bool(self.custom_var.get())
        self.combos["framesize"].config(state="disabled" if custom else "readonly")
        self.width_spin.config(state="normal" if custom else "disabled")
        self.height_spin.config(state="normal" if custom else "disabled")

    def _on_custom_toggle(self):
        self._update_res_state()
        self._schedule_send()

    def _collect(self):
        params = {name: var.get() for name, var in self.vars.items()}
        fs = self.combos["framesize"].current()
        params["framesize"] = FRAME_SIZES[fs][1] if fs >= 0 else DEFAULTS["framesize"]
        params["wb_mode"] = self.combos["wb_mode"].current()
        params["effect"] = self.combos["effect"].current()
        for name, var in self.checks.items():
            params[name] = int(var.get())

        # kích thước tuỳ chỉnh: chuẩn hoá về số chẵn và ghi lại để UI khớp firmware
        width = self._clamp_even(self.width_var, DEFAULTS["width"],
                                 CUSTOM_W_MIN, CUSTOM_W_MAX)
        height = self._clamp_even(self.height_var, DEFAULTS["height"],
                                  CUSTOM_H_MIN, CUSTOM_H_MAX)
        self.width_var.set(width)
        self.height_var.set(height)
        params["custom"] = 1 if self.custom_var.get() else 0
        params["width"] = width
        params["height"] = height
        return params

    def _schedule_send(self):
        if self._building or self._closing:
            return
        if self._send_job is not None:
            self.root.after_cancel(self._send_job)
        self._send_job = self.root.after(SEND_DEBOUNCE_MS, self.apply)

    def apply(self):
        """Gửi cấu hình xuống ESP32.

        Hai lần gửi liên tiếp luôn cách nhau >= SEND_MIN_INTERVAL giây: nếu chưa
        đủ thời gian giãn cách thì hẹn lại, và mọi thay đổi trong lúc chờ được
        gộp vào đúng một lần gửi (lấy trạng thái cuối của UI).
        """
        # Huỷ lệnh gửi đang chờ (nếu có) để một lần đổi không gửi 2 lần.
        job, self._send_job = self._send_job, None
        if job is not None:
            try:
                self.root.after_cancel(job)
            except tk.TclError:
                pass
        if self._closing:
            return

        now = time.monotonic()
        wait = self._last_send + SEND_MIN_INTERVAL - now
        if wait > 0:
            try:
                self._send_job = self.root.after(int(wait * 1000) + 1, self.apply)
            except tk.TclError:
                self._send_job = None
            return

        self._last_send = now
        try:
            params = self._collect()
        except Exception as exc:        # ô nhập rác không được làm sập UI
            self._set_status(f"Invalid value: {exc}")
            return
        if not self.link.send_params(params):
            self._set_status("Control channel is not connected — parameters were not sent")
            return
        self._set_status("Configuration sent: " +
                         " ".join(f"{k}={v}" for k, v in list(params.items())[:4]) + " ...")

    def apply_preset(self, preset, name):
        """Áp nhanh một nhóm thông số (ánh sáng hoặc băng thông) rồi gửi xuống ESP32."""
        for key, value in preset.items():
            if key == "framesize":
                for idx, (_, fs) in enumerate(FRAME_SIZES):
                    if fs == value:
                        self.combos["framesize"].current(idx)
                        break
                # độ phân giải chuẩn có hiệu lực -> tắt kích thước tuỳ chỉnh
                self.custom_var.set(0)
                self._update_res_state()
                continue
            if key == "custom":
                self.custom_var.set(value)
                self._update_res_state()
                continue
            if key in ("width", "height"):
                var = self.width_var if key == "width" else self.height_var
                var.set(value)
                continue
            var = self.vars.get(key)
            if var is not None:
                var.set(value)
                self.scales[key].set(value)
                self.value_labels[key].config(text=str(value))
                continue
            check = self.checks.get(key)
            if check is not None:
                check.set(value)
        self._schedule_send()
        self._set_status(f"Selected {name} mode; updated settings will be sent")

    def reset_defaults(self):
        for name, _, _, _, default in SLIDERS:
            self.vars[name].set(default)
            self.scales[name].set(default)
            self.value_labels[name].config(text=str(default))
        self.combos["framesize"].current(12)
        self.combos["wb_mode"].current(0)
        self.combos["effect"].current(0)
        for name, var in self.checks.items():
            var.set(DEFAULTS[name])
        self.custom_var.set(DEFAULTS["custom"])
        self.width_var.set(DEFAULTS["width"])
        self.height_var.set(DEFAULTS["height"])
        self._update_res_state()
        self.apply()

    def restart_esp(self):
        if not self.link.send_raw("RESTART"):
            self._set_status("Control channel is not connected")
            return
        self._set_status("ESP32 restart requested")

    def read_stats(self):
        if not self.link.send_raw("STAT"):
            self._set_status("Control channel is not connected")
            return
        self._set_status("Requested ESP32 status...")

    # ---------- ghi video MP4 ----------
    def toggle_record(self):
        """Bật/tắt ghi video ra thư mục RECORD_DIR."""
        if self.record_writer is not None:
            self._stop_record("stopped by user")
        else:
            self._start_record()

    def _start_record(self):
        if self.last_frame is None:
            self._set_status("No frames received — cannot start recording")
            return
        try:
            os.makedirs(RECORD_DIR, exist_ok=True)
        except OSError as exc:
            self._set_status(f"Cannot create folder {RECORD_DIR}: {exc}")
            self._log(f"Recording: cannot create folder {RECORD_DIR} ({exc})")
            return

        try:
            target = max(1, int(self.vars["fps"].get()))
        except (KeyError, tk.TclError, ValueError):
            target = 15
        # Dùng FPS đo được nếu đã ổn định (thiếu sáng nên FPS thật thường thấp
        # hơn mức đặt); nhờ vậy file MP4 phát đúng tốc độ thực tế.
        self.record_fps = (int(round(self.fps_measured))
                           if self.fps_measured >= 5 else target)
        self.record_frames = 0
        self.record_t0 = time.time()
        self._size_warned = False
        self.record_path = ""
        self.record_part = 1
        self.record_seg_frames = 0
        # Tên gốc chung cho mọi đoạn của lần ghi này (kèm mili giây cho khỏi trùng)
        self.record_base = time.strftime("esp32cam_%Y%m%d_%H%M%S") + \
            f"_{int(time.time() * 1000) % 1000:03d}"
        self.record_seg_limit = (
            max(1, int(round(self.record_fps * RECORD_SEGMENT_SECONDS)))
            if RECORD_SEGMENT_SECONDS > 0 else 0)

        h, w = self.last_frame.shape[:2]
        if not self._open_writer(w, h):
            return
        self.record_btn.config(text="■ Stop recording")
        if RECORD_SEGMENT_SECONDS >= 60:
            seg = f", split every {RECORD_SEGMENT_SECONDS // 60} min"
        elif RECORD_SEGMENT_SECONDS > 0:
            seg = f", split every {RECORD_SEGMENT_SECONDS} s"
        else:
            seg = ""
        self._set_status(f"Recording to {RECORD_DIR} ({self.record_fps} fps{seg})")
        self._log(f"Recording: {self.record_path} ({w}x{h}, {self.record_fps} fps{seg})")

    def _open_writer(self, w, h):
        """Mở file MP4 mới; trả về True nếu thành công."""
        name = self.record_base or time.strftime("esp32cam_%Y%m%d_%H%M%S")
        if self.record_part > 1:
            name += f"_p{self.record_part}"      # đoạn 2, 3... khi chia 5 phút
        path = os.path.join(RECORD_DIR, name + ".mp4")
        # Không ghi đè file cũ (bắt đầu 2 lần trong cùng giây)
        n = 1
        while os.path.exists(path):
            n += 1
            path = os.path.join(RECORD_DIR, f"{name}_{n}.mp4")

        for fourcc in RECORD_CODECS:
            writer = cv2.VideoWriter(path, cv2.VideoWriter_fourcc(*fourcc),
                                     self.record_fps, (w, h))
            if writer.isOpened():
                self.record_writer = writer
                self.record_size = (w, h)
                self.record_path = path
                return True
            writer.release()

        self.record_writer = None
        self.record_size = None
        self._set_status("Cannot open MP4 file (missing codec?) — see log")
        self._log(f"Recording: OpenCV could not open {path} with codecs {RECORD_CODECS}")
        return False

    def _record_frame(self, frame):
        """Ghi 1 frame vào file đang mở.

        Kích thước frame thay đổi (đổi độ phân giải, ESP32 reboot rồi nối lại...)
        chỉ khiến frame đó được resize cho khớp cỡ file, KHÔNG tạo file mới. File
        mới chỉ được tạo khi đủ một đoạn (RECORD_SEGMENT_SECONDS).
        """
        if self.record_writer is None:
            return
        h, w = frame.shape[:2]
        if (w, h) != self.record_size:
            try:
                frame = cv2.resize(frame, self.record_size,
                                   interpolation=cv2.INTER_AREA)
            except cv2.error as exc:
                self._log(f"Recording: could not resize {w}x{h} frame ({exc})")
                return
            if not self._size_warned:
                self._size_warned = True
                self._log(f"Recording: {w}x{h} frame differs from file size "
                          f"{self.record_size[0]}x{self.record_size[1]} "
                          f"-> resized to fit (same file)")
        try:
            self.record_writer.write(frame)
            self.record_frames += 1
            self.record_seg_frames += 1
        except cv2.error as exc:
            self._log(f"Recording error: {exc}")
            self._stop_record("disk write error")
            return

        # Đủ thời lượng 1 đoạn -> chốt file và mở đoạn mới (không mất toàn bộ
        # bản ghi nếu file về sau bị lỗi).
        if self.record_seg_limit > 0 and self.record_seg_frames >= self.record_seg_limit:
            self._rotate_segment()

    def _rotate_segment(self):
        """Chốt file hiện tại, mở file đoạn kế tiếp với cùng kích thước."""
        size = self.record_size
        prev = os.path.basename(self.record_path)
        self._close_writer()
        if size is None:
            return
        self.record_part += 1
        self.record_seg_frames = 0
        if not self._open_writer(*size):
            self._stop_record("could not open next segment")
            return
        self._log(f"Recording: {RECORD_SEGMENT_SECONDS} s reached -> new segment "
                  f"{os.path.basename(self.record_path)} (previous: {prev})")

    def _close_writer(self):
        if self.record_writer is not None:
            try:
                self.record_writer.release()
            except cv2.error:
                pass
            self.record_writer = None

    def _stop_record(self, reason):
        self._close_writer()
        self.record_size = None
        dur = max(0.0, time.time() - self.record_t0) if self.record_t0 else 0.0
        self.record_t0 = 0.0
        try:
            self.record_btn.config(text="● Record MP4")
        except tk.TclError:
            pass
        if not self.record_path:
            return

        if self.record_frames == 0:
            # Không có frame nào -> xoá file rỗng, tránh để lại clip 0 giây
            try:
                os.remove(self.record_path)
            except OSError:
                pass
            self._log(f"Recording: no frames received ({reason}) -> removed empty file")
            self.record_path = ""
            return

        seg_txt = f", {self.record_part} segments" if self.record_part > 1 else ""
        self._set_status(f"Saved {self.record_path} ({self.record_frames} frames, "
                         f"~{dur:.0f} s{seg_txt}) — {reason}")
        self._log(f"Recording: saved {self.record_path} ({self.record_frames} frames, "
                  f"~{dur:.0f} s{seg_txt}, {reason})")
        self.record_path = ""

    # ---------- vòng cập nhật video ----------
    def _set_status_async(self, text):
        # Được gọi từ thread mạng -> chỉ đẩy vào queue, main thread sẽ hiển thị
        self.status_queue.put(text)

    def _set_status(self, text):
        # Dùng trong main thread (nút bấm, callback Tk)
        if not self._closing:
            self.status_var.set(text)

    def _drain_status(self):
        last = None
        try:
            while True:
                last = self.status_queue.get_nowait()
        except queue.Empty:
            pass
        if last is not None and not self._closing:
            self.status_var.set(last)

    def _drain_ctrl_log(self):
        try:
            while True:
                line = self.log_queue.get_nowait()
                if line in ("OK", "PONG"):
                    continue          # phản hồi bình thường của CFG/PING
                self._log("ESP32: " + self._format_ctrl_line(line))
        except queue.Empty:
            pass

    @staticmethod
    def _format_ctrl_line(text):
        """Đổi dòng 'STAT fps=.. exp=.. rssi=..' thành câu dễ đọc + gợi ý."""
        if not text.startswith("STAT"):
            return text

        kv = {}
        for part in text.split()[1:]:
            if "=" in part:
                key, val = part.split("=", 1)
                kv[key] = val

        def num(key, default=0.0):
            raw = str(kv.get(key, default))
            raw = "".join(ch for ch in raw if ch.isdigit() or ch in "+-.")
            try:
                return float(raw)
            except ValueError:
                return float(default)

        fps, exp, rssi = num("fps"), num("exp"), num("rssi")
        length, send, heap = num("len"), num("send"), num("heap")
        gain, maxexp = num("gain"), num("maxexp")
        txp = num("txp")

        notes = []
        if kv.get("cam") == "0":
            notes.append("CAMERA ERROR/NOT INITIALIZED — ESP32 retries every 5 s; "
                         "check the camera cable and restart")
        if txp and txp < 80:
            notes.append("TX power is limited below 20 dBm — check the 5V/2A supply "
                         "and PHY configuration")
        if maxexp and exp >= maxexp * 0.95:
            notes.append("AEC reached the exposure limit — lower FPS (if FPS sets "
                         "the limit), add light, or reduce resolution")
        # 1 dòng ≈ HTS/PCLK ≈ 2300/10 MHz ≈ 0,23 ms (khung 4:3 binned).
        # Trần 2352 dòng ≈ 0,54 s: exposure được phép dài hơn 1 frame.
        if exp >= 1500:
            notes.append("very low light: long exposure (~0.35–0.54 s/frame); "
                         "moving subjects may blur")
        elif exp >= 600:
            notes.append("low light: long exposure (~0.14–0.23 s/frame); "
                         "low FPS is expected")
        if gain >= 480:
            notes.append("very high gain (noisy image) — add light or lower "
                         "'Gain ceiling' for a cleaner image")
        if rssi != 0 and rssi <= -75:
            notes.append("weak Wi-Fi signal; move closer to the router")
        if send >= 200:
            notes.append("slow frame transmission")
        if length >= 100000:
            notes.append("very large JPEG frames — increase 'JPEG quality' "
                         "(higher value) to reduce Wi-Fi load")

        tail = ("  → " + "; ".join(notes)) if notes else ""
        mode = "" if kv.get("stream", "1") == "1" else "  [waiting: no client → 1 fps]"
        res = f"  size={kv['res']}" if kv.get("custom") == "1" and "res" in kv else ""
        return (f"fps={fps:.1f}  exp={int(exp)}/{int(maxexp) if maxexp else '?'} lines  "
                f"gain={kv.get('gain', '?')}  "
                f"len={int(length)} B  send={int(send)} ms  "
                f"rssi={int(rssi)} dBm  txp={int(txp) // 4 if txp else '?'} dBm  "
                f"heap={int(heap)} B{res}{tail}{mode}")

    def _log(self, text):
        self.info.config(state="normal")
        self.info.insert("end", text + "\n")
        if int(self.info.index("end-1c").split(".")[0]) > 200:
            self.info.delete("1.0", "50.0")
        self.info.see("end")
        self.info.config(state="disabled")

    def _pump(self):
        """Vòng cập nhật 10 ms. Toàn bộ thân hàm nằm trong try/except/finally:
        một lỗi bất ngờ (JPEG hỏng, cửa sổ video bị đóng, OpenCV lỗi driver)
        KHÔNG được làm mất lịch after() - nếu mất, video và UI đứng im."""
        if self._closing:
            return

        reschedule = True
        try:
            self._drain_status()
            self._drain_ctrl_log()

            # Kênh điều khiển im lặng quá lâu -> PING, hoặc cắt để ESP32 nối lại
            try:
                self.link.ctrl_watchdog()
            except Exception:
                pass

            # ESP32 vừa kết nối lại kênh điều khiển -> đẩy lại toàn bộ cấu hình
            if self.link.ctrl_connected and self.link.need_sync:
                self.link.need_sync = False
                self.apply()

            decoded = 0
            try:
                while True:
                    data = self.link.frames.get_nowait()
                    try:
                        np_arr = np.frombuffer(data, dtype=np.uint8)
                        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                    except Exception as exc:      # ảnh hỏng/không đủ bộ nhớ
                        self._log(f"Warning: JPEG decode failed ({exc})")
                        continue
                    if frame is not None:
                        self.last_frame = frame
                        decoded += 1       # đếm ĐỦ số frame, không chỉ 1/lần pump
                        self._record_frame(frame)
                    else:
                        self._log("Warning: could not decode JPEG")
            except queue.Empty:
                pass

            if decoded:
                self.frame_count += decoded
                now = time.time()
                if now - self._fps_t0 >= 1.0:
                    self.fps_measured = self.frame_count / (now - self._fps_t0)
                    self.frame_count = 0
                    self._fps_t0 = now
                    h, w = self.last_frame.shape[:2]
                    rec = (f" | REC {self.record_frames} frame"
                           if self.record_writer is not None else "")
                    self._log(f"Video {w}x{h} | {self.fps_measured:4.1f} fps | "
                              f"control: {'OK' if self.link.ctrl_connected else 'not connected'}"
                              f"{rec}")

            if self._video_window_open:
                try:
                    if cv2.getWindowProperty(
                            VIDEO_WINDOW, cv2.WND_PROP_VISIBLE) < 1:
                        self._video_window_open = False
                    else:
                        display_frame = (self.last_frame
                                         if self.last_frame is not None
                                         else self._waiting_frame)
                        cv2.imshow(VIDEO_WINDOW,
                                   apply_filter(display_frame, self.filter_mode))
                        if cv2.waitKey(1) & 0xFF == ord("q"):
                            reschedule = False
                            self.on_close()
                except cv2.error:
                    self._video_window_open = False
        except Exception as exc:
            # Không để một exception lạ giết vòng lặp: ghi log rồi chạy tiếp
            try:
                self._log(f"Update loop error: {exc}")
            except Exception:
                pass
        finally:
            if reschedule and not self._closing:
                try:
                    self.root.after(10, self._pump)
                except tk.TclError:
                    pass      # cửa sổ Tk đã bị huỷ

    def _open_video_window(self):
        if self._closing or self._video_window_open:
            return
        try:
            cv2.namedWindow(VIDEO_WINDOW, cv2.WINDOW_NORMAL)
            self._video_window_open = True
            display_frame = (self.last_frame if self.last_frame is not None
                             else self._waiting_frame)
            cv2.imshow(VIDEO_WINDOW, display_frame)
            cv2.waitKey(1)
        except cv2.error as exc:
            self._video_window_open = False
            self._set_status_async(f"Cannot open video window: {exc}")

    def on_close(self):
        if self._closing:
            return
        self._stop_record("application closed")   # finalize MP4 before exit
        self._closing = True
        # Huỷ lệnh gửi đang chờ: nếu không, after_cancel sẽ gọi apply() trên
        # widget đã bị destroy -> TclError
        if self._send_job is not None:
            try:
                self.root.after_cancel(self._send_job)
            except tk.TclError:
                pass
            self._send_job = None
        try:
            cv2.destroyAllWindows()
        except cv2.error:
            pass
        self.root.destroy()


def main():
    root = tk.Tk()
    link = CameraLink(status_cb=lambda text: None)
    panel = ControlPanel(root, link)
    root.protocol("WM_DELETE_WINDOW", panel.on_close)
    root.mainloop()


if __name__ == "__main__":
    main()
