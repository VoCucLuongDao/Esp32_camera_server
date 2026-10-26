"""
ESP32-CAM client + bảng điều khiển (Tkinter).

Kiến trúc 2 kênh TCP:
  - Cổng 1357 : ESP32 -> PC, truyền frame nhị phân [uint32 len][JPEG]
  - Cổng 1358 : PC -> ESP32, gửi lệnh text "CFG key=value ..." và nhận OK/ERR

Chạy:  python record.py
Thoát: nhấn 'q' ở cửa sổ video hoặc đóng cửa sổ điều khiển.
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

MAX_FRAME_SIZE = 1024 * 1024  # 1 MB, khớp với kiểm tra phía ESP32
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

WB_MODES = [("Auto", 0), ("Nắng", 1), ("Mây", 2), ("Văn phòng", 3), ("Đèn sợi đốt", 4)]
EFFECTS = [("Không", 0), ("Âm bản", 1), ("Trắng đen", 2), ("Ánh đỏ", 3),
           ("Ánh xanh lá", 4), ("Ánh xanh dương", 5), ("Sepia", 6)]

# name, nhãn hiển thị, min, max, mặc định
# Mặc định = bộ "thiếu sáng" (khớp camera_config_defaults() trong firmware).
SLIDERS = [
    ("brightness", "Độ sáng", -2, 2, 1),
    ("contrast", "Tương phản", -2, 2, 1),
    ("saturation", "Độ bão hoà", -2, 2, 1),
    ("sharpness", "Độ nét", -2, 2, 2),
    ("denoise", "Khử nhiễu (0-8)", 0, 8, 3),
    ("ae_level", "Bù sáng AE", -2, 2, 2),
    ("gainceiling", "Trần gain (0-6)", 0, 6, 5),
    ("quality", "Chất lượng JPEG (0 nét nhất)", 0, 63, 6),
    ("aec_value", "Phơi sáng thủ công", 0, 1200, 400),
    ("agc_gain", "Gain thủ công", 0, 30, 0),
    ("fps", "FPS gửi frame", 1, 30, 10),
]

# Bộ thông số nhanh cho các điều kiện ánh sáng thường gặp. Chỉ gồm các thanh
# trượt; độ phân giải / lật ảnh / white balance giữ nguyên như người dùng đặt.
LOW_LIGHT_PRESET = {
    "brightness": 1, "contrast": 1, "saturation": 1, "sharpness": 2,
    "denoise": 3, "ae_level": 2, "gainceiling": 5, "fps": 10,
}
# Ưu tiên chi tiết khuôn mặt: ít khử nhiễu + làm nét tối đa + gain cao nhất.
# Đổi lại ảnh sẽ hạt hơn; nếu mặt vẫn nhoè thì nguyên nhân là phơi sáng dài
# (người di chuyển) hoặc mất nét ống kính, không phải tham số ISP.
FACE_PRESET = {
    "brightness": 1, "contrast": 1, "saturation": 1, "sharpness": 2,
    "denoise": 1, "ae_level": 2, "gainceiling": 5, "fps": 10,
}
DAYLIGHT_PRESET = {
    "brightness": 0, "contrast": 1, "saturation": 1, "sharpness": 1,
    "denoise": 2, "ae_level": 0, "gainceiling": 2, "fps": 15,
}

DEFAULTS = {name: default for name, _, _, _, default in SLIDERS}
DEFAULTS.update({
    "framesize": 12, "wb_mode": 0, "effect": 0,
    "hmirror": 0, "vflip": 0, "aec": 1, "agc": 1,
    "bpc": 1, "wpc": 1, "lenc": 1, "raw_gma": 1,
    # kích thước khung hình tuỳ chỉnh (bỏ qua framesize khi custom = 1)
    "custom": 0, "width": 1024, "height": 768,
})

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
                        self._status(f"UDP discovery lỗi: {exc}")
                        time.sleep(1.0)
                        break
                    if data.decode("utf-8", errors="ignore") == "DISCOVER_SERVER":
                        try:
                            sock.sendto(f"KEY-IP {local_ip}".encode(), addr)
                        except OSError:
                            continue
                        self._status(f"ESP32 {addr[0]} hỏi IP, đã trả lời {local_ip}")
            except OSError as exc:
                self._status(f"Không mở được cổng UDP {UDP_BROADCAST_PORT}: {exc} "
                             f"— thử lại sau 5 s")
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
                self._status(f"Đang chờ video trên cổng {SERVER_PORT}...")
            except OSError as exc:
                self._status(f"Không mở được cổng video {SERVER_PORT}: {exc} "
                             f"(có chương trình khác đang dùng?) — thử lại sau 5 s")
                if srv is not None:
                    srv.close()
                time.sleep(5.0)
                continue

            while True:
                try:
                    conn, addr = srv.accept()
                except OSError as exc:
                    self._status(f"Lỗi chờ video: {exc}")
                    break
                threading.Thread(target=self._video_session, args=(conn, addr),
                                 daemon=True).start()

            srv.close()
            time.sleep(2.0)

    def _video_session(self, conn, addr):
        try:
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
                self._status(f"Xác thực thất bại: {auth!r}")
                return

            self.video_connected = True
            self._status(f"Video đã kết nối từ {addr[0]}")

            while True:
                header = self._recv_exact(conn, 4)
                if header is None:
                    break
                size = struct.unpack("<I", header)[0]
                if size == 0 or size > MAX_FRAME_SIZE:
                    self._status(f"Kích thước frame không hợp lệ ({size}), ngắt kết nối")
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
            self._status(f"Video quá {VIDEO_STALL_TIMEOUT:.0f} s không có dữ liệu — "
                         f"ngắt để ESP32 kết nối lại")
        except OSError as exc:
            self._status(f"Video session lỗi: {exc}")
        except Exception as exc:        # lỗi lạ không được làm chết tiến trình
            self._status(f"Video session lỗi không xác định: {exc}")
        finally:
            self.video_connected = False
            try:
                conn.close()
            except OSError:
                pass
            self._status("Video đã ngắt, chờ ESP32 kết nối lại...")

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
                self._status(f"Không mở được cổng điều khiển {CONTROL_PORT}: {exc} "
                             f"(có chương trình khác đang dùng?) — thử lại sau 5 s")
                if srv is not None:
                    srv.close()
                time.sleep(5.0)
                continue

            while True:
                try:
                    conn, addr = srv.accept()
                except OSError as exc:
                    self._status(f"Lỗi chờ kênh điều khiển: {exc}")
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
        self._status(f"Kênh điều khiển đã kết nối từ {addr[0]}:{CONTROL_PORT}")

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
            self._status(f"Kênh điều khiển lỗi: {exc}")
        finally:
            with self.ctrl_lock:
                if self.ctrl_sock is conn:
                    self.ctrl_sock = None
            self.ctrl_connected = False
            try:
                conn.close()
            except OSError:
                pass
            self._status("Kênh điều khiển đã đóng")

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
            self._status("Cấu hình quá dài, không gửi (lỗi nội bộ)")
            return False

        with self.ctrl_lock:
            if self.ctrl_sock is None:
                return False
            try:
                self.ctrl_sock.sendall(line.encode())
                self.last_ctrl_rx = time.time()
                return True
            except OSError as exc:
                self._status(f"Gửi lệnh thất bại: {exc}")
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
                self._status(f"Gửi lệnh thất bại: {exc}")
                return False

    def ctrl_watchdog(self):
        """Gửi PING khi kênh im lặng; cắt kênh nếu mất phản hồi quá lâu.
        Gọi từ main thread (vòng _pump)."""
        if not self.ctrl_connected:
            return
        idle = self.ctrl_idle_seconds()
        if idle >= CTRL_DEAD_AFTER:
            self._status("Kênh điều khiển không phản hồi — đóng để ESP32 nối lại")
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

        self.status_var = tk.StringVar(value="Khởi động...")

        self._build_ui()
        self._building = False    # từ đây mọi thay đổi mới được gửi đi
        self.link.status_cb = self._set_status_async
        self.link.on_ctrl_line = self.log_queue.put
        self.link.start()
        self.root.after(30, self._pump)

    # ---------- dựng giao diện ----------
    def _build_ui(self):
        self.root.title("ESP32-CAM — Bảng điều khiển OV3660")
        self.root.geometry("1180x760")

        main = ttk.Frame(self.root, padding=8)
        main.pack(fill="both", expand=True)

        # ---- bên trái: thông tin video ----
        left = ttk.Frame(main)
        left.pack(side="left", fill="both", expand=True)

        box = ttk.LabelFrame(left, text="Video (cửa sổ OpenCV)", padding=8)
        box.pack(fill="both", expand=True)
        self.info = tk.Text(box, height=12, width=44, state="disabled",
                            font=("Consolas", 9))
        self.info.pack(fill="both", expand=True)
        ttk.Label(left, text="Nhấn 'q' ở cửa sổ video để thoát.",
                  foreground="#555").pack(anchor="w", pady=(6, 0))

        # ---- bên phải: bảng điều khiển ----
        right = ttk.Frame(main, width=460)
        right.pack(side="right", fill="y", padx=(10, 0))
        right.pack_propagate(False)

        notebook = ttk.Notebook(right)
        notebook.pack(fill="both", expand=True)

        self._build_image_tab(notebook)
        self._build_sensor_tab(notebook)

        # ---- hàng nút ----
        buttons = ttk.Frame(right)
        buttons.pack(fill="x", pady=(8, 2))

        # Mọi thay đổi được gửi tự động (~150 ms) nên nút này chỉ là "gửi lại
        # toàn bộ" khi cần (ví dụ sau khi ESP32 vừa khởi động lại).
        ttk.Button(buttons, text="Gửi lại", command=self.apply).pack(side="left")
        ttk.Button(buttons, text="Mặc định", command=self.reset_defaults).pack(side="left", padx=4)
        ttk.Button(buttons, text="Đọc trạng thái", command=self.read_stats).pack(side="left", padx=4)
        self.record_btn = ttk.Button(buttons, text="● Ghi MP4",
                                     command=self.toggle_record)
        self.record_btn.pack(side="left", padx=(4, 0))

        ttk.Label(right, text="Thay đổi được gửi tự động — không cần bấm nút "
                              "(tối đa 1 lần/giây).",
                  foreground="#666", wraplength=410).pack(anchor="w", pady=(0, 2))

        ttk.Button(right, text="Khởi động lại ESP32",
                   command=self.restart_esp).pack(fill="x", pady=(2, 4))

        ttk.Separator(right).pack(fill="x", pady=6)
        ttk.Label(right, text="Trạng thái:").pack(anchor="w")
        ttk.Label(right, textvariable=self.status_var, wraplength=410,
                  foreground="#0a5").pack(anchor="w", pady=(2, 0))

    def _build_image_tab(self, notebook):
        tab = ttk.Frame(notebook, padding=8)
        notebook.add(tab, text="Hình ảnh")

        # Bộ thông số nhanh: đổi cả nhóm thanh trượt bằng 1 cú bấm
        row = ttk.Frame(tab)
        row.pack(fill="x", pady=(0, 6))
        ttk.Label(row, text="Chế độ", width=10).pack(side="left")
        ttk.Button(row, text="Thiếu sáng",
                   command=lambda: self.apply_preset(LOW_LIGHT_PRESET,
                                                    "thiếu sáng")).pack(side="left")
        ttk.Button(row, text="Rõ mặt",
                   command=lambda: self.apply_preset(FACE_PRESET,
                                                    "rõ mặt")).pack(side="left", padx=4)
        ttk.Button(row, text="Ban ngày",
                   command=lambda: self.apply_preset(DAYLIGHT_PRESET,
                                                    "ban ngày")).pack(side="left")

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

        # độ phân giải chuẩn
        row = ttk.Frame(tab)
        row.pack(fill="x", pady=(10, 2))
        ttk.Label(row, text="Độ phân giải", width=27).pack(side="left")
        self.combos["framesize"] = ttk.Combobox(
            row, state="readonly", width=20,
            values=[f"{n}  (#{i})" for n, i in FRAME_SIZES])
        self.combos["framesize"].current(12)
        self.combos["framesize"].pack(side="left")
        self.combos["framesize"].bind("<<ComboboxSelected>>", lambda _e: self._schedule_send())

        # kích thước tuỳ chỉnh: bật -> bỏ qua ô "Độ phân giải" ở trên
        row = ttk.Frame(tab)
        row.pack(fill="x", pady=(8, 2))
        self.custom_var = tk.IntVar(value=DEFAULTS["custom"])
        ttk.Checkbutton(row, text="Kích thước tuỳ chỉnh (W × H)",
                        variable=self.custom_var,
                        command=self._on_custom_toggle).pack(side="left")

        row = ttk.Frame(tab)
        row.pack(fill="x", pady=2)
        ttk.Label(row, text="Rộng × Cao", width=27).pack(side="left")
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
        ttk.Label(row, text="  pixel", foreground="#666").pack(side="left")

        # gõ tay số rồi Enter / rời ô cũng gửi đi
        for spin in (self.width_spin, self.height_spin):
            spin.bind("<Return>", lambda _e: self._schedule_send())
            spin.bind("<FocusOut>", lambda _e: self._schedule_send())

        ttk.Label(tab, justify="left", wraplength=390, foreground="#666",
                  text="Số chẵn, tối đa 2048 × 1536 (vùng cảm biến OV3660). "
                       "Mọi kích thước đều thu nhỏ TOÀN KHUNG cảm biến (giữ "
                       "góc nhìn rộng); cửa sổ được canh giữa nên ảnh luôn lấy "
                       "từ trung tâm, không lệch góc. Kích thước thật xem ở ô "
                       "log (STAT res=...).").pack(anchor="w", pady=(2, 0))

        self._update_res_state()

        # bộ lọc phía client
        row = ttk.Frame(tab)
        row.pack(fill="x", pady=2)
        ttk.Label(row, text="Bộ lọc hiển thị (PC)", width=27).pack(side="left")
        self.filter_combo = ttk.Combobox(row, state="readonly", width=20,
                                         values=FILTER_MODES)
        self.filter_combo.set(self.filter_mode)
        self.filter_combo.pack(side="left")
        self.filter_combo.bind("<<ComboboxSelected>>",
                               lambda _e: setattr(self, "filter_mode", self.filter_combo.get()))

    def _build_sensor_tab(self, notebook):
        tab = ttk.Frame(notebook, padding=8)
        notebook.add(tab, text="Cảm biến OV3660")

        # lật / gương
        for name, label in (("hmirror", "Lật ngang (mirror 0x3821)"),
                            ("vflip", "Lật dọc (flip 0x3820)")):
            var = tk.IntVar(value=DEFAULTS[name])
            self.checks[name] = var
            ttk.Checkbutton(tab, text=label, variable=var,
                            command=self._schedule_send).pack(anchor="w", pady=2)

        # auto exposure / gain
        ttk.Separator(tab).pack(fill="x", pady=6)
        self.checks["aec"] = tk.IntVar(value=DEFAULTS["aec"])
        ttk.Checkbutton(tab, text="Auto Exposure (AEC)", variable=self.checks["aec"],
                        command=self._schedule_send).pack(anchor="w", pady=2)
        self.checks["agc"] = tk.IntVar(value=DEFAULTS["agc"])
        ttk.Checkbutton(tab, text="Auto Gain (AGC)", variable=self.checks["agc"],
                        command=self._schedule_send).pack(anchor="w", pady=2)

        # hiệu chỉnh điểm ảnh
        ttk.Separator(tab).pack(fill="x", pady=6)
        for name, label in (("bpc", "BPC — sửa điểm đen"),
                            ("wpc", "WPC — sửa điểm trắng"),
                            ("lenc", "LENC — bù sáng viền ống kính"),
                            ("raw_gma", "Gamma RAW")):
            var = tk.IntVar(value=DEFAULTS[name])
            self.checks[name] = var
            ttk.Checkbutton(tab, text=label, variable=var,
                            command=self._schedule_send).pack(anchor="w", pady=2)

        # white balance + hiệu ứng
        ttk.Separator(tab).pack(fill="x", pady=6)
        row = ttk.Frame(tab)
        row.pack(fill="x", pady=2)
        ttk.Label(row, text="Cân bằng trắng", width=18).pack(side="left")
        self.combos["wb_mode"] = ttk.Combobox(row, state="readonly", width=18,
                                              values=[f"{n}" for n, _ in WB_MODES])
        self.combos["wb_mode"].current(0)
        self.combos["wb_mode"].pack(side="left")
        self.combos["wb_mode"].bind("<<ComboboxSelected>>", lambda _e: self._schedule_send())

        row = ttk.Frame(tab)
        row.pack(fill="x", pady=2)
        ttk.Label(row, text="Hiệu ứng màu", width=18).pack(side="left")
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
                  text="Lưu ý: các tham số này ghi trực tiếp vào thanh ghi OV3660 "
                       "(0x3503, 0x3501/02, 0x3508/09, 0x3820/21, 0x4407...). "
                       "Thay đổi được áp dụng ở task capture nên video có thể "
                       "giật 1-2 frame.").pack(anchor="w", pady=(12, 0))

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
            self._set_status(f"Giá trị không hợp lệ: {exc}")
            return
        if not self.link.send_params(params):
            self._set_status("Chưa kết nối kênh điều khiển — không gửi được tham số")
            return
        self._set_status("Đã gửi cấu hình: " +
                         " ".join(f"{k}={v}" for k, v in list(params.items())[:4]) + " ...")

    def apply_preset(self, preset, name):
        """Áp nhanh một nhóm thông số ánh sáng rồi gửi xuống ESP32."""
        for key, value in preset.items():
            var = self.vars.get(key)
            if var is None:
                continue
            var.set(value)
            self.scales[key].set(value)
            self.value_labels[key].config(text=str(value))
        self._schedule_send()
        self._set_status(f"Đã chọn chế độ {name}: sẽ gửi cấu hình mới")

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
            self._set_status("Chưa kết nối kênh điều khiển")
            return
        self._set_status("Đã yêu cầu ESP32 khởi động lại")

    def read_stats(self):
        if not self.link.send_raw("STAT"):
            self._set_status("Chưa kết nối kênh điều khiển")
            return
        self._set_status("Đã hỏi trạng thái ESP32...")

    # ---------- ghi video MP4 ----------
    def toggle_record(self):
        """Bật/tắt ghi video ra thư mục RECORD_DIR."""
        if self.record_writer is not None:
            self._stop_record("người dùng dừng")
        else:
            self._start_record()

    def _start_record(self):
        if self.last_frame is None:
            self._set_status("Chưa có frame nào — chưa ghi được video")
            return
        try:
            os.makedirs(RECORD_DIR, exist_ok=True)
        except OSError as exc:
            self._set_status(f"Không tạo được thư mục {RECORD_DIR}: {exc}")
            self._log(f"Ghi video: không tạo được thư mục {RECORD_DIR} ({exc})")
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
        self.record_btn.config(text="■ Dừng ghi")
        if RECORD_SEGMENT_SECONDS >= 60:
            seg = f", chia mỗi {RECORD_SEGMENT_SECONDS // 60} phút"
        elif RECORD_SEGMENT_SECONDS > 0:
            seg = f", chia mỗi {RECORD_SEGMENT_SECONDS} giây"
        else:
            seg = ""
        self._set_status(f"Đang ghi video vào {RECORD_DIR} ({self.record_fps} fps{seg})")
        self._log(f"Ghi video: {self.record_path} ({w}x{h}, {self.record_fps} fps{seg})")

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
        self._set_status("Không mở được file MP4 (thiếu codec?) — xem ô log")
        self._log(f"Ghi video: OpenCV không mở được {path} với codec {RECORD_CODECS}")
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
                self._log(f"Ghi video: không resize được frame {w}x{h} ({exc})")
                return
            if not self._size_warned:
                self._size_warned = True
                self._log(f"Ghi video: frame {w}x{h} khác cỡ file "
                          f"{self.record_size[0]}x{self.record_size[1]} "
                          f"-> resize cho khớp (vẫn 1 file)")
        try:
            self.record_writer.write(frame)
            self.record_frames += 1
            self.record_seg_frames += 1
        except cv2.error as exc:
            self._log(f"Ghi video lỗi: {exc}")
            self._stop_record("lỗi ghi đĩa")
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
            self._stop_record("không mở được file đoạn mới")
            return
        self._log(f"Ghi video: đủ {RECORD_SEGMENT_SECONDS} s -> đoạn mới "
                  f"{os.path.basename(self.record_path)} (trước: {prev})")

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
            self.record_btn.config(text="● Ghi MP4")
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
            self._log(f"Ghi video: không có frame nào ({reason}) -> huỷ file rỗng")
            self.record_path = ""
            return

        seg_txt = f", {self.record_part} đoạn" if self.record_part > 1 else ""
        self._set_status(f"Đã lưu {self.record_path} ({self.record_frames} frame, "
                         f"~{dur:.0f} s{seg_txt}) — {reason}")
        self._log(f"Ghi video: lưu {self.record_path} ({self.record_frames} frame, "
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
        gain = num("gain")

        notes = []
        if kv.get("cam") == "0":
            notes.append("CAMERA LỖI/CHƯA KHỞI TẠO — ESP32 tự thử lại mỗi 5 s, "
                         "kiểm tra cáp camera rồi khởi động lại")
        if exp >= 1000:
            notes.append("thiếu sáng nặng: cảm biến phơi sáng ~1 s/frame")
        elif exp >= 500:
            notes.append("ánh sáng yếu: phơi sáng dài, FPS thấp là bình thường")
        if gain >= 480:
            notes.append("gain rất cao (ảnh dễ hạt) — nên bật nút 'Thiếu sáng' "
                         "hoặc thêm đèn")
        if rssi != 0 and rssi <= -75:
            notes.append("WiFi yếu, nên lại gần router")
        if send >= 200:
            notes.append("gửi frame qua mạng chậm")

        tail = ("  → " + "; ".join(notes)) if notes else ""
        mode = "" if kv.get("stream", "1") == "1" else "  [chờ: không có client → 1 fps]"
        res = f"  cỡ={kv['res']}" if kv.get("custom") == "1" and "res" in kv else ""
        return (f"fps={fps:.1f}  exp={int(exp)} dòng  gain={kv.get('gain', '?')}  "
                f"len={int(length)} B  send={int(send)} ms  "
                f"rssi={int(rssi)} dBm  heap={int(heap)} B{res}{tail}{mode}")

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

            got = False
            try:
                while True:
                    data = self.link.frames.get_nowait()
                    try:
                        np_arr = np.frombuffer(data, dtype=np.uint8)
                        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                    except Exception as exc:      # ảnh hỏng/không đủ bộ nhớ
                        self._log(f"Cảnh báo: giải mã JPEG lỗi ({exc})")
                        continue
                    if frame is not None:
                        self.last_frame = frame
                        got = True
                        self._record_frame(frame)
                    else:
                        self._log("Cảnh báo: không giải mã được JPEG")
            except queue.Empty:
                pass

            if got:
                self.frame_count += 1
                now = time.time()
                if now - self._fps_t0 >= 1.0:
                    self.fps_measured = self.frame_count / (now - self._fps_t0)
                    self.frame_count = 0
                    self._fps_t0 = now
                    h, w = self.last_frame.shape[:2]
                    rec = (f" | REC {self.record_frames} frame"
                           if self.record_writer is not None else "")
                    self._log(f"Video {w}x{h} | {self.fps_measured:4.1f} fps | "
                              f"điều khiển: {'OK' if self.link.ctrl_connected else 'chưa'}"
                              f"{rec}")

            if self.last_frame is not None:
                try:
                    cv2.imshow(VIDEO_WINDOW,
                               apply_filter(self.last_frame, self.filter_mode))
                except cv2.error:
                    # Người dùng đã đóng cửa sổ video -> tạo lại rồi hiện tiếp
                    self._reopen_video_window()
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    reschedule = False
                    self.on_close()
        except Exception as exc:
            # Không để một exception lạ giết vòng lặp: ghi log rồi chạy tiếp
            try:
                self._log(f"Lỗi vòng cập nhật: {exc}")
            except Exception:
                pass
        finally:
            if reschedule and not self._closing:
                try:
                    self.root.after(10, self._pump)
                except tk.TclError:
                    pass      # cửa sổ Tk đã bị huỷ

    def _reopen_video_window(self):
        try:
            cv2.namedWindow(VIDEO_WINDOW, cv2.WINDOW_NORMAL)
        except cv2.error:
            pass

    def on_close(self):
        if self._closing:
            return
        self._stop_record("đóng chương trình")   # chốt file MP4 trước khi thoát
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
