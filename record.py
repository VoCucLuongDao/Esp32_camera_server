import socket
import struct
import cv2
import numpy as np
import threading
import queue
import time

# Configurations
SERVER_IP = "0.0.0.0"
SERVER_PORT = 1357
AUTH_PASSWORD = "pass187631"
UDP_BROADCAST_PORT = 5005

# 👉 THAM SỐ RAW ĐƯỢC LƯU SẴN - TỰ ĐỘNG NỐI ĐUÔI GỬI NGAY SAU KHI KẾT NỐI TCP
AUTO_RAW_CONFIG = "1,0,0,0,0,0,8000,600,640,320,0,0"

FILTER_MODE = "sharp" 
frame_queue = queue.Queue(maxsize=5)

def get_local_ip():
    """Lấy địa chỉ IP cục bộ của máy tính"""
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        local_ip = s.getsockname()[0]
        s.close()
        return local_ip
    except Exception:
        return "127.0.0.1"

def udp_discovery_thread():
    """Luồng lắng nghe yêu cầu khám phá từ ESP32 và phản hồi IP Server"""
    discovery_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    discovery_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    discovery_socket.bind(('0.0.0.0', UDP_BROADCAST_PORT))

    local_ip = get_local_ip()
    print(f"[*] UDP Discovery Thread started. Listening on port {UDP_BROADCAST_PORT}")
    print(f"[*] Server IP: {local_ip}")

    try:
        while True:
            data, addr = discovery_socket.recvfrom(1024)
            message = data.decode('utf-8', errors='ignore')
            
            if message == "DISCOVER_SERVER":
                response = f"KEY-IP {local_ip}"
                discovery_socket.sendto(response.encode('utf-8'), addr)
                print(f"[*] Responded to ESP32 discovery request from {addr}")
    except Exception as e:
        print(f"[-] UDP Discovery error: {e}")
    finally:
        discovery_socket.close()

def apply_filter(frame, mode):
    """Hàm xử lý và áp dụng bộ lọc đồ họa lên frame ảnh OpenCV"""
    if mode == "none":
        return frame
    elif mode == "gray":
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    elif mode == "blur":
        return cv2.GaussianBlur(frame, (5, 5), 0)
    elif mode == "sharp":
        kernel = np.array([[0, -1, 0], 
                           [-1, 5, -1], 
                           [0, -1, 0]])
        return cv2.filter2D(frame, -1, kernel)
    elif mode == "vintage":
        kernel = np.array([[0.272, 0.534, 0.131],
                           [0.349, 0.686, 0.168],
                           [0.393, 0.769, 0.189]])
        vintage_frame = cv2.transform(frame, kernel)
        return np.clip(vintage_frame, 0, 255).astype(np.uint8)
    return frame

def tcp_receiver_thread():
    """Luồng phụ: Gửi dữ liệu authen gộp thầm lặng và đón luồng nhận video TCP"""
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind((SERVER_IP, SERVER_PORT))
    server_socket.listen(1)
    print(f"[*] TCP Receiver Thread started. Listening on port {SERVER_PORT}...")

    try:
        while True:
            conn, addr = server_socket.accept()
            print(f"\n[+] Incoming connection from: {addr}.")
            
            try:
                # --- STEP 1: AUTHENTICATION + APPENDED RAW CONFIG ---
                # 👉 Tạo gói payload hỗn hợp: Mật khẩu nằm trước, lệnh SET_RAW nối ngay phía sau phân tách bằng \n
                combined_payload = f"{AUTH_PASSWORD}\nSET_RAW:{AUTO_RAW_CONFIG}\n"
                
                print(f"[*] Transmitting integrated handshake payload to ESP32...")
                conn.sendall(combined_payload.encode('utf-8'))
                
                # Chờ phản hồi xác thực ngược lại từ chip ESP32
                client_auth_data = b""
                connection_lost = False
                while b"\n" not in client_auth_data:
                    packet = conn.recv(1024)
                    if not packet: 
                        connection_lost = True
                        break
                    client_auth_data += packet
                    if len(client_auth_data) > 32: 
                        break 
                
                if connection_lost:
                    print("[-] Connection lost during handshake.")
                    conn.close()
                    continue

                received_response = client_auth_data.decode('utf-8', errors='ignore').strip()
                if received_response == "AUTH_OK":
                    print(f"[+] Handshake SUCCESSFUL! ESP32 accepted configuration. Initializing stream...")
                else:
                    print(f"[-] Handshake DENIED by ESP32. Chip response: {received_response}")
                    conn.close()
                    continue
                    
                # --- STEP 2: VIDEO RECEPTION LOOP ---
                data = b""
                payload_size = struct.calcsize("<I")
                
                while True:
                    while len(data) < payload_size:
                        packet = conn.recv(4096)
                        if not packet:
                            connection_lost = True
                            break
                        data += packet
                    
                    if connection_lost or len(data) < payload_size: 
                        break
                    
                    packed_msg_size = data[:payload_size]
                    data = data[payload_size:]
                    msg_size = struct.unpack("<I", packed_msg_size)[0]
                    
                    if msg_size == 0 or msg_size > 1024 * 1024:
                        print(f"[-] Invalid frame size received ({msg_size}). Resetting...")
                        break
                    
                    while len(data) < msg_size:
                        packet = conn.recv(4096)
                        if not packet: 
                            connection_lost = True
                            break
                        data += packet
                        
                    if connection_lost or len(data) < msg_size: 
                        break
                    
                    frame_data = data[:msg_size]
                    data = data[msg_size:]
                    
                    # Đẩy dữ liệu nhị phân ảnh JPEG vào hàng đợi
                    if not frame_queue.full():
                        frame_queue.put(frame_data)
                    else:
                        try:
                            frame_queue.get_nowait()
                            frame_queue.put(frame_data)
                        except queue.Empty:
                            pass
                            
            except Exception as e:
                print(f"[-] Connection session error: {e}")
            finally:
                conn.close()
                print(f"[-] Closed session socket with {addr}. Standing by for reconnect...")
    finally:
        server_socket.close()

# ---------------------------------------------------------
# LUỒNG CHÍNH (MAIN THREAD): Xử lý hiển thị cửa sổ giao diện video
# ---------------------------------------------------------
if __name__ == "__main__":
    # 1. Khởi động luồng UDP Discovery
    discovery_thread = threading.Thread(target=udp_discovery_thread, daemon=True)
    discovery_thread.start()

    # 2. Khởi động luồng kết nối nhận video TCP
    receiver_thread = threading.Thread(target=tcp_receiver_thread, daemon=True)
    receiver_thread.start()

    print(f"[*] Main UI Thread running. Active filter: {FILTER_MODE.upper()}")
    print("[*] Focus the video screen window and press 'q' to exit safely.")
    
    while True:
        try:
            # Lấy mảng byte từ queue giải mã ảnh màu
            frame_data = frame_queue.get(timeout=0.1)
            np_arr = np.frombuffer(frame_data, dtype=np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
            if frame is not None:
                # Áp dụng bộ lọc mịn/nét/vintage trước khi kết xuất ảnh
                processed_frame = apply_filter(frame, FILTER_MODE)
                cv2.imshow("ESP32-CAM Stream (Filtered)", processed_frame)
            else:
                print("[-] Warning: Failed to decode incoming JPEG frame data.")
        except queue.Empty:
            pass

        # Nhấn phím 'q' trên màn hình OpenCV để ngắt chương trình sạch sẽ
        if cv2.waitKey(1) & 0xFF == ord('q'):
            print("[*] 'q' command pressed. Shutting down application...")
            break

    cv2.destroyAllWindows()
    print("[*] Terminal session successfully finished.")
