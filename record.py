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
UDP_BROADCAST_INTERVAL = 3  # Send broadcast every 2 seconds  

# CHỌN BỘ LỌC TẠI ĐÂY: 
# "none"      -> Ảnh gốc
# "gray"      -> Ảnh trắng đen
# "blur"      -> Làm mịn / Khử nhiễu hạt
# "sharp"     -> Làm nét chi tiết
# "vintage"   -> Màu hoài cổ (ấm)
FILTER_MODE = "sharp" 

# Hàng đợi chứa các frame ảnh sẵn sàng hiển thị (Giới hạn tối đa 5 frame)
frame_queue = queue.Queue(maxsize=5)

def get_local_ip():
    """Get the local IP address of the machine"""
    try:
        # Create a UDP socket to find the local IP
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        local_ip = s.getsockname()[0]
        s.close()
        return local_ip
    except:
        return "127.0.0.1"

def udp_discovery_thread():
    """Thread to listen for ESP32 discovery requests and respond with server IP"""
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
            print(f"[*] Received discovery request from {addr}: {message}")

            # If ESP32 sends discovery request, respond with server IP
            if message == "DISCOVER_SERVER":
                response = f"KEY-IP {local_ip}"
                discovery_socket.sendto(response.encode('utf-8'), addr)
                print(f"[*] Sent server IP to {addr}: {response}")
    except Exception as e:
        print(f"[-] UDP Discovery error: {e}")
    finally:
        discovery_socket.close()

def apply_filter(frame, mode):
    """Hàm xử lý và áp dụng bộ lọc lên frame ảnh OpenCV"""
    if mode == "none":
        return frame
        
    elif mode == "gray":
        # Chuyển sang ảnh xám (Trắng đen)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # Chuyển ngược lại 3 kênh màu để đồng bộ hiển thị
        return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        
    elif mode == "blur":
        # Bộ lọc Gaussian Blur giúp làm mịn ảnh, giảm nhiễu hạt (noise) từ ESP32-CAM
        return cv2.GaussianBlur(frame, (5, 5), 0)
        
    elif mode == "sharp":
        # Ma trận Kernel giúp tăng cường độ sắc nét cho các đường viền
        kernel = np.array([[0, -1, 0], 
                           [-1, 5, -1], 
                           [0, -1, 0]])
        return cv2.filter2D(frame, -1, kernel)
        
    elif mode == "vintage":
        # Tạo hiệu ứng màu phim cũ bằng cách thay đổi ma trận màu sắc
        kernel = np.array([[0.272, 0.534, 0.131],
                           [0.349, 0.686, 0.168],
                           [0.393, 0.769, 0.189]])
        vintage_frame = cv2.transform(frame, kernel)
        # Giới hạn giá trị pixel trong khoảng 0-255 để tránh lỗi hiển thị
        return np.clip(vintage_frame, 0, 255).astype(np.uint8)
        
    return frame

def tcp_receiver_thread():
    """Luồng phụ: Chuyên nhận dữ liệu từ TCP và đẩy vào hàng đợi"""
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind((SERVER_IP, SERVER_PORT))
    server_socket.listen(1)
    print(f"[*] TCP Receiver Thread started. Listening on port {SERVER_PORT}...")

    try:
        while True:
            conn, addr = server_socket.accept()
            print(f"[*] Incoming connection from: {addr}.")
            
            try:
                # --- STEP 1: AUTHENTICATION ---
                client_auth_data = b""
                connection_lost = False
                while b"\n" not in client_auth_data:
                    packet = conn.recv(1024)
                    if not packet: 
                        connection_lost = True
                        break
                    client_auth_data += packet
                    if len(client_auth_data) > 100: 
                        break 
                
                if connection_lost:
                    conn.close()
                    continue

                received_password = client_auth_data.decode('utf-8', errors='ignore').strip()
                if received_password == AUTH_PASSWORD:
                    print(f"[+] Auth SUCCESSFUL for {addr}.")
                    conn.sendall(b"AUTH_OK\n")
                else:
                    print(f"[-] Auth FAILED for {addr}.")
                    conn.sendall(b"AUTH_FAIL\n")
                    conn.close()
                    continue
                    
                # --- STEP 2: VIDEO RECEPTION ---
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
                        print(f"[-] Invalid size {msg_size}. Resetting...")
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
                    
                    if not frame_queue.full():
                        frame_queue.put(frame_data)
                    else:
                        try:
                            frame_queue.get_nowait()
                            frame_queue.put(frame_data)
                        except queue.Empty:
                            pass
                            
            except Exception as e:
                print(f"[-] Connection error: {e}")
            finally:
                conn.close()
                print(f"[*] Disconnected from {addr}. Standing by...")
    finally:
        server_socket.close()

# ---------------------------------------------------------
# LUỒNG CHÍNH (MAIN THREAD): Xử lý bộ lọc và hiển thị UI
# ---------------------------------------------------------
if __name__ == "__main__":
    # Start UDP discovery thread (listen for ESP32 requests)
    discovery = threading.Thread(target=udp_discovery_thread, daemon=True)
    discovery.start()

    receiver = threading.Thread(target=tcp_receiver_thread, daemon=True)
    receiver.start()

    print(f"[*] Main UI Thread running. Active filter: {FILTER_MODE.upper()}")
    
    while True:
        try:
            frame_data = frame_queue.get(timeout=0.1)
            
            np_arr = np.frombuffer(frame_data, dtype=np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
            if frame is not None:
                # 👉 ÁP DỤNG BỘ LỌC TẠI ĐÂY TRƯỚC KHI HIỂN THỊ
                processed_frame = apply_filter(frame, FILTER_MODE)
                
                # Hiển thị frame đã qua xử lý
                cv2.imshow("ESP32-CAM Stream (Filtered)", processed_frame)
            else:
                print("[-] Warning: Failed to decode JPEG frame.")
                
        except queue.Empty:
            pass

        if cv2.waitKey(1) & 0xFF == ord('q'):
            print("[*] 'q' pressed. Exiting...")
            break

    cv2.destroyAllWindows()
    print("[*] Application closed.")
