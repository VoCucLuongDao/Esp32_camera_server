import socket
import struct
import cv2
import numpy as np
import threading
import queue
import time
import sys

# Configurations
SERVER_IP = "0.0.0.0"
SERVER_PORT = 1357
AUTH_PASSWORD = "pass187631"
UDP_BROADCAST_PORT = 5005

# CHỌN BỘ LỌC MẶC ĐỊNH TẠI ĐÂY: 
# "none"      -> Ảnh gốc
# "gray"      -> Ảnh trắng đen
# "blur"      -> Làm mịn / Khử nhiễu hạt
# "sharp"     -> Làm nét chi tiết
# "vintage"   -> Màu hoài cổ (ấm)
FILTER_MODE = "sharp" 

# Hàng đợi chứa các frame ảnh sẵn sàng hiển thị (Giới hạn tối đa 5 frame)
frame_queue = queue.Queue(maxsize=5)

# Biến toàn cục lưu trữ kết nối hiện tại để gửi lệnh bất kỳ lúc nào
current_client_conn = None
conn_lock = threading.Lock()

def get_local_ip():
    """Lấy địa chỉ IP cục bộ của máy tính"""
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        local_ip = s.getsockname()[0]
        s.close()
        return local_ip
    except:
        return "127.0.0.1"

def udp_discovery_thread():
    """Luồng lắng nghe các yêu cầu khám phá từ ESP32 và phản hồi IP của Server"""
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
                print(f"\n[*] Sent server IP to ESP32 {addr}: {response}")
    except Exception as e:
        print(f"[-] UDP Discovery error: {e}")
    finally:
        discovery_socket.close()

def apply_filter(frame, mode):
    """Hàm xử lý và áp dụng bộ lọc lên frame ảnh OpenCV"""
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
    """Luồng phụ: Chuyên nhận dữ liệu mạng từ TCP và đẩy vào hàng đợi hình ảnh"""
    global current_client_conn
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind((SERVER_IP, SERVER_PORT))
    server_socket.listen(1)
    print(f"[*] TCP Receiver Thread started. Listening on port {SERVER_PORT}...")

    try:
        while True:
            conn, addr = server_socket.accept()
            print(f"\n[+] Incoming connection from: {addr}.")
            
            with conn_lock:
                current_client_conn = conn
            
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
                with conn_lock:
                    current_client_conn = None
                conn.close()
                print(f"[-] Disconnected from {addr}. Standing by...")
    finally:
        server_socket.close()

def command_sender_thread():
    """Luồng Terminal: Đứng đợi và bắt sự kiện nhập tham số RAW hợp lệ liên tục"""
    time.sleep(1.5)
    print("\n" + "="*75)
    print("[*] TERMINAL ACTIVE: Video stream đang chạy tự động song song.")
    print("[*] ĐỊNH DẠNG YÊU CẦU (Số từ 0 - 10000):")
    print('    RAW: 0,0,0,0,0,240,1280,240,1280,240,0,0   (Sau đó nhấn ENTER)')
    print("="*75 + "\n")
    
    while True:
        try:
            user_input = input().strip()
            if not user_input:
                continue
                
            if not user_input.startswith("RAW:"):
                print("[-] [PHẢN HỒI]: Sai định dạng! Chuỗi nhập phải bắt đầu bằng 'RAW:'")
                print("👉 Mời nhập lại lần tiếp theo:")
                continue
            
            raw_params_str = user_input[4:].strip()
            
            if raw_params_str.count(',') != 11:
                print(f"[-] [PHẢN HỒI]: Thiếu hoặc thừa số! Nhận được {raw_params_str.count(',') + 1}/12 tham số.")
                print("👉 Mời nhập lại lần tiếp theo:")
                continue
            
            try:
                num_list = [int(x.strip()) for x in raw_params_str.split(',')]
                out_of_range = [n for n in num_list if n < 0 or n > 10000]
                if out_of_range:
                    print(f"[-] [PHẢN HỒI]: Giá trị không hợp lệ {out_of_range}! Tất cả các số phải từ 0 đến 10000.")
                    print("👉 Mời nhập lại lần tiếp theo:")
                    continue
            except ValueError:
                print("[-] [PHẢN HỒI]: Lỗi định dạng số! Chuỗi chứa ký tự lạ không phải số nguyên.")
                print("👉 Mời nhập lại lần tiếp theo:")
                continue
                
            cmd_string = f"SET_RAW:{raw_params_str}\n"
            
            with conn_lock:
                if current_client_conn is not None:
                    try:
                        current_client_conn.sendall(cmd_string.encode('utf-8'))
                        print(f"[+] [PHẢN HỒI THÀNH CÔNG]: Đã ghi cấu hình RAW mới vào ESP32: {raw_params_str}")
                    except Exception as ex:
                        print(f"[-] [PHẢN HỒI LỖI]: Lỗi đường truyền mạng: {ex}")
                else:
                    print("[-] [PHẢN HỒI LỖI]: ESP32 chưa kết nối thành công, lệnh bị hủy.")
            
            print("👉 Mời nhập lại lần tiếp theo:")
            
        except Exception as e:
            print(f"[-] Lỗi hệ thống CLI: {e}")
            time.sleep(1)

if __name__ == "__main__":
    discovery = threading.Thread(target=udp_discovery_thread, daemon=True)
    discovery.start()

    receiver = threading.Thread(target=tcp_receiver_thread, daemon=True)
    receiver.start()

    cli_sender = threading.Thread(target=command_sender_thread, daemon=True)
    cli_sender.start()

    print(f"[*] Main UI Thread running. Active filter: {FILTER_MODE.upper()}")
    print("[*] Để thoát chương trình, click chuột vào cửa sổ ảnh và bấm phím 'q'.")
    
    while True:
        try:
            frame_data = frame_queue.get(timeout=0.1)
            np_arr = np.frombuffer(frame_data, dtype=np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
            if frame is not None:
                processed_frame = apply_filter(frame, FILTER_MODE)
                cv2.imshow("ESP32-CAM Stream (Filtered)", processed_frame)
            else:
                print("[-] Warning: Failed to decode JPEG frame.")
                
        except queue.Empty:
            pass

        if cv2.waitKey(1) & 0xFF == ord('q'):
            print("[*] 'q' pressed. Exiting... Goodbye!")
            break

    cv2.destroyAllWindows()
