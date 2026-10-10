import socket
import struct
import cv2
import numpy as np

# Network and Security Configurations
SERVER_IP = "0.0.0.0"       # Listen on all available network interfaces
SERVER_PORT = 1357
AUTH_PASSWORD = "pass187631"  # Must match the password set in ESP-IDF

# Initialize TCP Socket
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
# Allow immediate port reuse after server restart
server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
server_socket.bind((SERVER_IP, SERVER_PORT))
server_socket.listen(1)

print(f"[*] Python Server is listening on port {SERVER_PORT}...")

try:
    while True:
        conn, addr = server_socket.accept()
        print(f"[*] Incoming connection from: {addr}. Awaiting authentication...")
        
        try:
            # --- STEP 1: CLIENT AUTHENTICATION HANDSHAKE ---
            client_auth_data = b""
            # Read until newline character (\n) is received
            while b"\n" not in client_auth_data:
                packet = conn.recv(1024)
                if not packet: 
                    break
                client_auth_data += packet
                # Prevent buffer overflow attacks by limiting password length
                if len(client_auth_data) > 100: 
                    break 
            
            # Decode the raw string and strip trailing whitespace/newlines
            received_password = client_auth_data.decode('utf-8', errors='ignore').strip()
            
            if received_password == AUTH_PASSWORD:
                print(f"[+] Authentication SUCCESSFUL for client {addr}. Starting video stream...")
                conn.sendall(b"AUTH_OK\n")  # Signal client to start sending frames
            else:
                print(f"[-] Authentication FAILED for client {addr}. Incorrect password received: '{received_password}'")
                conn.sendall(b"AUTH_FAIL\n")
                conn.close()
                continue  # Reject client and wait for a new connection
                
            # --- STEP 2: VIDEO PACKET RECEPTION ---
            data = b""
            payload_size = struct.calcsize("<I")  # 4 bytes unsigned int (uint32_t from C)
            
            while True:
                # 2.1 Receive the 4-byte frame size header
                while len(data) < payload_size:
                    packet = conn.recv(4096)
                    if not packet: 
                        break
                    data += packet
                
                if len(data) < payload_size: 
                    break
                
                packed_msg_size = data[:payload_size]
                data = data[payload_size:]
                msg_size = struct.unpack("<I", packed_msg_size)[0]
                
                # 2.2 Receive the binary image payload based on the declared size
                while len(data) < msg_size:
                    packet = conn.recv(4096)
                    if not packet: 
                        break
                    data += packet
                    
                if len(data) < msg_size: 
                    break
                
                frame_data = data[:msg_size]
                data = data[msg_size:]
                
                # 2.3 Decode JPEG binary buffer into an OpenCV matrix
                np_arr = np.frombuffer(frame_data, dtype=np.uint8)
                frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                
                if frame is not None:
                    cv2.imshow("ESP32-CAM Stream (Authenticated)", frame)
                else:
                    print("[-] Warning: Failed to decode incoming JPEG frame data.")
                
                # Press 'q' on the keyboard inside the OpenCV window to exit
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    print("[*] 'q' pressed. Closing stream display.")
                    break
                    
        except Exception as e:
            print(f"[-] Connection error or unexpected disconnection: {e}")
        finally:
            conn.close()
            print(f"[*] Connection with client {addr} closed. Standing by for next device...")
            
finally:
    server_socket.close()
    cv2.destroyAllWindows()
    print("[*] Server shut down successfully.")
