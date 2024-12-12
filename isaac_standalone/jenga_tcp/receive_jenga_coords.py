import socket
import threading
import json

HOST = 'localhost'  # The server's hostname or IP address
PORT = 65433        # The port used by the server for sending pose data

def handle_client(conn, addr):
    print('Connected by', addr)
    try:
        with conn:
            while True:
                data = conn.recv(1024)
                if not data:
                    break
                analyze_data(data.decode())
    finally:
        print(f"Connection closed for {addr}")

# def has_fallen(rotation):
    # Assuming instability if rotation is above a certain threshold, for example, any non-identity quaternion
    # You can modify this logic based on your stability conditions
    # return rotation != [1.0, 0.0, 0.0, 0.0]

def analyze_data(data):
    try:
        # Decode the received JSON string
        decoded_data = json.loads(data)
        print("Received data:", decoded_data)

        # Iterate through each Jenga block and its data
        for block_path, block_info in decoded_data.items():
            position = block_info["position"]
            rotation = block_info["rotation"]
            print(f"Block {block_path} - Position: {position}, Rotation: {rotation}")

            # # Check if the block has fallen
            # if has_fallen(rotation):
            #     print(f"The structure at {block_path} has fallen or is unstable.")
            # else:
            #     print(f"The structure at {block_path} is stable.")
    
    except json.JSONDecodeError as e:
        print(f"Failed to decode JSON data: {e}")

def main():
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind((HOST, PORT))
        s.listen()
        print("Listening for incoming data...")
        
        while True:  # Continuously accept new connections
            conn, addr = s.accept()
            client_thread = threading.Thread(target=handle_client, args=(conn, addr))
            client_thread.start()  # Start a new thread for each connection

if __name__ == '__main__':
    main()
