import socket
import time

URSIM_IP = "172.17.0.2"  # Update this to match your URSim IP
PORT = 30002  # Primary interface (real-time)
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((URSIM_IP, PORT))

# Move to a joint position (in radians)
move_script = """
def move():
  movej([0.0, -1.57, 1.57, 0.0, 0.0, 0.0], a=1.4, v=1.05)
end
move()
"""
s.send((move_script + "\n").encode("utf-8"))

time.sleep(2)  # Allow motion to start
s.close()
