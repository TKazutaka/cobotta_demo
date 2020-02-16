import socket
import struct

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.connect(('192.168.1.100', 50000))
    while True:
        data = s.recv(24)
        if not data:
            break
        print(len(data))
        res = struct.unpack('<ddd', data)
        print(res)
    s.close()
