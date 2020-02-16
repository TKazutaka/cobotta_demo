import socket

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.connect(('192.168.0.100', 50000))
    while True:
        data = s.recv(1024)
        if not data:
            break
        print(data)
    s.close()
