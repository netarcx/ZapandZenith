import socket
import sys

address = ('localhost', 5802)
data = ''.join(sys.argv[1:])
while True:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.connect(address)
        s.sendall(bytes(data + input('Enter Command: '), "utf-8"))
        received = str(s.recv(1024), "utf-8")
        print(received)
