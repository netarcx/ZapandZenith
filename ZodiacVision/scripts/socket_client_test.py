import socket
import sys

address = ('localhost', 5802)
data = ''.join(sys.argv[1:])
with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.connect(address)
    s.send(b'calib|EXPOSURE|10')
    received = s.recv(20000)
    print(received)