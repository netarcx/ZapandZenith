import socket
import sys
from vision import vision_frame_pb2
import google.protobuf.internal.decoder as decoder

address = ('127.0.0.1', 5802)
data = ''.join(sys.argv[1:])
with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.connect(address)
    while True:
        s.send(b'distance' + b'\n')

        # Length-delimited protobufs sent over socket
        # Format:
        # length [data bytes] length [data bytes] ...

        # Read length as varint
        buffer = s.recv(2048)
        (size, position) = decoder._DecodeVarint(buffer, 0);
        frame = vision_frame_pb2.VisionFrame()
        frame.ParseFromString(buffer[position:position+size])
        print(frame)
