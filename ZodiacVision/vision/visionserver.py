import socket
import threading
import yaml
from . import vision_frame_pb2
import google.protobuf.internal.encoder as encoder

class ThreadedVisionServer(object):
    def __init__(self, host, port, yml_path, yaml_data):
        self.host = host
        self.port = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((self.host, self.port))
        self.cx = '-1'
        self.cy = '-1'
        self.distance = '-1'
        self.yml_data = yaml_data
        self.yml_path = yml_path
        self.update_exposure = False
        self.calib_camera = False

    def listen(self):
        self.sock.listen(5)
        while True:
            client, address = self.sock.accept()
            client.settimeout(300)
            threading.Thread(target=self.listenToClient, args=(client, address)).start()

    def listenToClient(self, client, address):
        read = client.makefile('r')
        write = client.makefile('w')
        with client, read, write:
            while True:
                try:
                    data = read.readline()
                    if data:
                        # Set the response to echo back the recieved data
                        response = data.strip()
                        print(response)
                        self.dispatchResponse(client, response)
                except Exception as e:
                    print(e)
                    client.close()
                    return False

    def dispatchResponse(self, client, msg):
        if 'calib' in msg:
            print(str(msg))
            msg = str(msg).replace("'", "").split('|')
            self.calibChange(msg[1], msg[2])
        else:
            response_frame = vision_frame_pb2.VisionFrame()
            response_frame.distance = self.distance
            response_frame.center_x = self.cx
            response_frame.center_y = self.cy

            frame_bytes = response_frame.SerializeToString()  # technically serialized as byte array
            len_bytes = encoder._VarintBytes(len(frame_bytes))
            print(len(frame_bytes))
            client.sendall(len_bytes + frame_bytes)


    def updateSavedCenter(self, cx, cy):
        self.cx = cx
        self.cy = cy

    def updateSavedDistance(self, d):
        self.distance = d
    def calibChange(self, key, value):
        print(value)
        def dumpYML():
            with open(self.yml_path, "w") as f:
                yaml.dump(self.yml_data, f)
        # if key == "VISION":
        #     self.vision_use = value
        if key == "HMIN":
            value = float(value)
            self.yml_data['color']['lower']['H'] = value
            dumpYML()
        elif key == "SMIN":
            value = float(value)
            self.yml_data['color']['lower']['S'] = value
            dumpYML()
        elif key == "VMIN":
            value = float(value)
            self.yml_data['color']['lower']['V'] = value
            dumpYML()
        elif key == "HMAX":
            value = float(value)
            self.yml_data['color']['upper']['H'] = value
            dumpYML()
        elif key == "SMAX":
            value = float(value)
            self.yml_data['color']['upper']['S'] = value
            dumpYML()
        elif key == "VMAX":
            value = float(value)
            self.yml_data['color']['upper']['V'] = value
            dumpYML()
        elif key == "EXPS":
            value = float(value)
            self.yml_data['camera']['exposure'] = value
            dumpYML()
            self.update_exposure = True
        elif key == "LINE":
            self.yml_data['stream']['line'] = value
            dumpYML()
            self.line = True
        elif key == "CalibrationCamera":
            self.calib_camera = value
        print(self.yml_data)

