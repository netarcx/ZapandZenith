import cv2
import yaml
import socketserver

from vision import networktables, camera, detect, fps
import time
path = 'vision.yml'
with open(path, 'r') as file:
    data = yaml.safe_load(file)
# print(data)
net = networktables.NetworkTables(data, path)
net.setupCalib()

visionCamera = camera.Camera()

detector = detect.Detector(net, visionCamera)
#streamer = stream.Streamer(data['stream']['port'])
width = int(net.yml_data['stream']['line'])
fpsCounter = fps.FPS()

    # stream_image = fps.putIterationsPerSec(stream_image, fpsCounter.fps())
    #
    # if net.line:
    #     width = int(net.yml_data['stream']['line'])
    #     net.line = False
    # stream_image = cv2.line(stream_image, (width, 0), (width, int(stream_image.shape[0])), (0, 255, 0), 3)
    # if net.calib_camera:
    #     streamer.write(mask)
    # else:
    #     streamer.write(stream_image)

class VisionDispatcher(socketserver.StreamRequestHandler):
    def handle(self):
        # print("Got connection from", self.client_address)
        try:
            for line in self.rfile:
            # print("Got line:", line)
                self.handleOne(line.rstrip())
        except:
            # this throws when the client disconnects
            print("Client disconnected")

    def handleOne(self, msg):
        mask = self.getMask()
        center_x, center_y, distance = -1, -1, -1
        if mask.all() != -1: # if -1 - give them that, let the client handle
            contour, center_x, center_y = detector.findTarget(mask)
            distance = visionCamera.findTargetDistance(center_x, center_y)

        self.dispatchResponse(msg, center_x, center_y, distance)
        
        fpsCounter.update()
        fpsCounter.stop()

    def reply(self, msg):
        try: # crashes if client disconnects
            self.request.sendall((msg + "\n").encode())
        except: 
            return None

    def dispatchResponse(self, msg, center_x, center_y, distance):
        center_x = str(center_x)
        center_y = str(center_y)
        distance = str(distance)

        if msg == b'distance':
            self.reply(("distance|"+distance+"\n"))
        elif msg == b'center_x':
            self.reply("center_x|"+center_x+"\n")
        elif msg == b'cxy':
            self.reply(""+center_x+","+center_y)
        elif msg == b'cxd':
            self.reply(""+center_x+","+center_y+","+distance)
        else:
            self.reply('error|invalid request')
        
    def getMask(self):
        fpsCounter.reset()
        fpsCounter.start()
        if net.update_exposure:
            visionCamera.updateExposure(net)
            net.update_exposure = False
        # A new image is available if grab() returns SUCCESS
        frame = visionCamera.read()
        if net.vision_use:
            cv2.imwrite('/home/jetson/ZodiacVision/capture/' + time.strftime("%Y%m%d-%H%M%S") + '.png', frame)
            net.vision_use = False
        return detector.preProcessFrame(frame)


port = net.yml_data['main']['port']

with socketserver.ThreadingTCPServer(('localhost', port), VisionDispatcher) as server:
    server.serve_forever()
