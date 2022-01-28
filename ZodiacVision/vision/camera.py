from threading import Thread

import numpy as np
import time

from . import hwcheck
import yaml
import cv2
import math

gstreamer = True
class Camera:
    def __init__(self):
        self.hw = hwcheck.HWCheck()
        self.frame = np.zeros((100,100,3), np.uint8)
        self.isZed = self.hw.CheckZed()
        print(self.isZed)
        path = 'vision.yml'
        with open(path, 'r') as file:
            data = yaml.safe_load(file)
        if self.isZed:
            import pyzed.sl as sl
            # Set configuration parameters
            self.zed = sl.Camera()
            self.point_cloud = sl.Mat()
            # Set configuration parameters
            self.init_params = sl.InitParameters()
            self.init_params.depth_mode = sl.DEPTH_MODE.PERFORMANCE  # Use PERFORMANCE depth mode
            self.init_params.coordinate_units = sl.UNIT.INCH  # Use milliliter units (for depth measurements)
            self.init_params.camera_resolution = sl.RESOLUTION.VGA
            self.init_params.camera_fps = 100
            self.init_params.depth_maximum_distance = 400
            # Open the camera
            err = self.zed.open(self.init_params)
            print(err)
            if err != sl.ERROR_CODE.SUCCESS:
                exit(-1)
            self.image = sl.Mat()
            self.zed.set_camera_settings(sl.VIDEO_SETTINGS.EXPOSURE, data['camera']['exposure'])
            self.runtime_parameters = sl.RuntimeParameters()
        else:
            if not gstreamer:
                self.cap = cv2.VideoCapture(0)
            else:
                self.cap = cv2.VideoCapture("shmsrc socket-path=/tmp/foo ! video/x-raw, format=BGRx, width=672, height=376, framerate=100/1 ! videoconvert ! appsink drop=1", cv2.CAP_GSTREAMER)
        self.thread = Thread(target=self.read, args=())
        self.thread.daemon = True
        self.thread.start()
    def read(self):
        if self.isZed:
            import pyzed.sl as sl
            while True:
                if self.zed.grab(self.runtime_parameters) == sl.ERROR_CODE.SUCCESS:
                    # A new image is available if grab() returns SUCCESS
                    self.zed.retrieve_image(self.image, sl.VIEW.RIGHT)
                    self.zed.retrieve_measure(self.point_cloud, sl.MEASURE.XYZRGBA)
                    frame = self.image.get_data()
                    print(time.time())
                    self.frame = frame
        else:
            while True:
                _, frame = self.cap.read()
                self.frame = frame

    def updateExposure(self, net):
        if self.isZed:
            import pyzed.sl as sl
            self.zed.set_camera_settings(sl.VIDEO_SETTINGS.EXPOSURE, net.yml_data['camera']['exposure'])
    def findTargetInfo(self, nt, cx, cy):
        if self.isZed:
            import pyzed.sl as sl
            err, point3D = self.point_cloud.get_value(cx, cy)
            distance = math.sqrt(point3D[0] * point3D[0] + point3D[1] * point3D[1] + point3D[2] * point3D[2])
            if math.isnan(distance) or math.isinf(distance):
                nt.putValue('distance', -1)
            else:
                nt.putValue('distance', round(distance))
        else:
            nt.putValue('distance', -1)
