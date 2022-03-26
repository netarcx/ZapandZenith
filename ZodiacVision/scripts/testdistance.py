import pyzed.sl as sl
import cv2
import math

zed = sl.Camera()
point_cloud = sl.Mat()
# Set configuration parameters
init_params = sl.InitParameters()
init_params.depth_mode = sl.DEPTH_MODE.ULTRA  # Use PERFORMANCE depth mode
init_params.coordinate_units = sl.UNIT.INCH  # Use milliliter units (for depth measurements)
init_params.camera_resolution = sl.RESOLUTION.VGA
init_params.camera_fps = 100
init_params.depth_maximum_distance = 400
# Open the camera
err = zed.open(init_params)
if err != sl.ERROR_CODE.SUCCESS:
    exit(-1)
image = sl.Mat()
runtime_parameters = sl.RuntimeParameters()

while True:
    if zed.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS:
        zed.retrieve_measure(point_cloud, sl.MEASURE.XYZRGBA)
        # A new image is available if grab() returns SUCCESS
        zed.retrieve_image(image, sl.VIEW.RIGHT)  # Retrieve the left image
        frame = image.get_data()
        err, point3D = point_cloud.get_value(336, 376)
        distance = math.sqrt(point3D[0] * point3D[0] + point3D[1] * point3D[1] + point3D[2] * point3D[2])
        if math.isnan(distance) or math.isinf(distance):
            continue
        print(distance)
        drawn_frame = cv2.circle(frame, (int(336), int(376)), radius=0, color=(255, 0, 255), thickness=5)
        cv2.imshow('frame', drawn_frame)
        cv2.waitKey(1)
