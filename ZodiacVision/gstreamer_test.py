import cv2

cap = cv2.VideoCapture("""gst-zed-rtsp-launch --gst-debug=1 -a 10.18.16.16 -p 5600 zedsrc camera-resolution=3 camera-fps=100 ! tee name=t \
t. ! queue ! videoconvert ! 'video/x-raw, format=(string)I420' ! nvvidconv ! nvv4l2h265enc bitrate=720000 ! rtph265pay pt=96 name=pay0 \
t. ! queue ! videoconvert ! 'video/x-raw, format=(string)I420' ! nvvidconv ! 'video/x-raw(memory:NVMM)' ! nvvidconv ! video/x-raw,format=BGRx ! videoconvert ! video/x-raw,format=BGR ! appsink drop=1""")

if not cap.isOpened():
    print("Cannot capture from camera. Exiting.")
    quit()

while True:

    ret, frame = cap.read()
    if ret == False:
        break

    print('Got Frame')
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break