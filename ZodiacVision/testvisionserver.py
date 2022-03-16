import random
import threading

from vision import visionserver
import yaml

path = 'vision.yml'
with open(path, 'r') as file:
    data = yaml.safe_load(file)
print(data)
vs = visionserver.ThreadedVisionServer('', 5802, path, data)
server_thread = threading.Thread(target=vs.listen)
# Exit the server thread when the main thread terminates
server_thread.daemon = True
server_thread.start()

while True:
    vs.updateSavedCenter(str(random.randint(-20, 20)), str(random.randint(-20, 20)))
    vs.updateSavedDistance(str(random.randint(0, 100)))

