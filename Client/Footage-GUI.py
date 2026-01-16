import cv2
import zmq
import base64
import numpy as np
from ip_utils import get_ip_address

# Import IP address from IP.txt
ip_adr = get_ip_address()
print(f"Connecting to video stream @ {ip_adr}:5555")

context = zmq.Context()
footage_socket = context.socket(zmq.PAIR)

# Die Client-Seite versucht zu binden (Server-Rolle), obwohl sie sich verbinden sollte (Client-Rolle).
#footage_socket.bind('tcp://*:5555')
footage_socket.connect(f'tcp://{ip_adr}:5555')

#footage_socket.setsockopt_string(zmq.SUBSCRIBE, '')
cv2.namedWindow('Stream',flags=cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO)
cv2.resizeWindow('Stream',width=640,height=480)
font = cv2.FONT_HERSHEY_SIMPLEX

while True:
	frame = footage_socket.recv_string()
	img = base64.b64decode(frame)
	npimg = np.frombuffer(img, dtype=np.uint8)
	source = cv2.imdecode(npimg, 1)
	cv2.imshow("Stream", source)
	cv2.waitKey(1)


	