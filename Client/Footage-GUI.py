import cv2
import zmq
import base64
import numpy as np
import time
from ip_utils import get_ip_address

# Import IP address from IP.txt
ip_adr = get_ip_address()
print(f"Connecting to video stream @ {ip_adr}:5555")

context = zmq.Context()
footage_socket = context.socket(zmq.SUB)  # Changed from PAIR to SUB

# Set high-water mark to prevent buffering old frames
footage_socket.setsockopt(zmq.RCVHWM, 1)

footage_socket.connect(f'tcp://{ip_adr}:5555')
footage_socket.setsockopt_string(zmq.SUBSCRIBE, '')  # Subscribe to all messages

# Give ZMQ time to establish subscription (avoid "slow joiner" problem)
time.sleep(0.2)
print("Connected to video stream, waiting for frames...")

cv2.namedWindow('Stream',flags=cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO)
cv2.resizeWindow('Stream',width=640,height=480)
font = cv2.FONT_HERSHEY_SIMPLEX

print("Starting video receive loop...")
frame_count = 0
try:
	while True:
		try:
			# Check if message is available (non-blocking check)
			if footage_socket.poll(timeout=1000):  # Wait up to 1 second
				frame = footage_socket.recv_string()
				frame_count += 1
				if frame_count % 30 == 0:  # Log every 30 frames
					print(f"Received frame {frame_count}")

				img = base64.b64decode(frame)
				npimg = np.frombuffer(img, dtype=np.uint8)
				source = cv2.imdecode(npimg, 1)
				cv2.imshow("Stream", source)
				cv2.waitKey(1)
			else:
				print("⚠ Timeout waiting for video frame")
		except zmq.error.Again:
			print("⚠ No message available (ZMQ Again)")
			time.sleep(0.1)
		except Exception as e:
			print(f"✗ Error in video receive loop: {e}")
			import traceback
			traceback.print_exc()
			time.sleep(0.1)
except KeyboardInterrupt:
	print("\n✓ Video stream stopped by user")
finally:
	cv2.destroyAllWindows()
	footage_socket.close()
	context.term()
	print("✓ Cleanup complete")
