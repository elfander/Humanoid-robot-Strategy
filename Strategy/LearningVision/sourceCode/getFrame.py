import cv2
import time

# Path to video file
vid = cv2.VideoCapture('output.mp4')
# Used as counter variable
count = 0

while True:
	# Extract images
	ret, frame = vid.read()
	# end of frames
	if not ret:
		break
	gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
	cv2.imwrite("./Image/image_%d.jpg" % count, gray)
	print ('Create image_%d.jpg...' % count)
	count += 1
	time.sleep(1)
