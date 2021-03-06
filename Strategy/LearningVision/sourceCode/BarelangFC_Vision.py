import os
import numpy as np
import cv2
import socket
import time

host = 'localhost'
port = 2000

# Default filename to save all data
IMAGE_POSITIVE_PATH = "../Vision/Positif_example/images_{:01d}.png"
IMAGE_NEGATIVE_PATH = "../Vision/Negatif_example/image_{:01d}.png"

# Classifier
bola_cascade = cv2.CascadeClassifier('cascade_10000.xml')
gawang_cascade = cv2.CascadeClassifier('Gawang.xml')

# Default filename to save all data
settingValueFilename='BarelangFC-SettingValue.csv'

# Global variable for thresholding
cameraSetting = np.zeros(10, dtype=int)
# Field
lowerFieldGr = np.zeros(3, dtype=int)
upperFieldGr = 255 * np.ones(3, dtype=int)
edFieldGr = np.zeros(2, dtype=int)
# Ball
BallParameter = np.zeros(4, dtype=int)
# Goal
GoalParameter = np.zeros(4, dtype=int)

def nothing(x):
    pass

def showHelp():
        print ('------------------BarelangFC-Vision-------------------------')
        print ('Parse Field -------------------------------------------- [F]')
        print ('Ball Parameter ----------------------------------------- [B]')
        print ('Goal Parameter ----------------------------------------- [G]')
        print ('Capture Positive Image --------------------------------- [c]')
        print ('Capture Negative Image --------------------------------- [v]')
        print ('Save --------------------------------------------------- [S]')
        print ('Load --------------------------------------------------- [L]')
        print ('Destroy All Windows ------------------------------------ [R]')
        print ('Help --------------------------------------------------- [Z]')
        print ('Exit BarelangFC-Vision --------------------------------- [X]')
	print ('\n')

def createTrackBar(mode):
        cv2.namedWindow('Control')
        if mode == 1: #field
		cv2.createTrackbar('HMax','Control',255,255,nothing)
                cv2.createTrackbar('HMin','Control',0,255,nothing)
                cv2.createTrackbar('SMax','Control',255,255,nothing)
                cv2.createTrackbar('SMin','Control',0,255,nothing)
                cv2.createTrackbar('VMax','Control',255,255,nothing)
                cv2.createTrackbar('VMin','Control',0,255,nothing)
                cv2.createTrackbar('Erode','Control',0,10,nothing)
                cv2.createTrackbar('Dilate','Control',0,100,nothing)
        elif mode == 2: #ball
		cv2.createTrackbar('minNeighbors','Control',0,36,nothing)
		cv2.createTrackbar('flags','Control',0,16,nothing)
		cv2.createTrackbar('minSize','Control',0,255,nothing)
		cv2.createTrackbar('maxSize','Control',0,255,nothing)
        elif mode == 4: #goal
		cv2.createTrackbar('minNeighbors','Control',0,36,nothing)
		cv2.createTrackbar('flags','Control',0,16,nothing)
		cv2.createTrackbar('minSize','Control',0,255,nothing)
		cv2.createTrackbar('maxSize','Control',0,255,nothing)
        elif mode == 5: #setCameraParameter
		cv2.createTrackbar('brightness','Control',128,255,nothing)
                cv2.createTrackbar('contrast','Control',128,255,nothing)
                cv2.createTrackbar('saturation','Control',128,255,nothing)
                cv2.createTrackbar('white_balance_temperature_auto','Control',1,1,nothing)
                cv2.createTrackbar('white_balance_temperature','Control',4000,7500,nothing)
                cv2.createTrackbar('sharpness','Control',128,255,nothing)
                cv2.createTrackbar('exposure_auto','Control',3,3,nothing)
                cv2.createTrackbar('exposure_absolute','Control',250,2047,nothing)
                cv2.createTrackbar('exposure_auto_priority','Control',0,1,nothing)
                cv2.createTrackbar('focus_auto','Control',1,1,nothing)

def loadTrackBar(mode):
        if mode == 1: #Field Load Control
                cv2.setTrackbarPos('HMin','Control',lowerFieldGr[0])
                cv2.setTrackbarPos('SMin','Control',lowerFieldGr[1])
                cv2.setTrackbarPos('VMin','Control',lowerFieldGr[2])
                cv2.setTrackbarPos('HMax','Control',upperFieldGr[0])
                cv2.setTrackbarPos('SMax','Control',upperFieldGr[1])
                cv2.setTrackbarPos('VMax','Control',upperFieldGr[2])
                cv2.setTrackbarPos('Erode','Control',edFieldGr[0])
                cv2.setTrackbarPos('Dilate','Control',edFieldGr[1])
        elif mode == 2: #ball Load Control
                cv2.setTrackbarPos('minNeighbors','Control',BallParameter[0])
                cv2.setTrackbarPos('flags','Control',BallParameter[1])
                cv2.setTrackbarPos('minSize','Control',BallParameter[2])
                cv2.setTrackbarPos('maxSize','Control',BallParameter[3])
        elif mode == 4: #goal Load Control
                cv2.setTrackbarPos('minNeighbors','Control',GoalParameter[0])
                cv2.setTrackbarPos('flags','Control',GoalParameter[1])
                cv2.setTrackbarPos('minSize','Control',GoalParameter[2])
                cv2.setTrackbarPos('maxSize','Control',GoalParameter[3])
        elif mode == 5: #Camera Parameter
		cv2.setTrackbarPos('brightness', 'Control', cameraSetting[0])
                cv2.setTrackbarPos('contrast', 'Control', cameraSetting[1])
                cv2.setTrackbarPos('saturation', 'Control', cameraSetting[2])
                cv2.setTrackbarPos('white_balance_temperature_auto', 'Control', cameraSetting[3])
                cv2.setTrackbarPos('white_balance_temperature', 'Control', cameraSetting[4])
                cv2.setTrackbarPos('sharpness', 'Control', cameraSetting[5])
                cv2.setTrackbarPos('exposure_auto', 'Control', cameraSetting[6])
                cv2.setTrackbarPos('exposure_absolute', 'Control', cameraSetting[7])
                cv2.setTrackbarPos('exposure_auto_priority', 'Control', cameraSetting[8])
                cv2.setTrackbarPos('focus_auto', 'Control', cameraSetting[9])

def saveConfig():
	npSettingValue = np.zeros(26, dtype=int)
        #Nilai Parameter Field
        npSettingValue[0]= lowerFieldGr[0]
        npSettingValue[1]= lowerFieldGr[1]
        npSettingValue[2]= lowerFieldGr[2]
        npSettingValue[3]= upperFieldGr[0]
        npSettingValue[4]= upperFieldGr[1]
        npSettingValue[5]= upperFieldGr[2]
        npSettingValue[6]= edFieldGr[0]
        npSettingValue[7]= edFieldGr[1]
	#Nilai Parameter Bola
        npSettingValue[8]= BallParameter[0]
        npSettingValue[9]= BallParameter[1]
        npSettingValue[10]= BallParameter[2]
        npSettingValue[11]= BallParameter[3]
	#Nilai Parameter Camera
	npSettingValue[12] = cameraSetting[0]
        npSettingValue[13] = cameraSetting[1]
        npSettingValue[14] = cameraSetting[2]
        npSettingValue[15] = cameraSetting[3]
        npSettingValue[16] = cameraSetting[4]
        npSettingValue[17] = cameraSetting[5]
        npSettingValue[18] = cameraSetting[6]
        npSettingValue[19] = cameraSetting[7]
        npSettingValue[20] = cameraSetting[8]
        npSettingValue[21] = cameraSetting[9]
	#Nilai Parameter Gawang
        npSettingValue[22]= GoalParameter[0]
        npSettingValue[23]= GoalParameter[1]
        npSettingValue[24]= GoalParameter[2]
        npSettingValue[25]= GoalParameter[3]

        npSettingValue=np.reshape(npSettingValue,(1,26))
        headerLabel= 'F HMin, F SMin, VMin, F HMax, F SMax, F VMax, F Erode, F Dilate, B minNeighbors, B flags, B minSize, B maxSize, Brightness, Contrast, Saturation, WB Temp Auto, WB Temp, Sharpness, Exp Auto, Exp Abs, Exp Auto Priority, Focus Auto, G minNeighbors, G flags, G minSize, G maxSize'
        np.savetxt(settingValueFilename, npSettingValue, fmt='%d', delimiter = ',', header= headerLabel)
        print ("Setting Parameter Saved")

def loadConfig():
        csvSettingValue = np.genfromtxt(settingValueFilename,dtype=int, delimiter=',', skip_header=True)
        print (csvSettingValue)
        #Field
        lowerFieldGr[0] = csvSettingValue[0]
        lowerFieldGr[1] = csvSettingValue[1]
        lowerFieldGr[2] = csvSettingValue[2]
        upperFieldGr[0] = csvSettingValue[3]
        upperFieldGr[1] = csvSettingValue[4]
        upperFieldGr[2] = csvSettingValue[5]
        edFieldGr[0] = csvSettingValue[6]
        edFieldGr[1] = csvSettingValue[7]

	#Ball
        BallParameter[0] = csvSettingValue[8]
        BallParameter[1] = csvSettingValue[9]
        BallParameter[2] = csvSettingValue[10]
        BallParameter[3] = csvSettingValue[11]

	#Camera Parameter
	cameraSetting[0] = csvSettingValue[12] #128     # Brightness
        cameraSetting[1] = csvSettingValue[13] #128     # Contrast
        cameraSetting[2] = csvSettingValue[14] #128     # Saturation
        cameraSetting[3] = csvSettingValue[15] #0       # White_balance_temperature_auto
        cameraSetting[4] = csvSettingValue[16] #5000    # White_balance_temperature
        cameraSetting[5] = csvSettingValue[17] #128     # Sharpness
        cameraSetting[6] = csvSettingValue[18] #1       # Exposure_auto
        cameraSetting[7] = csvSettingValue[19] #312     # Exposure_absolute
        cameraSetting[8] = csvSettingValue[20] #0       # Exposure_auto_priority
        cameraSetting[9] = csvSettingValue[21] #0       # Focus_auto

	#Goal
        GoalParameter[0] = csvSettingValue[22]
        GoalParameter[1] = csvSettingValue[23]
        GoalParameter[2] = csvSettingValue[24]
        GoalParameter[3] = csvSettingValue[25]
        print ("Setting Parameter Loaded")

def setCameraParameter():
        #print "Before Set Camera Setting Parameter"
        #os.system("v4l2-ctl --list-ctrls")

        Brightness = "v4l2-ctl --set-ctrl brightness={}".format(cameraSetting[0])
        Contrast = "v4l2-ctl --set-ctrl contrast={}".format(cameraSetting[1])
        Saturation = "v4l2-ctl --set-ctrl saturation={}".format(cameraSetting[2])
        White_balance_temperature_auto = "v4l2-ctl --set-ctrl white_balance_temperature_auto={}".format(cameraSetting[3])
        White_balance_temperature = "v4l2-ctl --set-ctrl white_balance_temperature={}".format(cameraSetting[4])
        Sharpness = "v4l2-ctl --set-ctrl sharpness={}".format(cameraSetting[5])
        Exposure_auto = "v4l2-ctl --set-ctrl exposure_auto={}".format(cameraSetting[6])
        Exposure_absolute = "v4l2-ctl --set-ctrl exposure_absolute={}".format(cameraSetting[7])
        Exposure_auto_priority = "v4l2-ctl --set-ctrl exposure_auto_priority={}".format(cameraSetting[8])
        Focus_auto = "v4l2-ctl --set-ctrl focus_auto={}".format(cameraSetting[9])

        os.system(Brightness)
        os.system(Contrast)
        os.system(Saturation)
        os.system(White_balance_temperature_auto)
        os.system(White_balance_temperature)
        os.system(Sharpness)
        os.system(Exposure_auto)
        #os.system(Exposure_absolute)
        os.system(Exposure_auto_priority)
        os.system(Focus_auto)
        print ("v4l2 camera manual parameter override succesfully")

        #print "After Set Camera Setting Parameter"
        #os.system("v4l2-ctl --list-ctrls")

def distance_to_camera(knownWidth, focalLength, perWidth):
        try:
                return int((knownWidth * focalLength) / perWidth)
        except:
                return 0

def main():
	# Connect to localhost
	try:
		s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
	except socket.error:
		print 'Failed to create socket'
		sys.exit()

    	# create a video capture
    	cap = cv2.VideoCapture(0)
    	image_width = 640
    	image_height = 480

    	#init calibration single camera
	#Ball
    	B_KNOWN_PIXEL = 115 #lebar pixel without calibration
    	B_KNOWN_DISTANCE = 50 #jarak cm
    	B_KNOWN_WIDTH = 14 #diameter cm
    	B_focalLength = (B_KNOWN_PIXEL * B_KNOWN_DISTANCE) / B_KNOWN_WIDTH
	#Field
	G_KNOWN_PIXEL = 35 #lebar pixel without calibration
        G_KNOWN_DISTANCE = 450 #jarak cm
        G_KNOWN_HEIGHT = 14 #180 #tinggi cm
        G_focalLength = (G_KNOWN_PIXEL * G_KNOWN_DISTANCE) / G_KNOWN_HEIGHT

    	frameIdPOS = 0 #data gambar terakhir di folder Positive_example
    	frameIdNEG = 0 #data gambar terakhir di folder Negative_example
    	clear = 0

    	# Image yang akan ditampilkan
    	imageToDisplay = 0
    	kernel = cv2.getStructuringElement(cv2.MORPH_RECT,(2,2))
	fps = 30

    	showHelp()
    	loadConfig()
    	os.system("v4l2-ctl --set-ctrl exposure_auto={}".format(3))

    	while(True):
    		# Capture frame-by-frame
    		ret, frame = cap.read()
		start = time.time() # Start time
    		rgbImage = cv2.resize(frame, (image_width, image_height),interpolation = cv2.INTER_AREA)

    		fieldMask = np.zeros(rgbImage.shape[:2], np.uint8) #hitam
    		notFieldMask = 255 * np.ones(rgbImage.shape[:2], np.uint8) #putih

    		# Color Convertion
    		modRgbImage = rgbImage.copy()
		grayImage = cv2.cvtColor(rgbImage, cv2.COLOR_BGR2GRAY)
    		yuvImage = cv2.cvtColor(rgbImage, cv2.COLOR_BGR2YUV)
    		#hsvImage = cv2.cvtColor(rgbImage, cv2.COLOR_BGR2HSV)

    		# Field convertion color
    		#blurRgbImage = rgbImage.copy()
    		#blurRgbImage = cv2.GaussianBlur(rgbImage,(5,5),0)
    		#blurRgbImage = cv2.GaussianBlur(rgbImage,(3,3),0)
    		#blurGrayImage = cv2.cvtColor(blurRgbImage, cv2.COLOR_BGR2GRAY)
    		#blurLabImage = cv2.cvtColor(blurRgbImage,cv2.COLOR_BGR2LAB)

    		# =============================================================== Field ===========================================================================================
    		#Field Green Color Filter
    		fieldGrBin = cv2.inRange(yuvImage, lowerFieldGr, upperFieldGr)
    		fieldGrBinErode = cv2.erode(fieldGrBin, kernel, iterations = edFieldGr[0])
    		fieldGrBinFinal = cv2.dilate(fieldGrBinErode, kernel, iterations = edFieldGr[1])

    		#Field Contour Detection
    		ret, listFieldContours, ret = cv2.findContours(fieldGrBinFinal.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    		if len(listFieldContours) > 0:
              		fieldContours = sorted(listFieldContours, key=cv2.contourArea, reverse = True)[:1]
              		fieldContours = max(listFieldContours, key=cv2.contourArea)
              		hull = cv2.convexHull(fieldContours)

              		cv2.drawContours(fieldMask, [hull], 0, 255, cv2.FILLED, offset=(0,0))
			cv2.drawContours(notFieldMask, [hull], -1, 0, -1)
	      		if clear == 0:
	            		cv2.drawContours(modRgbImage, [hull], 0, (255,255,0), 2, offset=(0,0))

    		# =============================================================== Ball ============================================================================================
    		# Initialize to default
    		Ball_x = -1
    		Ball_y = -1
    		Ball_distance = -1

		BnewImage = cv2.bitwise_and(grayImage, fieldMask)
                ballImage = cv2.resize(BnewImage, (320, 240),interpolation = cv2.INTER_AREA)
    		bola = bola_cascade.detectMultiScale(ballImage, 1.3, BallParameter[0], BallParameter[1], minSize=(BallParameter[2],BallParameter[2]), maxSize=(BallParameter[3],BallParameter[3]))

    		for (x,y,w,h) in bola:
			x = x * 2
                        y = y * 2
                        w = w * 2
                        h = h * 2
	     		Ball_x = (x + w + x ) / 2
	      		Ball_y = (y + h + y ) / 2
	      		Ball_distance = distance_to_camera(B_KNOWN_WIDTH, B_focalLength, w)

	      		if clear == 0:
	            		cv2.circle(modRgbImage, (Ball_x,Ball_y), 2, (0,0,255), -1)
	            		cv2.circle(modRgbImage, (Ball_x,Ball_y), (Ball_x - x), (255,0,255), 2, 8, 0)

    		# =============================================================== Goal ============================================================================================
    		# Initialize to default
		Goal_x = -1
		Goal_y = -1
    		GoalLeft_x = -1
    		GoalLeft_y = -1
    		GoalRight_x = -1
    		GoalRight_y = -1
    		GoalCenter_x = -1
    		GoalCenter_y = -1
		GoalLeftDistance = -1
		GoalRightDistance = -1

		GnewImage = cv2.bitwise_and(grayImage, notFieldMask)
                goalImage = cv2.resize(GnewImage, (320, 240),interpolation = cv2.INTER_AREA)
		gawang = gawang_cascade.detectMultiScale(goalImage, 1.3, GoalParameter[0], GoalParameter[1], minSize=(GoalParameter[2],GoalParameter[2]), maxSize=(GoalParameter[3],GoalParameter[3]))

		if (len(gawang) == 2):
			x1 = gawang[0,0]
			y1 = gawang[0,1]
			w1 = gawang[0,2]
			h1 = gawang[0,3]
			x2 = gawang[1,0]
			y2 = gawang[1,1]
			w2 = gawang[1,2]
			h2 = gawang[1,3]

			if x1 < x2:
				GoalLeft_x = (x1 + w1 + x1) / 2
				GoalLeft_y = (y1 + h1 + y1) / 2
				GoalRight_x = (x2 + w2 + x2) / 2
				GoalRight_y = (y2 + h2 + y2) / 2
				GoalCenter_x = (GoalLeft_x + GoalRight_x) / 2
				GoalCenter_y = (GoalLeft_y + GoalRight_y) / 2
				GoalLeftDistance = distance_to_camera(G_KNOWN_HEIGHT, G_focalLength, w1)
				GoalRightDistance = distance_to_camera(G_KNOWN_HEIGHT, G_focalLength, w2)

		    		if clear == 0:
					cv2.circle(modRgbImage, (GoalLeft_x,GoalLeft_y), 3, (0,0,255), -1) #merah
					cv2.rectangle(modRgbImage, (x1,y1), (x1+w1,y1+h1), (0,0,255), 2) #merah
       					cv2.circle(modRgbImage, (GoalRight_x,GoalRight_y), 3, (255,0,0), -1) #biru
					cv2.rectangle(modRgbImage, (x2,y2), (x2+w2,y2+h2), (255,0,0), 2) #biru
       					cv2.circle(modRgbImage, (GoalCenter_x,GoalCenter_y), 3, (0,255,255), -1) #coklat
			elif x1 > x2:
				GoalRight_x = (x1 + w1 + x1) / 2
				GoalRight_y = (y1 + h1 + y1) / 2
				GoalLeft_x = (x2 + w2 + x2) / 2
				GoalLeft_y = (y2 + h2 + y2) / 2
				GoalCenter_x = (GoalLeft_x + GoalRight_x) / 2
				GoalCenter_y = (GoalLeft_y + GoalRight_y) / 2
				GoalLeftDistance = distance_to_camera(G_KNOWN_HEIGHT, G_focalLength, w2)
				GoalRightDistance = distance_to_camera(G_KNOWN_HEIGHT, G_focalLength, w1)

		    		if clear == 0:
       					cv2.circle(modRgbImage, (GoalRight_x,GoalRight_y), 3, (255,0,0), -1) #biru
					cv2.rectangle(modRgbImage, (x1,y1), (x1+w1,y1+h1), (255,0,0), 2) #biru
					cv2.circle(modRgbImage, (GoalLeft_x,GoalLeft_y), 3, (0,0,255), -1) #merah
					cv2.rectangle(modRgbImage, (x2,y2), (x2+w2,y2+h2), (0,0,255), 2) #merah
       					cv2.circle(modRgbImage, (GoalCenter_x,GoalCenter_y), 3, (0,255,255), -1) #coklat
		elif (len(gawang) == 1):
	    		for (x,y,w,h) in gawang:
				x = x * 2
	                        y = y * 2
	                        w = w * 2
        	                h = h * 2
				Goal_x = (x + w + x) / 2
				Goal_y = (y + h + y) / 2
				if clear == 0:
       					cv2.circle(modRgbImage, (Goal_x,Goal_y), 3, (0,255,0), -1)
					cv2.rectangle(modRgbImage, (x,y), (x+w,y+h), (0,255,0), 2)

    		#==================================================================================================================================================================

    		# Send detection result to localhost
    		try:
	     		#s.flush()
              		visionMessage = '%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d'%(Ball_x, Ball_y, 0, 0, Ball_distance, GoalCenter_x, GoalCenter_y, 0, 0, 0, GoalLeftDistance, GoalRightDistance)
              		s.sendto(visionMessage, (host, port))
    		except socket.error:
	      		#print 'Error Code : ' + str(msg[0]) + ' Message ' + msg[1]
              		sys.exit()

    		# Print Value to Frame
    		font = cv2.FONT_HERSHEY_SIMPLEX
		textLine0 = '%d FPS'%(fps)
    		textLine1 = 'Ball -> X = %d, Y = %d, D = %dcm'%(Ball_x, Ball_y, Ball_distance)
    		textLine2 = 'Goal -> X = %d, Y = %d, LD = %dcm, RD = %dcm'%(GoalCenter_x, GoalCenter_y, GoalLeftDistance, GoalRightDistance)
    		if clear == 0:
			cv2.putText(modRgbImage, textLine0, (10,20), font, 0.6, (255,0,0), 1, cv2.LINE_AA)
	      		cv2.putText(modRgbImage, textLine1, (10,440), font, 0.5, (0,0,255), 1, cv2.LINE_AA)
	      		cv2.putText(modRgbImage, textLine2, (10,460), font, 0.5, (0,0,255), 1, cv2.LINE_AA)

    		# image to display
    		if imageToDisplay == 1: #field
	      		lowerFieldGr[0] = cv2.getTrackbarPos('HMin','Control')
              		lowerFieldGr[1] = cv2.getTrackbarPos('SMin','Control')
              		lowerFieldGr[2] = cv2.getTrackbarPos('VMin','Control')
              		upperFieldGr[0] = cv2.getTrackbarPos('HMax','Control')
              		upperFieldGr[1] = cv2.getTrackbarPos('SMax','Control')
              		upperFieldGr[2] = cv2.getTrackbarPos('VMax','Control')
              		edFieldGr[0] = cv2.getTrackbarPos('Erode','Control')
              		edFieldGr[1] = cv2.getTrackbarPos('Dilate','Control')
              		cv2.imshow("Barelang Vision", modRgbImage)
              		#cv2.imshow("Field Blur RGB", blurRgbImage)
              		cv2.imshow("Field Binary Image", fieldGrBinFinal)
              		#cv2.imshow("FieldMask", fieldMask)
              		#cv2.imshow("Not Field Mask", notFieldMask)
    		elif imageToDisplay == 2: #ball
  	      		BallParameter[0] = cv2.getTrackbarPos('minNeighbors','Control')
  	      		BallParameter[1] = cv2.getTrackbarPos('flags','Control')
    	      		BallParameter[2] = cv2.getTrackbarPos('minSize','Control')
    	      		BallParameter[3] = cv2.getTrackbarPos('maxSize','Control')
	      		cv2.imshow('Barelang Vision',modRgbImage)
    		elif imageToDisplay == 4: #Goal
  	      		GoalParameter[0] = cv2.getTrackbarPos('minNeighbors','Control')
  	      		GoalParameter[1] = cv2.getTrackbarPos('flags','Control')
    	      		GoalParameter[2] = cv2.getTrackbarPos('minSize','Control')
    	      		GoalParameter[3] = cv2.getTrackbarPos('maxSize','Control')
	      		cv2.imshow('Barelang Vision',modRgbImage)
    		elif imageToDisplay == 5: #SetCameraParameter
	      		cameraSetting[0] = cv2.getTrackbarPos('brightness','Control')
              		cameraSetting[1] = cv2.getTrackbarPos('contrast','Control')
              		cameraSetting[2] = cv2.getTrackbarPos('saturation','Control')
              		cameraSetting[3] = cv2.getTrackbarPos('white_balance_temperature_auto','Control')
              		cameraSetting[4] = cv2.getTrackbarPos('white_balance_temperature','Control')
              		if cameraSetting[4] < 2000: #fiture limit
	            		cv2.setTrackbarPos('white_balance_temperature','Control',2000)
              		cameraSetting[5] = cv2.getTrackbarPos('sharpness','Control')
              		cameraSetting[6] = cv2.getTrackbarPos('exposure_auto','Control')
              		cameraSetting[7] = cv2.getTrackbarPos('exposure_absolute','Control')
              		if cameraSetting[7] < 3: #fiture limit
          	    		cv2.setTrackbarPos('exposure_absolute','Control',3)
              		cameraSetting[8] = cv2.getTrackbarPos('exposure_auto_priority','Control')
              		cameraSetting[9] = cv2.getTrackbarPos('focus_auto','Control')
              		cv2.imshow("Barelang Vision", modRgbImage)
    		else:
			cv2.imshow('Barelang Vision',modRgbImage)
			#cv2.imshow('GRAY',grayImage)

    		# input Keyboard
    		keyBoard = cv2.waitKey(1)
    		if keyBoard == ord('f'):
	     		cv2.destroyAllWindows()
              		imageToDisplay = 1
              		createTrackBar(imageToDisplay)
              		loadTrackBar(imageToDisplay)
              		print('Setting Field Parameter')
    		elif keyBoard == ord('b'):
	      		cv2.destroyAllWindows()
              		imageToDisplay = 2
              		createTrackBar(imageToDisplay)
              		loadTrackBar(imageToDisplay)
              		print('Setting Ball Parameter')
    		elif keyBoard == ord('g'):
	      		cv2.destroyAllWindows()
              		imageToDisplay = 4
              		createTrackBar(imageToDisplay)
              		loadTrackBar(imageToDisplay)
              		print('Setting Goal Parameter')
    		elif keyBoard == ord('s'):
              		saveConfig()
			print 'Save Setting Value'
		elif keyBoard == ord('l'):
              		loadConfig()
              		loadTrackBar(imageToDisplay)
              		print ('Load Setting Value')
    		elif keyBoard == ord('r'):
             		cv2.destroyAllWindows()
              		imageToDisplay = 0
              		print ('Close All Windows')
    		elif keyBoard == ord('x'):
	      		cv2.destroyAllWindows()
              		imageToDisplay = 0
              		print('Exit Program')
              		break
    		elif keyBoard == ord('z'):
	      		showHelp()
    		elif keyBoard == ord('c'):
	      		cv2.imwrite(IMAGE_POSITIVE_PATH.format(frameIdPOS), modRgbImage)
              		frameIdPOS += 1
              		print 'Capture Positive Image'
    		elif keyBoard == ord('v'):
	      		cv2.imwrite(IMAGE_NEGATIVE_PATH.format(frameIdNEG), modRgbImage)
              		frameIdNEG += 1
              		print 'Capture Negative Image'
    		elif keyBoard == ord('q'):
              		if clear == 0:
                    		clear = 1
          	    		print 'Hidden Line'
              		elif clear == 1:
          	    		clear = 0
          	    		print 'Show Line'
    		elif keyBoard == ord('o'):
              		cv2.destroyAllWindows()
              		imageToDisplay = 5
              		createTrackBar(imageToDisplay)
              		loadTrackBar(imageToDisplay)
              		print('set Camera Parameter')
        	if keyBoard == ord('p'):
              		setCameraParameter()
        	if keyBoard == ord('i'):
              		os.system("v4l2-ctl --set-ctrl exposure_auto={}".format(3))
              		print ('v4l2-ctl auto exposure enabled')

		end = time.time() # End time
                seconds = end - start # Time elapsed
                fps  = 1 / seconds; # fps

	cap.release()
	cv2.destroyAllWindows()

if __name__ == "__main__":
        print 'Running BarelangFC-Vision'
        main()
