import os
import numpy as np
import cv2
import socket
import time

from scipy.spatial import distance

host = 'localhost'
port = 2000

# Default filename to save all data
IMAGE_POSITIVE_PATH = "../sourceCode/Positif_example/images_gawang_{:01d}.jpg"
IMAGE_NEGATIVE_PATH = "../sourceCode/Negatif_example/image_{:01d}.png"

# Classifier
bola_cascade = cv2.CascadeClassifier('cascade_10000.xml')

# Default filename to save all data
settingValueFilename='BarelangFC-SettingValue_N.csv'

# <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< Global variable >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
cameraSetting = np.zeros(10, dtype=int)

# Field
lowerFieldGr = np.zeros(3, dtype=int)
upperFieldGr = 255 * np.ones(3, dtype=int)
edFieldGr = np.zeros(2, dtype=int)

# Ball
BallParameter = np.zeros(4, dtype=int)

# Goal
lowerGoalWh = np.zeros(3, dtype=int)
upperGoalWh = 255 * np.ones(3, dtype=int)
edGoalWh = np.zeros(2, dtype=int)
debugGoalWh = np.zeros(16, dtype=int)
detectedGoal = np.zeros(7, dtype=int)
#<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

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
        elif mode == 3: #goal
		cv2.createTrackbar('Debug Goal','Control',0,1,nothing)
		cv2.createTrackbar('HMax','Control',255,255,nothing)
                cv2.createTrackbar('HMin','Control',0,255,nothing)
                cv2.createTrackbar('SMax','Control',255,255,nothing)
                cv2.createTrackbar('SMin','Control',0,255,nothing)
                cv2.createTrackbar('VMax','Control',255,255,nothing)
                cv2.createTrackbar('VMin','Control',0,255,nothing)
                cv2.createTrackbar('Erode','Control',0,10,nothing)
                cv2.createTrackbar('Dilate','Control',0,100,nothing)
        elif mode == 4: #goal Parameter
		#Ax100 10000, RAx1 100, ARx100 100, WRx10 20, PWx1 100, Rx1 320, Hx1 480, wx1 640.
                cv2.namedWindow('Control2')
                cv2.createTrackbar('Goal','Control2',0,20,nothing)
                #cv2.createTrackbar('Height Max','Control2',0,480,nothing)
                #cv2.createTrackbar('Height Min','Control2',0,480,nothing)
                #cv2.createTrackbar('Width Max','Control2',0,640,nothing)
                #cv2.createTrackbar('Width Min','Control2',0,640,nothing)
                #cv2.createTrackbar('Area Max','Control2',0,3000,nothing) #10000
                #cv2.createTrackbar('Area Min','Control2',0,3000,nothing) #10000
                cv2.createTrackbar('Rect Area Max','Control2',0,100,nothing)
                cv2.createTrackbar('Rect Area Min','Control2',0,100,nothing)
                cv2.createTrackbar('Area Ratio Max','Control2',0,100,nothing)
                cv2.createTrackbar('Area Ratio Min','Control2',0,100,nothing)
                cv2.createTrackbar('Wh Ratio Max','Control2',0,100,nothing)
                cv2.createTrackbar('Wh Ratio Min','Control2',0,100,nothing)
                #cv2.createTrackbar('Percent White Max','Control2',0,100,nothing)
                #cv2.createTrackbar('Percent White Min','Control2',0,100,nothing)
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
        elif mode == 3: #goal Load Control
		cv2.setTrackbarPos('HMin', 'Control', lowerGoalWh[0])
                cv2.setTrackbarPos('SMin', 'Control', lowerGoalWh[1])
                cv2.setTrackbarPos('VMin', 'Control', lowerGoalWh[2])
                cv2.setTrackbarPos('HMax', 'Control', upperGoalWh[0])
                cv2.setTrackbarPos('SMax', 'Control', upperGoalWh[1])
                cv2.setTrackbarPos('VMax', 'Control', upperGoalWh[2])
                cv2.setTrackbarPos('Erode', 'Control', edGoalWh[0])
                cv2.setTrackbarPos('Dilate', 'Control', edGoalWh[1])
        elif mode == 4: #Goal Load Parameter
		#cv2.setTrackbarPos('Height Max', 'Control2', debugGoalWh[0])
                #cv2.setTrackbarPos('Height Min', 'Control2', debugGoalWh[1])
                #cv2.setTrackbarPos('Width Max', 'Control2', debugGoalWh[2])
                #cv2.setTrackbarPos('Width Min', 'Control2', debugGoalWh[3])
                #cv2.setTrackbarPos('Area Max', 'Control2', debugGoalWh[4])
                #cv2.setTrackbarPos('Area Min', 'Control2', debugGoalWh[5])
                cv2.setTrackbarPos('Rect Area Max', 'Control2', debugGoalWh[6])
                cv2.setTrackbarPos('Rect Area Min', 'Control2', debugGoalWh[7])
                cv2.setTrackbarPos('Area Ratio Max', 'Control2', debugGoalWh[8])
                cv2.setTrackbarPos('Area Ratio Min', 'Control2', debugGoalWh[9])
                cv2.setTrackbarPos('Wh Ratio Max', 'Control2', debugGoalWh[10])
                cv2.setTrackbarPos('Wh Ratio Min', 'Control2', debugGoalWh[11])
                #cv2.setTrackbarPos('Percent White Max', 'Control2', debugGoalWh[12])
                #cv2.setTrackbarPos('Percent White Min', 'Control2', debugGoalWh[13])
                #cv2.setTrackbarPos('Solidity Max', 'Control2', debugGoalWh[14])
                #cv2.setTrackbarPos('Solidity Min', 'Control2', debugGoalWh[15])
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
	npSettingValue = np.zeros(46, dtype=int)
        #Nilai Field
        npSettingValue[0]= lowerFieldGr[0]
        npSettingValue[1]= lowerFieldGr[1]
        npSettingValue[2]= lowerFieldGr[2]
        npSettingValue[3]= upperFieldGr[0]
        npSettingValue[4]= upperFieldGr[1]
        npSettingValue[5]= upperFieldGr[2]
        npSettingValue[6]= edFieldGr[0]
        npSettingValue[7]= edFieldGr[1]
	#Nilai Bola
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
	#Nilai Gawang
	npSettingValue[22] = lowerGoalWh[0]
        npSettingValue[23] = lowerGoalWh[1]
        npSettingValue[24] = lowerGoalWh[2]
        npSettingValue[25] = upperGoalWh[0]
        npSettingValue[26] = upperGoalWh[1]
        npSettingValue[27] = upperGoalWh[2]
        npSettingValue[28] = edGoalWh[0]
        npSettingValue[29] = edGoalWh[1]
	#Nilai Parameter Gawang
	npSettingValue[30] = debugGoalWh[0]
        npSettingValue[31] = debugGoalWh[1]
        npSettingValue[32] = debugGoalWh[2]
        npSettingValue[33] = debugGoalWh[3]
        npSettingValue[34] = debugGoalWh[4]
        npSettingValue[35] = debugGoalWh[5]
        npSettingValue[36] = debugGoalWh[6]
        npSettingValue[37] = debugGoalWh[7]
        npSettingValue[38] = debugGoalWh[8]
        npSettingValue[39] = debugGoalWh[9]
        npSettingValue[40] = debugGoalWh[10]
        npSettingValue[41] = debugGoalWh[11]
        npSettingValue[42] = debugGoalWh[12]
        npSettingValue[43] = debugGoalWh[13]
        npSettingValue[44] = debugGoalWh[14]
        npSettingValue[45] = debugGoalWh[15]

        npSettingValue=np.reshape(npSettingValue,(1,46))
        headerLabel= 'F HMin, F SMin, VMin, F HMax, F SMax, F VMax, F Erode, F Dilate, B minNeighbors, B flags, B minSize, B maxSize, Brightness, Contrast, Saturation, WB Temp Auto, WB Temp, Sharpness, Exp Auto, Exp Abs, Exp Auto Priority, Focus Auto, G HMin, G SMin, G SMin, G HMax, G SMax, G SMax, G Erode, G Dilate, G Wh HgMax, G Wh HgMin, G Wh WdMax, G Wh WdMin, G Wh AMax, G Wh AMin, G Wh RAMax, G Wh RAMin, G Wh ARMax, G Wh ARMin, G Wh WRMax, G Wh WRMin, G Wh PWMax, G Wh PWMin, G Wh SDMax, G Wh SDMin'
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
	lowerGoalWh[0] = csvSettingValue[22]
        lowerGoalWh[1] = csvSettingValue[23]
        lowerGoalWh[2] = csvSettingValue[24]
        upperGoalWh[0] = csvSettingValue[25]
        upperGoalWh[1] = csvSettingValue[26]
        upperGoalWh[2] = csvSettingValue[27]
        edGoalWh[0] = csvSettingValue[28]
        edGoalWh[1] = csvSettingValue[29]

	#Goal Parameter
	debugGoalWh[0] = csvSettingValue[30]
	debugGoalWh[1] = csvSettingValue[31]
	debugGoalWh[2] = csvSettingValue[32]
	debugGoalWh[3] = csvSettingValue[33]
	debugGoalWh[4] = csvSettingValue[34]
	debugGoalWh[5] = csvSettingValue[35]
	debugGoalWh[6] = csvSettingValue[36]
	debugGoalWh[7] = csvSettingValue[37]
	debugGoalWh[8] = csvSettingValue[38]
	debugGoalWh[9] = csvSettingValue[39]
	debugGoalWh[10] = csvSettingValue[40]
	debugGoalWh[11] = csvSettingValue[41]
	debugGoalWh[12] = csvSettingValue[42]
	debugGoalWh[13] = csvSettingValue[43]
	debugGoalWh[14] = csvSettingValue[44]
	debugGoalWh[15] = csvSettingValue[45]
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
    	IMAGE_WIDTH = 640
    	IMAGE_HEIGHT = 480
	IMAGE_AREA = IMAGE_HEIGHT * IMAGE_WIDTH

    	#init calibration single camera
	#Ball
    	B_KNOWN_PIXEL = 115 #lebar pixel without calibration
    	B_KNOWN_DISTANCE = 50 #jarak cm
    	B_KNOWN_WIDTH = 14 #diameter cm
    	B_focalLength = (B_KNOWN_PIXEL * B_KNOWN_DISTANCE) / B_KNOWN_WIDTH
	#Field
	G_KNOWN_PIXEL = 198 #lebar pixel without calibration
        G_KNOWN_DISTANCE = 440 #jarak cm
        G_KNOWN_HEIGHT = 162 #180 #tinggi cm
        G_focalLength = (G_KNOWN_PIXEL * G_KNOWN_DISTANCE) / G_KNOWN_HEIGHT

    	frameIdPOS = 0 #data gambar terakhir di folder Positive_example
    	frameIdNEG = 0 #data gambar terakhir di folder Negative_example

    	# Image yang akan ditampilkan
    	imageToDisplay = 0
    	kernel = cv2.getStructuringElement(cv2.MORPH_RECT,(2,2))

	# Define the codec and create VideoWriter object
	fourcc = cv2.VideoWriter_fourcc(*'XVID')
	out = cv2.VideoWriter('output.mp4',fourcc, 20.0, (IMAGE_WIDTH, IMAGE_HEIGHT))

	# Local Variable
	goalColor = (255, 0, 0 )
	last_debug = debug_goal = clear = get = 0
	goalProperties = np.zeros((1,10))
	fps = 30

    	showHelp()
    	loadConfig()
    	os.system("v4l2-ctl --set-ctrl exposure_auto={}".format(3))

    	while(True):
    		# Capture frame-by-frame
    		ret, frame = cap.read()
		start = time.time() # Start time
    		rgbImage = cv2.resize(frame, (IMAGE_WIDTH, IMAGE_HEIGHT),interpolation = cv2.INTER_AREA)

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

		# write the flipped frame
		if get == 1:
			out.write(frame)

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

    		newImage = cv2.bitwise_and(grayImage, fieldMask)
		ballImage = cv2.resize(newImage, (480, 320),interpolation = cv2.INTER_AREA)
    		bola = bola_cascade.detectMultiScale(ballImage, 1.3, BallParameter[0], BallParameter[1], minSize=(BallParameter[2],BallParameter[2]), maxSize=(BallParameter[3],BallParameter[3]))

    		for (x,y,w,h) in bola:
			x = int(x * 1.34)
			y = int(y * 1.5)
			w = int(w * 1.34)
			h = int(h * 1.5)
	     		Ball_x = (x + w + x ) / 2
	      		Ball_y = (y + h + y ) / 2
	      		Ball_distance = distance_to_camera(B_KNOWN_WIDTH, B_focalLength, w)
			if clear == 0:
				cv2.circle(modRgbImage, (Ball_x,Ball_y), 2, (0,0,255), -1)
				cv2.circle(modRgbImage, (Ball_x,Ball_y), (Ball_x - x), (255,0,255), 2, 8, 0)

    		# =============================================================== Goal ============================================================================================
		goalWhBinary = cv2.inRange(yuvImage, lowerGoalWh, upperGoalWh)
		goalWhBinaryErode = cv2.erode(goalWhBinary,kernel,iterations = edGoalWh[0])
		goalWhBinaryDilate = cv2.dilate(goalWhBinaryErode,kernel,iterations = edGoalWh[1])
		goalWhFinal = cv2.bitwise_and(goalWhBinaryDilate,notFieldMask)

		# Goal detection variable
		goalContourLen = 0
		goalIteration = 0

		# Initialize to default
		detectedGoal[0] = -1 #-888
		detectedGoal[1] = -1
		detectedGoal[2] = 0
		detectedGoal[3] = 0
		detectedGoal[4] = 0
		detectedGoal[5] = -1
		detectedGoal[6] = -1

		# Field Contour Detection
		_, listGoalContours, _ = cv2.findContours(goalWhFinal.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
		if len(listGoalContours) > 0:
			listSortedGoalContours = sorted(listGoalContours, key=cv2.contourArea, reverse=True)[:5]
			goalContourLen += len(listSortedGoalContours)
			goal_iteration = 1
			for goalContour in listSortedGoalContours:
				goalTopLeftX, goalTopLeftY, goalWidth, goalHeight = cv2.boundingRect(goalContour)
				# mode baru
				#print goalWidth, goalHeight
				goalArea = float(cv2.contourArea(goalContour)) / float(IMAGE_AREA) * 100.0		#area
				goalRectArea = (float(goalWidth) * float(goalHeight)) / float(IMAGE_AREA) * 100.0	#rect_area
				#goalRectArea = (float(goalWidth) / float(goalHeight)) * 100
				if goalRectArea != 0:
					goalExtent = float(goalArea) / float(goalRectArea)				#area_ratio
				if goalHeight != 0:
					goalAspectRatio = float(goalWidth) / float(goalHeight)				#wh_ratio
				#goalHull = cv2.convexHull(goalContour)
				#goalHullArea = cv2.contourArea(goalHull) / float(IMAGE_AREA)
				#if goalHullArea > 0:
				#	goalSolidity = float(goalArea) / float(goalHullArea)
				#else:
				#	goalSolidity = 0

				goalRoiBinary = goalWhFinal[goalTopLeftY:goalTopLeftY + goalHeight, goalTopLeftX:goalTopLeftX + goalWidth]

				#sepertinya goalRrctArea perlu di kali 100, cz beda dengan nilai yg lama
				if debug_goal == 1:
					selected_goal = cv2.getTrackbarPos('Goal','Control2')
					if selected_goal == goal_iteration :
						print 'Goal --> RA = %.2f, AR = %.2f, WH_Rat = %.2f'%(goalRectArea, goalExtent*100, goalAspectRatio*10)
						goal_color = (0, 0, 255)
					else:
						goal_color = (255, 255, 255)
					#cv2.drawContours(modRgbImage, [goal_box], 0, goal_color, 3)
					cv2.rectangle(modRgbImage, (goalTopLeftX,goalTopLeftY), (goalTopLeftX + goalWidth, goalTopLeftY + goalHeight), goal_color, 3)

				#Ax100 10000, RAx1 100, ARx100 100, WRx10 20, PWx1 100, Rx1 320, Hx1 480, wx1 640.
				if goalRectArea >= debugGoalWh[7] and goalRectArea <= debugGoalWh[6]:
					if (goalExtent >= (float(debugGoalWh[9])/100)) and (goalExtent <= (float(debugGoalWh[8])/100)):
						if (goalAspectRatio >= (float(debugGoalWh[11])/10)) and (goalAspectRatio <= (float(debugGoalWh[10])/10)):
							# print 'Contour'
							# print goalContour[:,0,:]
							npGoalContourPoint = goalContour[:,0,:]
							# Create numpy ROI rectangle points
							npRoiTopLeft = np.array([[goalTopLeftX, goalTopLeftY]])
							npRoiTopRight = np.array([[(goalTopLeftX + goalWidth), goalTopLeftY]])
							npRoiBottomRight = np.array([[(goalTopLeftX + goalWidth), (goalTopLeftY + goalHeight)]])
							npRoiBottomLeft = np.array([[goalTopLeftX, (goalTopLeftY + goalHeight)]])
							# Gambar titik
							# cv2.circle(modRgbImage, tuple(npRoiTopLeft), 8, (0, 0, 255), -1)
							# cv2.circle(modRgbImage, tuple(npRoiTopRight), 8, (0, 255, 0), -1)
							# cv2.circle(modRgbImage, tuple(npRoiBottomRight), 8, (255, 0, 0), -1)
							# cv2.circle(modRgbImage, tuple(npRoiBottomLeft), 8, (255, 255, 0), -1)
							# Find minimum distance from ROI points to contour
							npDistanceResult = distance.cdist(npRoiTopLeft, npGoalContourPoint, 'euclidean')
							poleTopLeft = tuple(npGoalContourPoint[np.argmin(npDistanceResult),:])

							npDistanceResult = distance.cdist(npRoiTopRight, npGoalContourPoint, 'euclidean')
							poleTopRight = tuple(npGoalContourPoint[np.argmin(npDistanceResult),:])

							npDistanceResult = distance.cdist(npRoiBottomRight, npGoalContourPoint, 'euclidean')
							poleBottomRight = tuple(npGoalContourPoint[np.argmin(npDistanceResult),:])

							npDistanceResult = distance.cdist(npRoiBottomLeft, npGoalContourPoint, 'euclidean')
							poleBottomLeft = tuple(npGoalContourPoint[np.argmin(npDistanceResult),:])

							showPole = True
							if showPole == True:
								if clear == 0:
									cv2.circle(modRgbImage, poleBottomLeft, 8, (0, 0, 255), -1)
									cv2.circle(modRgbImage, poleTopRight, 8, (0, 255, 0), -1)
									cv2.circle(modRgbImage, poleTopLeft, 8, (255, 0, 0), -1)
									cv2.circle(modRgbImage, poleBottomRight, 8, (255, 255, 0), -1)
							poleLeftHeight = 0
							poleRightHeight = 0
							topDistance = 0
							bottomDistance = 0
							try:
								poleLeftHeight = np.linalg.norm(np.array(poleTopLeft)-np.array(poleBottomLeft))
								poleRightHeight = np.linalg.norm(np.array(poleTopRight)-np.array(poleBottomRight))
								topDistance = np.linalg.norm(np.array(poleTopRight)-np.array(poleTopLeft))
								bottomDistance = np.linalg.norm(np.array(poleBottomRight)-np.array(poleBottomLeft))
							except:
								pass
							if topDistance < 80:
								continue

							# Pecah jadi 2 bagian
							goalRoiLeft = goalRoiBinary[0:goalHeight, 0:goalWidth/2]
							goalRoiRight = goalRoiBinary[0:goalHeight, goalWidth/2:goalWidth-1]

							# Hitung titik pusat gawang kiri
							poleMomentPosition = np.zeros((2,2), dtype=int)
							for goalPolePosition in range(0, 2):
								if goalPolePosition == 0:
									_, listGoalPoleContours, _ = cv2.findContours(goalRoiLeft.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
								elif goalPolePosition == 1:
									_, listGoalPoleContours, _ = cv2.findContours(goalRoiRight.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
								goalPoleContour = sorted(listGoalPoleContours, key=cv2.contourArea, reverse=True)[:1]
								try:
									goalPoleMoment = cv2.moments(goalPoleContour[0])
								except:
									pass
									# Kurang exception div by zero
								try:
									poleMomentPosition[goalPolePosition, 0] = int(goalPoleMoment["m10"] / goalPoleMoment["m00"]) #x
									poleMomentPosition[goalPolePosition, 1] = int(goalPoleMoment["m01"] / goalPoleMoment["m00"]) #y
								except:
									pass
							# Gambar titik moment
							showMoment = True
							if showMoment == True:
								if clear == 0:
									cv2.circle(modRgbImage, (goalTopLeftX + poleMomentPosition[0,0], goalTopLeftY + poleMomentPosition[0,1]), 7, (50, 100, 255), -1)
									cv2.circle(modRgbImage, (goalTopLeftX + goalWidth/2 + poleMomentPosition[1,0], goalTopLeftY + poleMomentPosition[1,1]), 7, (50, 100, 255), -1)

							cpX = (goalTopLeftX + poleMomentPosition[0,0])
							cpY = (goalTopLeftY + poleMomentPosition[0,1])
							checkpointGoalX = cpX + (abs(goalTopLeftX + goalWidth/2 + poleMomentPosition[1,0]) - (goalTopLeftX + poleMomentPosition[0,0]))/2
							checkpointGoalY = cpY + (abs(goalTopLeftY + poleMomentPosition[1,1]) - (goalTopLeftY + poleMomentPosition[0,1])) / 2
							cv2.circle(modRgbImage, (checkpointGoalX, checkpointGoalY), 15, (50, 255, 255), -1)
							goalPix0 = goalWhFinal.copy()[checkpointGoalY, checkpointGoalX] #[Y,X]
							#print goalPix0

							# Cari selisih titik moment y dari tiang 1 dan tiang 2
							diffMomentPosition = poleMomentPosition[0,1] - poleMomentPosition[1,1]
							poleClass = 0
							# print diffMomentPosition
							if diffMomentPosition < -80:
								poleClass = 1
							elif diffMomentPosition > 80:
								poleClass = -1
							else:
								poleClass = 0

							if goalPix0 == 0:
								if clear == 0:
									cv2.rectangle(modRgbImage, (goalTopLeftX,goalTopLeftY), (goalTopLeftX + goalWidth, goalTopLeftY + goalHeight), goalColor, 2)
								detectedGoal[0] = goalTopLeftX + goalWidth / 2 #X
								detectedGoal[1] = goalTopLeftY + goalHeight / 2 #Y
								detectedGoal[2] = poleLeftHeight
								detectedGoal[3] = poleRightHeight
								detectedGoal[4] = poleClass #-1 0 1
								poleLeftDistance = distance_to_camera(G_KNOWN_HEIGHT, G_focalLength, poleLeftHeight)
								poleRightDistance = distance_to_camera(G_KNOWN_HEIGHT, G_focalLength, poleRightHeight)
								detectedGoal[5] = poleLeftDistance
								detectedGoal[6] = poleRightDistance
							else:
								detectedGoal[0] = -1 #-888
								detectedGoal[1] = -1
								detectedGoal[2] = 0
								detectedGoal[3] = 0
								detectedGoal[4] = 0
								detectedGoal[5] = -1
								detectedGoal[6] = -1

							# Udah ketemu ya break aja
							break
				goal_iteration += 1

    		#==================================================================================================================================================================

    		# Send detection result to localhost
    		try:
	     		#s.flush()
              		visionMessage = '%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d'%(Ball_x, Ball_y, 0, 0, Ball_distance, detectedGoal[0], detectedGoal[1], detectedGoal[2], detectedGoal[3], detectedGoal[4], detectedGoal[5], detectedGoal[6])
              		s.sendto(visionMessage, (host, port))
    		except socket.error:
	      		#print 'Error Code : ' + str(msg[0]) + ' Message ' + msg[1]
              		sys.exit()

    		# Print Value to Frame
    		font = cv2.FONT_HERSHEY_SIMPLEX
    		textLine0 = '%d FPS'%(fps)
    		textLine1 = 'Ball -> X = %d, Y = %d, D = %dcm'%(Ball_x, Ball_y, Ball_distance)
		textLine2 = 'Goal -> X = %d, Y = %d, C = %d, LD = %dcm, RD = %dcm'%(detectedGoal[0], detectedGoal[1], detectedGoal[4], detectedGoal[5], detectedGoal[6])
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
    		elif imageToDisplay == 3: #Goal
			lowerGoalWh[0] = cv2.getTrackbarPos('HMin','Control')
			lowerGoalWh[1] = cv2.getTrackbarPos('SMin','Control')
			lowerGoalWh[2] = cv2.getTrackbarPos('VMin','Control')
			upperGoalWh[0] = cv2.getTrackbarPos('HMax','Control')
			upperGoalWh[1] = cv2.getTrackbarPos('SMax','Control')
			upperGoalWh[2] = cv2.getTrackbarPos('VMax','Control')
			edGoalWh[0] = cv2.getTrackbarPos('Erode','Control')
			edGoalWh[1] = cv2.getTrackbarPos('Dilate','Control')

			debug_goal = cv2.getTrackbarPos('Debug Goal','Control')
			if last_debug != debug_goal:
				if (debug_goal == 1):
					createTrackBar(4)
					loadTrackBar(4)
				else:
					cv2.destroyWindow("Control2")
				last_debug = debug_goal
			if (debug_goal == 1):
				#debugGoalWh[0] = cv2.getTrackbarPos('Height Max', 'Control2')
				#debugGoalWh[1] = cv2.getTrackbarPos('Height Min', 'Control2')
				#debugGoalWh[2] = cv2.getTrackbarPos('Width Max', 'Control2')
				#debugGoalWh[3] = cv2.getTrackbarPos('Width Min', 'Control2')
				#debugGoalWh[4] = cv2.getTrackbarPos('Area Max', 'Control2')
				#debugGoalWh[5] = cv2.getTrackbarPos('Area Min', 'Control2')
				debugGoalWh[6] = cv2.getTrackbarPos('Rect Area Max', 'Control2')
				debugGoalWh[7] = cv2.getTrackbarPos('Rect Area Min', 'Control2')
				debugGoalWh[8] = cv2.getTrackbarPos('Area Ratio Max', 'Control2')
				debugGoalWh[9] = cv2.getTrackbarPos('Area Ratio Min', 'Control2')
				debugGoalWh[10] = cv2.getTrackbarPos('Wh Ratio Max', 'Control2')
				debugGoalWh[11] = cv2.getTrackbarPos('Wh Ratio Min', 'Control2')
				#debugGoalWh[12] = cv2.getTrackbarPos('Percent White Max', 'Control2')
				#debugGoalWh[13] = cv2.getTrackbarPos('Percent White Min', 'Control2')
				#debugGoalWh[14] = cv2.getTrackbarPos('Solidity Max', 'Control2')
				#debugGoalWh[15] = cv2.getTrackbarPos('Solidity Min', 'Control2')

			cv2.imshow("Barelang Vision", modRgbImage)
			cv2.imshow("Goal Binary Image", goalWhFinal)
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
			cv2.imshow('Barelang Vision', modRgbImage)
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
			last_debug = 0
              		imageToDisplay = 3
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
    		elif keyBoard == ord('w'):
              		if get == 0:
                    		get = 1
          	    		print 'Start Video'
              		elif get == 1:
          	    		get = 0
          	    		print 'Stop Video'
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
	out.release()
	cv2.destroyAllWindows()

if __name__ == "__main__":
        print 'Running BarelangFC-Vision'
        main()
