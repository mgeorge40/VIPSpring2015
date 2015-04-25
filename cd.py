import cv2
import numpy as np
from datetime import datetime
   
def colorDetector():
	startTime = datetime.now()
	img = cv2.imread('test.jpg')
	hsvImg = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

	orgHeight, orgWidth = img.shape[:2]

	hsvLow = np.array([158, 130, 100], np.uint8)
	hsvHigh = np.array([173, 185, 135], np.uint8)
	frame_threshed = cv2.inRange(hsvImg, hsvLow, hsvHigh)
	
	#cv2.imwrite('threshold.jpg', frame_threshed)
	#simpleImg = cv2.imread('threshold.jpg')

	scaleBy = 0.5

	small = cv2.resize(frame_threshed, (0,0), fx=scaleBy, fy=scaleBy)
	cv2.imwrite('threshold-small.jpg', small)
	smallImg = cv2.imread('threshold-small.jpg')
	height, width = smallImg.shape[:2]

	numWhite = 0
	heightSum = 0
	widthSum = 0
	for h in range (0, height):
		for w in range (0, width):
			if smallImg[h,w][0] == 255:
				numWhite+=1
				heightSum += h
				widthSum += w

	avgHeight = heightSum/numWhite
	avgWidth = widthSum/numWhite

	scaleToOriginal = int(1.0/scaleBy)

	centerPoint = (avgHeight*scaleToOriginal,avgWidth*scaleToOriginal)

	angleOfPatch = round(((avgWidth/float(width))*57 - 28.5),3)


	print "Time: " + str(datetime.now() - startTime)
	toReturn = []
	toReturn.append(centerPoint)
	toReturn.append(angleOfPatch)
	print "the answer is: " + str(toReturn)
	return toReturn
colorDetector()