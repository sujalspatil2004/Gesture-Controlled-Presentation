import os
import cv2
from cvzone.HandTrackingModule import HandDetector
import numpy as np

#Variables
width , height = 1280 , 720
folderPath = "Presentation"

#Camera setup
cap = cv2.VideoCapture(0)
cap.set(3 , width)
cap.set(4 , height)

# Get the list of presentation images
pathImages = sorted(os.listdir(folderPath) , key=len) #sorted because 10th page coming after 1st
# print(pathImages)

#Variables
imageNumber = 0
hs , ws = int(120*1) , int(213*1)
gestureThreshold = 300
buttonPressed = False
buttonCounter=0
buttonDelay = 30
annotations = [[]]
annotationNumber = 0
annotationStart = False

#Hand Detector
detector = HandDetector(detectionCon=0.8 , maxHands=1)

while True:
    success , img = cap.read()
    img = cv2.flip(img , 1) #1means horizontal flip , 0 means vertical flip
    pathFullImage = os.path.join(folderPath , pathImages[imageNumber])
    imgCurrent = cv2.imread(pathFullImage)

    hands , img = detector.findHands(img)
    cv2.line(img , (0,gestureThreshold) , (width,gestureThreshold) , (0,255,0), 10)

    if hands and buttonPressed==False:
        hand = hands[0]
        fingers = detector.fingersUp(hand)
        cx , cy = hand['center']
        lmList = hand['lmList']

        # Constrain values for easier drawing
        xVal = int(np.interp(lmList[8][0], [0, width], [0, imgCurrent.shape[1]]))
        yVal = int(np.interp(lmList[8][1], [0, height], [0, imgCurrent.shape[0]]))
        indexFinger = xVal ,yVal

        if cy <= gestureThreshold: #hand above line

            #Gesture 1 - Left
            if fingers == [1,0,0,0,0]:
                annotationStart = False
                print("Left")
                if(imageNumber>0):
                    buttonPressed = True
                    annotations = [[]]
                    annotationNumber = 0
                    imageNumber -= 1

            #Gesture 2 - Right
            if fingers == [0,0,0,0,1]:
                annotationStart = False
                print("Right")
                if imageNumber < len(pathImages)-1:
                    buttonPressed = True
                    annotations = [[]]
                    annotationNumber = 0
                    imageNumber += 1

        # Gesture 3 - Show Pointer
        if fingers == [0, 1, 1, 0, 0]:
            cv2.circle(imgCurrent , indexFinger , 12 , (0,0,225) , cv2.FILLED)

        # Gesture 4 - Draw Pointer
        if fingers == [0, 1, 0, 0, 0]:
            if annotationStart is False :
                annotationStart = True
                annotationNumber += 1
                annotations.append([])
            cv2.circle(imgCurrent , indexFinger , 12 , (0,0,225) , cv2.FILLED)
            annotations[annotationNumber].append(indexFinger)
        else:
            annotationStart = False

        # Gesture 5 - Erase Drawing
        if fingers == [0, 1, 1, 1, 0]:
            if annotations:
                annotations.pop(-1)
                annotationNumber -= 1
                buttonPressed = True

    else:
        annotationStart = False

    #Button Pressed iterations
    if buttonPressed :
        buttonCounter += 1
        if buttonCounter > buttonDelay:
            buttonCounter =0
            buttonPressed = False

    for i in range (len(annotations)) :
        for j in  range(len(annotations[i])) :
            if j != 0 :
                cv2.line(imgCurrent , annotations[i][j-1] , annotations[i][j] , (0,0,200) , 12)


    # Resize presentation slide to match the camera resolution (if not done , will show zoomed image)
    imgCurrent = cv2.resize(imgCurrent, (width, height), interpolation=cv2.INTER_AREA)

    #Adding webcam image on the slides
    imgSmall = cv2.resize(img , (ws,hs) , interpolation=cv2.INTER_AREA)
    h , w , _ = imgCurrent.shape
    imgCurrent[:hs , w-ws:] = imgSmall

    cv2.imshow("Image" , img)
    cv2.imshow("Slides" , imgCurrent)
    key = cv2.waitKey(1)
    if key == ord("q"):
        break