import cv2
import numpy as np
import json
 
# Opening JSON file
try:
    with open('red_data_kalibrasi_2.json', 'r') as openfile:
        data_kalibrasi = json.load(openfile)
except Exception as e:
    data_kalibrasi = {
        "min" : [0,0,0],
        "max" : [0,0,0] 
    }


def nothing(x):
    pass

cap = cv2.VideoCapture(0)
# cap2 = cv2.VideoCapture(1)
cv2.namedWindow('RED')

# create trackbars for color change
cv2.createTrackbar('min_H','RED',data_kalibrasi['min'][0],255,nothing)
cv2.createTrackbar('min_S','RED',data_kalibrasi['min'][1],255,nothing)
cv2.createTrackbar('min_V','RED',data_kalibrasi['min'][2],255,nothing)
cv2.createTrackbar('max_H','RED',data_kalibrasi['max'][0],255,nothing)
cv2.createTrackbar('max_S','RED',data_kalibrasi['max'][1],255,nothing)
cv2.createTrackbar('max_V','RED',data_kalibrasi['max'][2],255,nothing)

pause = False

while(True):
    # Capture frame-by-frame
    if pause:
        frame = beforeFrame
    else :
        ret, frame = cap.read()
    
    if not ret:
        with open("red_data_kalibrasi_2.json", "w") as outfile:
            json.dump(data_kalibrasi, outfile)
        break


    # Our operations on the frame come here
    # get current positions of four trackbars
    data_kalibrasi['min'][0] = cv2.getTrackbarPos('min_H','RED')
    data_kalibrasi['min'][1] = cv2.getTrackbarPos('min_S','RED')
    data_kalibrasi['min'][2] = cv2.getTrackbarPos('min_V','RED')
    data_kalibrasi['max'][0] = cv2.getTrackbarPos('max_H','RED')
    data_kalibrasi['max'][1] = cv2.getTrackbarPos('max_S','RED')
    data_kalibrasi['max'][2] = cv2.getTrackbarPos('max_V','RED')

    # Segment image based on the color limits
    min_limit = np.array(data_kalibrasi['min'])
    max_limit = np.array(data_kalibrasi['max'])
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, min_limit, max_limit)

    # Calculate centroid
    # calculate moments of binary image
    blur = cv2.medianBlur(mask, 11)
    M = cv2.moments(blur)
    # calculate x,y coordinate of center
    if M["m00"] != 0:
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
    else:
        cX = 0
        cY = 0
    cv2.circle(frame, (cX, cY), 5, (255, 255, 255), -1)


    # Display the resulting frame
    frame = cv2.resize(frame, (320, 240))
    mask = cv2.resize(mask, (320, 240))
    blur = cv2.resize(blur, (320, 240))
    
#     cv2.imshow('image',cv2.hconcat([frame, cv2.cvtColor(blur, cv2.COLOR_GRAY2BGR)]))
    cv2.imshow('RED',cv2.hconcat([frame,  cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR), cv2.cvtColor(blur, cv2.COLOR_GRAY2BGR)]))
#     cv2.imshow('CAM2', frame2)
    pressedKey = cv2.waitKey(1) & 0xFF
    if pressedKey == ord('p'):
        beforeFrame = frame
        pause = True
    elif pressedKey == ord('s'):
        pause = False
    elif pressedKey == ord('q'):
        print(min_limit)
        print(max_limit)
        with open("red_data_kalibrasi_2.json", "w") as outfile:
            json.dump(data_kalibrasi, outfile)
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()
