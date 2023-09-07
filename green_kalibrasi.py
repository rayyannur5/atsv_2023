import cv2
import numpy as np
import json
 
# Opening JSON file
try:
    with open('green_data_kalibrasi.json', 'r') as openfile:
        data_kalibrasi = json.load(openfile)
except Exception as e:
    data_kalibrasi = {
        "min" : [0,0,0],
        "max" : [0,0,0] 
    }


def nothing(x):
    pass

cap = cv2.VideoCapture(0)
cv2.namedWindow('image')

# create trackbars for color change
cv2.createTrackbar('min_H','image',data_kalibrasi['min'][0],255,nothing)
cv2.createTrackbar('min_S','image',data_kalibrasi['min'][1],255,nothing)
cv2.createTrackbar('min_V','image',data_kalibrasi['min'][2],255,nothing)
cv2.createTrackbar('max_H','image',data_kalibrasi['max'][0],255,nothing)
cv2.createTrackbar('max_S','image',data_kalibrasi['max'][1],255,nothing)
cv2.createTrackbar('max_V','image',data_kalibrasi['max'][2],255,nothing)


while(True):
    # Capture frame-by-frame
    ret, frame = cap.read()
    frame = cv2.resize(frame, (160, 120))


    # Our operations on the frame come here
    # get current positions of four trackbars
    data_kalibrasi['min'][0] = cv2.getTrackbarPos('min_H','image')
    data_kalibrasi['min'][1] = cv2.getTrackbarPos('min_S','image')
    data_kalibrasi['min'][2] = cv2.getTrackbarPos('min_V','image')
    data_kalibrasi['max'][0] = cv2.getTrackbarPos('max_H','image')
    data_kalibrasi['max'][1] = cv2.getTrackbarPos('max_S','image')
    data_kalibrasi['max'][2] = cv2.getTrackbarPos('max_V','image')

    # Segment image based on the color limits
    min_limit = np.array(data_kalibrasi['min'])
    max_limit = np.array(data_kalibrasi['max'])
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, min_limit, max_limit)


    # # Overlay obtained mask on original image
    # mask_final = np.zeros((hsv.shape))
    # mask_final[:,:,1] = mask
    # mask_final = np.uint8(mask_final)
    # added_image = cv2.addWeighted(frame,0.7,mask_final,0.3,0)

    # Calculate centroid
    # calculate moments of binary image
    # M = cv2.moments(mask)
    # # calculate x,y coordinate of center
    # if M["m00"] != 0:
    #     cX = int(M["m10"] / M["m00"])
    #     cY = int(M["m01"] / M["m00"])
    # else:
    #     cX = 0
    #     cY = 0
    # cv2.circle(added_image, (cX, cY), 5, (255, 255, 255), -1)
    # cv2.putText(added_image, "centroid", (cX - 25, cY - 25),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)


    # gray = cv2.cvtColor(added_image, cv2.COLOR_BGR2GRAY)
    blur = cv2.medianBlur(mask, 25)
    thresh = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)[1]

    # Morph open 
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5,5))
    opening = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel, iterations=3)

    # Find contours and filter using contour area and aspect ratio
    cnts = cv2.findContours(opening, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = cnts[0] if len(cnts) == 2 else cnts[1]
    for c in cnts:
        peri = cv2.arcLength(c, True)
        approx = cv2.approxPolyDP(c, 0.02 * peri, True)
        area = cv2.contourArea(c)
        if len(approx) > 5 and area > 50 and area < 500000:
            ((x, y), r) = cv2.minEnclosingCircle(c)
            cv2.circle(frame, (int(x), int(y)), 5, (255, 255, 255), 2)
            cv2.circle(frame, (int(x), int(y)), int(r), (36, 255, 2), 2)




    # Display the resulting frame
    cv2.imshow('image',cv2.hconcat([frame,  cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR), cv2.cvtColor(blur, cv2.COLOR_GRAY2BGR)]))
    if cv2.waitKey(1) & 0xFF == ord('q'):

        print(min_limit)
        print(max_limit)
        with open("green_data_kalibrasi.json", "w") as outfile:
            json.dump(data_kalibrasi, outfile)
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()