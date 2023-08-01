import cv2
import numpy as np

def nothing(x):
    pass

cap = cv2.VideoCapture(0)
cv2.namedWindow('image')

# create trackbars for color change
# cv2.createTrackbar('min_Hue','image',0,255,nothing)
# cv2.createTrackbar('max_Hue','image',0,255,nothing)
# cv2.createTrackbar('min_Saturation','image',0,255,nothing)
# cv2.createTrackbar('max_Saturation','image',0,255,nothing)
# cv2.createTrackbar('min_Value','image',0,255,nothing)
# cv2.createTrackbar('max_Value','image',0,255,nothing)
red = True
while(True):
    # Capture frame-by-frame
    ret, frame = cap.read()

    # Our operations on the frame come here
    # get current positions of four trackbars
    # min_h = cv2.getTrackbarPos('min_Hue','image')
    # max_h = cv2.getTrackbarPos('max_Hue','image')
    # min_s = cv2.getTrackbarPos('min_Saturation','image')
    # max_s = cv2.getTrackbarPos('max_Saturation','image')
    # min_v = cv2.getTrackbarPos('min_Value','image')
    # max_v = cv2.getTrackbarPos('max_Value','image')

    
    
   
    
    

    # Calculate centroid
    # calculate moments of binary image
    # if red :
        # Segment image based on the color limits
    red_min_limit = (129, 96, 27)
    red_max_limit = (179, 255, 255)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    red_mask = cv2.inRange(hsv, red_min_limit, red_max_limit)
    
    # Overlay obtained red_mask on original image
    red_mask_final = np.zeros((hsv.shape))
    red_mask_final[:,:,1] = red_mask
    red_mask_final = np.uint8(red_mask_final)
    # added_image = cv2.addWeighted(frame,0.7,red_mask_final,0.3,0)

    red_M = cv2.moments(red_mask)
    # calculate x,y coordinate of center
    if red_M["m00"] != 0:
        red_cX = int(red_M["m10"] / red_M["m00"])
        red_cY = int(red_M["m01"] / red_M["m00"])
    else:
        red_cX = 0
        red_cY = 0
    cv2.circle(frame, (red_cX, red_cY), 5, (255, 255, 255), -1)
    cv2.putText(frame, "centroid", (red_cX - 25, red_cY - 25),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
    red = False

    
    # else :
    green_min_limit = (39, 66, 127)
    green_max_limit = (85, 255, 178)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    green_mask = cv2.inRange(hsv, green_min_limit, green_max_limit)

    
    green_mask_final = np.zeros((hsv.shape))
    green_mask_final[:,:,1] = green_mask
    green_mask_final = np.uint8(green_mask_final)
    # added_image = cv2.addWeighted(frame,0.7,green_mask_final,0.3,0)

    green_M = cv2.moments(green_mask)
    if green_M["m00"] != 0:
        green_cX = int(green_M["m10"] / green_M["m00"])
        green_cY = int(green_M["m01"] / green_M["m00"])
    else:
        green_cX = 0
        green_cY = 0
    cv2.circle(frame, (green_cX, green_cY), 5, (255, 255, 255), -1)
    cv2.putText(frame, "centroid", (green_cX - 25, green_cY - 25),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
    red = True


    # Display the resulting frame
    cv2.imshow('image',cv2.hconcat([frame]))
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()