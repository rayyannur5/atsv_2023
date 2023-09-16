import cv2
import numpy as np
import json
import serial
from threading import Thread


try:
    with open('red_data_kalibrasi.json', 'r') as openfile:
        red_data = json.load(openfile)
except Exception as e:
    print('data kalibrasi merah tidak ada')
    exit()

try:
    with open('green_data_kalibrasi.json', 'r') as openfile:
        green_data = json.load(openfile)
except Exception as e:
    print('data kalibrasi merah tidak ada')
    exit()

try:
    ser = serial.Serial('/dev/ttyUSB0', 115200)
except Exception as e:
    print(e)
    exit()


cap = cv2.VideoCapture('my_video-3.mkv')

def detect(frame, mask, blur, text):
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
    cv2.putText(frame, text, (cX - 25, cY - 25),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
    return cX, cY
    # blur = cv2.medianBlur(mask, blur)
    # thresh = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)[1]

    # # Morph open 
    # kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5,5))
    # opening = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel, iterations=3)

    # # Find contours and filter using contour area and aspect ratio
    # cnts = cv2.findContours(opening, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    # cnts = cnts[0] if len(cnts) == 2 else cnts[1]
    # for c in cnts:
    #     peri = cv2.arcLength(c, True)
    #     approx = cv2.approxPolyDP(c, 0.00 * peri, True)
    #     area = cv2.contourArea(c)
    #     if len(approx) > 3 and area > 0 and area < 500000:
    #         ((x, y), r) = cv2.minEnclosingCircle(c)
    #         cv2.circle(frame, (int(x), int(y)), 1, (255, 255, 255), 2)
    #         cv2.circle(frame, (int(x), int(y)), int(r), (36, 255, 2), 2)
    #         cv2.putText(frame, text, (int(x) - 20, int(y) - 25),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

    #         return x, y
    #     else :
    #         return 0,0
    # return 0,0


def decode_gps(data):
    gps = data[3:]
    data_gps = gps.split('<>')
    lat = float(data_gps[0])
    lon = float(data_gps[1])
    return lat, lon


def thread_serial():
    while True:
        data_ser = ser.readline().decode().strip()
        print(data_ser)
        if 'GPS' in data_ser:
            try:
                with open("data_share.json", "r") as r:
                    data_share = json.load(r)
                with open("data_share.json", "w") as w:
                    lat, lon = decode_gps(data_ser)
                    data_share['gps']['lat'] = lat
                    data_share['gps']['lon'] = lon
                    json.dump(data_share, w)
            except Exception as e:
                with open("data_share.json", "w") as w:
                    lat, lon = decode_gps(data_ser)
                    json.dump({
                        'gps' : {
                            'lat' : lat,
                            'lon' : lon
                        },
                        'speed' : '0',
                        'compass' : '0',
                    }, w)

        if 'SPEED' in data_ser:
            try:
                with open("data_share.json", "r") as r:
                    data_share = json.load(r)
                with open("data_share.json", "w") as w:
                    data_share['speed'] = data_ser[5:]
                    json.dump(data_share, w)
            except Exception as e:
                with open("data_share.json", "w") as w:
                    json.dump({
                        'gps' : {
                            'lat' : 0,
                            'lon' : 0
                        },
                        'speed' : data_ser[5:],
                        'compass' : '0',
                    }, w)
        if 'COMPASS' in data_ser:
            try:
                with open("data_share.json", "r") as r:
                    data_share = json.load(r)
                with open("data_share.json", "w") as w:
                    data_share['compass'] = data_ser[7:]
                    json.dump(data_share, w)
            except Exception as e:
                with open("data_share.json", "w") as w:
                    json.dump({
                        'gps' : {
                            'lat' : 0,
                            'lon' : 0
                        },
                        'speed' : '0',
                        'compass' : data_ser[7:]
                    }, w)
            # cv2.putText(blackboard, speed, (20, 60),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break



serial = Thread(target = thread_serial)
# vision = Thread(target=thread_vision)
# vision.start()
serial.start()

# def thread_vision():
while True:
    ret, frame = cap.read()
    frame = cv2.resize(frame, (160, 120))
    blackboard = np.zeros((240, 350, 3), dtype='uint8')

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    red_mask = cv2.inRange(hsv, np.array(red_data['min']),  np.array(red_data['max']))
    green_mask = cv2.inRange(hsv, np.array(green_data['min']),  np.array(green_data['max']))
    
    green_x, green_y = detect(frame, green_mask, 11, 'green')
    red_x, red_y = detect(frame, red_mask, 11, 'red')

    if green_x < red_x:
        center_x = green_x + (red_x-green_x)/2
    elif red_x < green_x :
        center_x = red_x + (green_x-red_x)/2
    else :
        center_x = 0

    
    cv2.circle(frame, (int(center_x), 60), 3, (255, 255, 0), 2)

    if center_x == 0:
        cv2.putText(frame, "TIDAK TERDETEKSI", (20, 20),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        ser.write(b'S')
    elif center_x < 70:
        cv2.putText(frame, "KEKIRI", (20, 20),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        ser.write(b'L')
    elif center_x > 90:
        cv2.putText(frame, "KEKANAN", (20, 20),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        ser.write(b'R')
    else :
        cv2.putText(frame, "CENTER", (20, 20),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        ser.write(b'C')

    try:
        with open('data_share.json', 'r') as openfile:
            data_share = json.load(openfile)
            # print(data_share)
            cv2.putText(blackboard, "GPS      :" + str(data_share['gps']['lat']) + ", " + str(data_share['gps']['lon']), (10, 20),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            cv2.putText(blackboard, "Compass :" + data_share['compass'], (10, 50),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            cv2.putText(blackboard, "Speed    :" + data_share['speed'], (10, 80),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    except Exception as e:
        print('file tidak ada')
        

    cv2.line(frame, (70,0), (70,120), (0, 255, 0), 1)
    cv2.line(frame, (90,0), (90,120), (0, 255, 0), 1)

    # cv2.imshow('image', cv2.hconcat([frame, cv2.cvtColor(blackboard, cv2.COLOR_GRAY2BGR)]))
    # frame = cv2.resize(frame, (640, 480)) 
   
    cv2.imshow('image', frame )
    cv2.imshow('data', blackboard )
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()
