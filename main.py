import cv2
import numpy as np
import json
import serial
from geographiclib.geodesic import Geodesic
from threading import Thread, Event


try:
    with open('red_data_kalibrasi_1.json', 'r') as openfile:
        red_data_1 = json.load(openfile)
except Exception as e:
    print('data kalibrasi merah tidak ada')
    exit()
try:
    with open('red_data_kalibrasi_2.json', 'r') as openfile:
        red_data_2 = json.load(openfile)
except Exception as e:
    print('data kalibrasi merah tidak ada')
    exit()
try:
    with open('red_data_kalibrasi_3.json', 'r') as openfile:
        red_data_3 = json.load(openfile)
except Exception as e:
    print('data kalibrasi merah tidak ada')
    exit()

try:
    with open('green_data_kalibrasi_1.json', 'r') as openfile:
        green_data_1 = json.load(openfile)
except Exception as e:
    print('data kalibrasi merah tidak ada')
    exit()
try:
    with open('green_data_kalibrasi_2.json', 'r') as openfile:
        green_data_2 = json.load(openfile)
except Exception as e:
    print('data kalibrasi merah tidak ada')
    exit()
try:
    with open('green_data_kalibrasi_3.json', 'r') as openfile:
        green_data_3 = json.load(openfile)
except Exception as e:
    print('data kalibrasi merah tidak ada')
    exit()

try:
    ser = serial.Serial('/dev/ttyUSB0', 115200)
    ser.write(b'r')
except Exception as e:
    print(e)
    exit()

event = Event()
cap = cv2.VideoCapture(0)

frame_height = 240
frame_width = 320
cap.set(cv2.CAP_PROP_FRAME_WIDTH, frame_width)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, frame_height)

def detect(frame, mask, blur, text):
    blurMask = cv2.medianBlur(mask, blur)
    M = cv2.moments(blurMask)
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


def decode_gps(data):
    gps = data[3:]
    data_gps = gps.split('<>')
    lat = float(data_gps[0])
    lon = float(data_gps[1])
    return lat, lon


def get_range(lat1, lat2, long1, long2):
    return Geodesic.WGS84.Inverse(lat1, long1, lat2, long2)['s12']


def get_bearing(lat1, lat2, long1, long2):
    brng = Geodesic.WGS84.Inverse(lat1, long1, lat2, long2)['azi1']
    if brng < 0:
        brng = 360 + brng
    return brng


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
        if event.is_set():
            break



serial = Thread(target = thread_serial)
serial.start()

def vision_mission(part):
    while True:
        ret, frame = cap.read()
        blackboard = np.zeros((frame_height, 350, 1), dtype='uint8')

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        if part == 1:
            red_mask = cv2.inRange(hsv, np.array(red_data_1['min']),  np.array(red_data_1['max']))
            green_mask = cv2.inRange(hsv, np.array(green_data_1['min']),  np.array(green_data_1['max']))
        elif part == 2:
            red_mask = cv2.inRange(hsv, np.array(red_data_2['min']),  np.array(red_data_2['max']))
            green_mask = cv2.inRange(hsv, np.array(green_data_2['min']),  np.array(green_data_2['max']))
        elif part == 3:
            red_mask = cv2.inRange(hsv, np.array(red_data_3['min']),  np.array(red_data_3['max']))
            green_mask = cv2.inRange(hsv, np.array(green_data_3['min']),  np.array(green_data_3['max']))


        green_x, green_y = detect(frame, green_mask, 11, 'green')
        red_x, red_y = detect(frame, red_mask, 11, 'red')

        if red_x == 0 and green_x == 0:
            cap.release()
            cv2.destroyAllWindows()
            break
        elif red_x == 0 and green_x != 0:
            center_x = frame_width
        elif green_x == 0 and red_x != 0:
            center_x =0
        elif green_x < red_x:
            center_x = green_x + (red_x-green_x)/2
        elif red_x < green_x :
            center_x = red_x + (green_x-red_x)/2
        else :
            center_x = 0

        
        cv2.circle(frame, (int(center_x), int(frame_height/2)), 3, (255, 255, 0), 2)
        
        error_x = (center_x - int(frame_width/2)) * -1
        error_x_str = 'X' + str(error_x) + 'n'
        ser.write(error_x_str.encode())

        if center_x == 0:
            cv2.putText(frame, "TIDAK TERDETEKSI", (20, 20),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
    #         ser.write(b'S')
        elif center_x < frame_width/2 - 10:
            cv2.putText(frame, "KEKIRI", (20, 20),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
    #         ser.write(b'L')
        elif center_x > frame_width/2 + 10:
            cv2.putText(frame, "KEKANAN", (20, 20),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
    #         ser.write(b'R')
        else :
            cv2.putText(frame, "CENTER", (20, 20),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
    #         ser.write(b'C')

        try:
            with open('data_share.json', 'r') as openfile:
                data_share = json.load(openfile)
                # print(data_share)
                cv2.putText(blackboard, "GPS      :" + str(data_share['gps']['lat']) + ", " + str(data_share['gps']['lon']), (10, 20),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                cv2.putText(blackboard, "Compass :" + data_share['compass'], (10, 50),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                cv2.putText(blackboard, "Speed    :" + data_share['speed'], (10, 80),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        except Exception as e:
            print('file tidak ada')
            

        cv2.line(frame, (int(frame_width/2),0), (int(frame_width/2),frame_height), (0, 255, 0), 2)
        cv2.imshow('image', cv2.hconcat([frame, cv2.cvtColor(blackboard, cv2.COLOR_GRAY2BGR)]))
        if cv2.waitKey(1) & 0xFF == ord('q'):
            event.set()
            break
    
def waypoint(lat, lon):
    while True:
        data_ser = ser.readline().decode().strip()
        # print(data_ser)
        if 'GPS' in data_ser:
            _lat, _lon = decode_gps(data_ser)
            set_point = get_bearing(_lat, lat, _lon, lon)
            distance = get_range(_lat, lat, _lon, lon)
    #         print("GPS\t" + str(lat) + ", " + str(lon))
            if distance < 1 :
                ser.write(b'S')
                break

        if 'SPEED' in data_ser:
            pass
            
        if 'COMPASS' in data_ser:
            compass = int(data_ser[7:])
            if compass < 0:
                compass = 360 + compass
            # compass -= 25
            
        if compass != None and set_point != None:
            error = compass - set_point
            print("set|dstnc|cmps|err\t" + str(int(set_point)) + "\t" + str(int(distance)) + "\t" + str(compass) + "\t" + str(error) )
            error_x_str = 'X' + str(error) + 'n'
            ser.write(error_x_str.encode())
    #         time.sleep(0.1)
    #         if compass > set_point: 
    #             ser.write(b'L')
    #         elif compass < set_point:
    #             ser.write(b'R')
    #         else :
    #             ser.write(b'C')


    