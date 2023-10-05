from geographiclib.geodesic import Geodesic
import serial
import cv2
import time

waypoints = [
    {
        'lat' :-7.3141167727750895, 
        'lon' :112.72600272647817 
    },
    
    
]

compass = None
set_point = None

try:
    ser = serial.Serial('/dev/ttyUSB0', 115200)
except Exception as e:
    print(e)
    exit()


def get_range(lat1, lat2, long1, long2):
    return Geodesic.WGS84.Inverse(lat1, long1, lat2, long2)['s12']


def get_bearing(lat1, lat2, long1, long2):
    brng = Geodesic.WGS84.Inverse(lat1, long1, lat2, long2)['azi1']
    if brng < 0:
        brng = 360 + brng
    return brng

def decode_gps(data):
    gps = data[3:]
    data_gps = gps.split('<>')
    lat = float(data_gps[0])
    lon = float(data_gps[1])
    return lat, lon

index = 0

while True:
    if index == len(waypoints):
        exit()

    data_ser = ser.readline().decode().strip()
    # print(data_ser)
    if 'GPS' in data_ser:
        lat, lon = decode_gps(data_ser)
        set_point = get_bearing(lat, waypoints[index]['lat'], lon, waypoints[index]['lon'])
        distance = get_range(lat, waypoints[index]['lat'], lon, waypoints[index]['lon'])
#         print("GPS\t" + str(lat) + ", " + str(lon))

        if distance < 5 :
            ser.write(b'S')
            exit()
    if 'SPEED' in data_ser:
        pass
        
    if 'COMPASS' in data_ser:
        compass = int(data_ser[7:])
        if compass < 0:
            compass = 360 + compass
        # compass -= 25
        
#         print("Compass\t" + str(compass))
        
    if compass != None and set_point != None:
        error = compass - set_point
        print("set|dstnc|cmps|err\t" + str(int(set_point)) + "\t" + str(int(distance)) + "\t" + str(compass) + "\t" + str(error) )
        error_x_str = 'X' + str(error) + 'n'
#     print(error_x_str)
        ser.write(error_x_str.encode())
#         time.sleep(0.1)
#         if compass > set_point: 
#             ser.write(b'L')
#         elif compass < set_point:
#             ser.write(b'R')
#         else :
#             ser.write(b'C')
            