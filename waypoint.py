from geographiclib.geodesic import Geodesic
import serial
import cv2

waypoints = [
    {
        'lat' :-7.314236862206032,  
        'lon' :112.7257244158201
    },
    {
        'lat' :-7.314364560590797,   
        'lon' :112.72614887540927
    },
    {
        'lat' :-7.314158380888935,    
        'lon' :112.72622263615935
    },
    {
        'lat' :-7.313904979250582,     
        'lon' :112.72602817599979
    },
    {
        'lat' :-7.313954196377295,      
        'lon' :112.725771354479
    },
    
]

try:
    ser = serial.Serial('/dev/ttyUSB0', 115200)
except Exception as e:
    print(e)
    exit()


def get_range():
    return Geodesic.WGS84.Inverse(lat1, long1, lat2, long2)['s12']


def get_bearing(lat1, lat2, long1, long2):
    brng = Geodesic.WGS84.Inverse(lat1, long1, lat2, long2)['azi1']
    return brng

def decode_gps(data):
    gps = data[3:]
    data_gps = gps.split('<>')
    lat = float(data_gps[0])
    lon = float(data_gps[1])
    return lat, lon

index = 0

while True:
    if index == len(waypoints) - 1:
        exit()

    data_ser = ser.readline().decode().strip()
    # print(data_ser)
    if 'GPS' in data_ser:
        lat, lon = decode_gps(data_ser)
        set_point = get_bearing(lat, waypoints[index]['lat'], lon, waypoints[index]['lon'])
        distance = get_range(lat, waypoints[index]['lat'], lon, waypoints[index]['lon'])
        print("set\t" + str(set_point))
        print("dstnc\t" + str(distance))

    if 'SPEED' in data_ser:
        pass
        
    if 'COMPASS' in data_ser:
        compass = data_ser[7:]
        # compass -= 25
        
        print("Compass\t" + str(compass))
        
    if compass != None and set_point != None:
        if compass > set_point: 
            ser.write(b'L')
        else if compass < set_point:
            ser.write(b'R')
        else :
            ser.write(b'C')