# from pymavlink import mavutil 
import time
import collections
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative 
import serial
# import math 
# import numpy as np 
# import argparse 
# import imutils 
# import cv2

auv = 0
port_fc = '/dev/ttyACM1,57600'

print ("Connecting to FCU")
vehicle = connect(port_fc, wait_ready=True)
print ("Connected.. :)")


def mode_stabilize():
    vehicle.mode = VehicleMode("STABILIZE")
    print ("Stabilize")


#dari remote pure dari program
def mode_manual():
    vehicle.mode = VehicleMode("MANUAL")
    print ("MANUAL")

#tahan ketinggian
def mode_depth_hold():
    vehicle.mode = VehicleMode("ALT_HOLD")
    print ("ALT HOLD")

def arm_auv():
    vehicle.mode = VehicleMode("STABILIZE")
    time.sleep(2)
    while vehicle.mode.name != "STABILIZE":
        print("Waiting")
    ok = input("Tekan Enter Untuk Arming Motor")
    time.sleep(1)
    vehicle.armed = True # robot sudah ready
    while not vehicle.armed:
        print ("Tunggu...")
        time.sleep(1)
    print ("Persiapan menyelam...")
    time.sleep(1)

def nyelam(depth):
    if vehicle.mode.name != "STABILIZE":
        print("Wait...")
        vehicle.mode = VehicleMode("STABILIZE")
        time.sleep(1)
    else :
        None

    while (auv >= depth):
        vehicle.channels.overrides['3'] = 1300
        print(" Ch3 override: %s" % vehicle.channels.overrides['3'])
    vehicle.mode = VehicleMode("ALT_HOLD")
    vehicle.channels.overrides['3'] = None


def naik(depth):
    # if vehicle.mode.name != "ALT_HOLD":
    #     print("Waiting for Depth Hold mode")
    #     vehicle.mode = VehicleMode("ALT_HOLD")
    #     time.sleep(1)
    # else :
    #     None

    while (auv >= depth):
        vehicle.channels.overrides['3'] = 1600
        print(" Ch3 override: %s" % vehicle.channels.overrides['3'])
    
    vehicle.channels.overrides['3'] = None

def nyelamrc():
    # if vehicle.mode.name != "ALT_HOLD":
    #     print("Waiting for Depth Hold mode")
    #     vehicle.mode = VehicleMode("ALT_HOLD")
    #     time.sleep(1)
    # else :
    #     None

    vehicle.channels.overrides['3'] = 1300
    print(" Ch3 override: %s" % vehicle.channels.overrides['3'])
    time.sleep(0.2)

    vehicle.channels.overrides['3'] = None


def naikrc():
    if vehicle.mode.name != "ALT_HOLD":
        print("Waiting for Depth Hold mode")
        vehicle.mode = VehicleMode("ALT_HOLD")
        time.sleep(1)
    else :
        None

    vehicle.channels.overrides['3'] = 1600
    print(" Ch3 override: %s" % vehicle.channels.overrides['3'])
    time.sleep(0.2)
    vehicle.channels.overrides['3'] = None

        
def maju(waktu):
    # if vehicle.mode.name != "ALT_HOLD":
    # if vehicle.mode.name != "MANUAL":
    #     print("Waiting for Depth Hold mode")
    #     vehicle.mode = VehicleMode("ALT_HOLD")
    #     time.sleep(1)
    # else :
    #     None
    
    vehicle.channels.overrides['5'] = 1600
    print(" Ch5 override: %s" % vehicle.channels.overrides['5'])
    time.sleep(waktu)
    vehicle.channels.overrides['5'] = 1500
    vehicle.channels.overrides['5'] = None

def kiri(waktu):
    # if vehicle.mode.name != "ALT_HOLD":
    #     print("Waiting for Depth Hold mode")
    #     vehicle.mode = VehicleMode("ALT_HOLD")
    #     time.sleep(1)
    # else :
    #     None
    
    vehicle.channels.overrides['6'] = 1400
    print(" Ch6 override: %s" % vehicle.channels.overrides['6'])
    time.sleep(waktu)
    vehicle.channels.overrides['6'] = None

def kanan(waktu):
    # if vehicle.mode.name != "ALT_HOLD":
    #     print("Waiting for Depth Hold mode")
    #     vehicle.mode = VehicleMode("ALT_HOLD")
    #     time.sleep(1)
    # else :
    #     None
    
    vehicle.channels.overrides['6'] = 1600
    print(" Ch6 override: %s" % vehicle.channels.overrides['6'])
    time.sleep(waktu)
    vehicle.channels.overrides['6'] = None

def mundur(waktu):
    # if vehicle.mode.name != "ALT_HOLD":
    #     print("Waiting for Depth Hold mode")
    #     vehicle.mode = VehicleMode("ALT_HOLD")
    #     time.sleep(1)
    # else :
    #     None
    
    vehicle.channels.overrides['5'] = 1400
    print(" Ch5 override: %s" % vehicle.channels.overrides['5'])
    time.sleep(waktu)
    vehicle.channels.overrides['5'] = None

def yaw_kiri(waktu):
    # if vehicle.mode.name != "ALT_HOLD":
    #     print("Waiting for Depth Hold mode")
    #     vehicle.mode = VehicleMode("ALT_HOLD")
    #     time.sleep(1)
    # else :
    #     None
    
    
    vehicle.channels.overrides['4'] = 1400
    print(" Ch4 override: %s" % vehicle.channels.overrides['4'])
    time.sleep(waktu)
    vehicle.channels.overrides['4'] = None

def yaw_kanan(waktu):
    # if vehicle.mode.name != "ALT_HOLD":
    #     print("Waiting for Depth Hold mode")
    #     vehicle.mode = VehicleMode("ALT_HOLD")
    # else :
    #     None
    
    vehicle.channels.overrides['4'] = 1600
    print(" Ch4 override: %s" % vehicle.channels.overrides['4'])
    time.sleep(waktu)
    vehicle.channels.overrides['4'] = 1500
    vehicle.channels.overrides['4'] = None
    # time.sleep(1)


def arm_and_takeoff(altitude):
    # while not vehicle.is_armable:
    #     print("waiting to be armable")
    #     time.sleep(1)

    print("Arming motors")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed: time.sleep(1)

    print("Taking Off")
    vehicle.simple_takeoff(altitude)

    while True:
        v_alt = vehicle.location.global_relative_frame.alt
        print(">> Altitude = %.1f m"%v_alt)
        if v_alt >= altitude:
            print("Target Depth reached")
            break
        time.sleep(1)
    vehicle.mode = VehicleMode("ALT_HOLD")

def set_velocity_body(vehicle, vx, vy, vz):
    """ Remember: vz is positive downward!!!
    http://ardupilot.org/dev/docs/copter-commands-in-guided-mode.html
    
    Bitmask to indicate which dimensions should be ignored by the vehicle 
    (a value of 0b0000000000000000 or 0b0000001000000000 indicates that 
    none of the setpoint dimensions should be ignored). Mapping: 
    bit 1: x,  bit 2: y,  bit 3: z, 
    bit 4: vx, bit 5: vy, bit 6: vz, 
    bit 7: ax, bit 8: ay, bit 9:
    
    
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
            0,
            0, 0,
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            0b0000111111000111, #-- BITMASK -> Consider only the velocities
            0, 0, 0,        #-- POSITION
            vx, vy, vz,     #-- VELOCITY
            0, 0, 0,        #-- ACCELERATIONS
            0, 0)
    vehicle.send_mavlink(msg)
    vehicle.flush()

def goto_position_target_local_ned(north, east, down):
    """
    Send SET_POSITION_TARGET_LOCAL_NED command to request the vehicle fly to a specified
    location in the North, East, Down frame.
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
        0b0000111111111000, # type_mask (only positions enabled)
        north, east, down,
        0, 0, 0, # x, y, z velocity in m/s  (not used)
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
    # send command to vehicle
    vehicle.send_mavlink(msg)


    #___________________________________________________________________
    #___________________________________________________________________
    #___________________________________________________________________
    #___________________________________________________________________
    
def maju_mundur(rc):    
    vehicle.channels.overrides['5'] = rc
    print(" Ch5 override: %s" % vehicle.channels.overrides['5'])


def kanan_kiri(rc):
    vehicle.channels.overrides['6'] = rc
    print(" Ch6 override: %s" % vehicle.channels.overrides['6'])

def yaw(rc):
    vehicle.channels.overrides['4'] = rc
    print(" Ch4 override: %s" % vehicle.channels.overrides['4'])

def menujuLokasi(lat, lon):
    a_location = LocationGlobalRelative(lat, lon)
    vehicle.simple_goto(a_location)
    vehicle.attitude.pitch