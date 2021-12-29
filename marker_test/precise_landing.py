# -*- coding: utf-8 -*-
"""
Created on Tue Dec 28 15:51:58 2021

@author: barun
"""


from os import sys, path
sys.path.append(path.dirname(path.dirname(path.abspath(__file__))))

import time
import math
import argparse


from dronekit import connect, VehicleMode, LocationGlobalRelative, Command, LocationGlobal
from pymavlink import mavutil
from lib_aruco_pose import *
import numpy as np

parser = argparse.ArgumentParser()
parser.add_argument('--connect', default = '')
args = parser.parse_args()
    
#--------------------------------------------------
#-------------- FUNCTIONS  
#--------------------------------------------------    

def get_location_metres(original_location, dNorth, dEast):
    earth_radius=6378137.0 #Radius of "spherical" earth
    #Coordinate offsets in radians
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))
    
    print("dlat, dlon", dLat, dLon)

    #New position in decimal degrees
    newlat = original_location.lat + (dLat * 180/math.pi)
    newlon = original_location.lon + (dLon * 180/math.pi)
    return(newlat, newlon)

#--Estimate the LOS(line of sight) angles to the marker   
def marker_position_to_angle(x, y, z): 
    angle_x = math.atan2(x,z)
    angle_y = math.atan2(y,z)
    return (angle_x, angle_y)

#--Camera to UAV frame: camera has X right and Y down. UAV has X forward, Y right.  
def camera_to_uav(x_cam, y_cam):
    x_uav =-y_cam
    y_uav = x_cam
    return(x_uav, y_uav)
 
#--UAV to NE reference frame   
def uav_to_ne(x_uav, y_uav, yaw_rad):
    c = math.cos(yaw_rad)
    s = math.sin(yaw_rad)  
    north   = x_uav*c - y_uav*s
    east    = x_uav*s + y_uav*c 
    return(north, east)
  
def check_angle_descend(angle_x, angle_y, angle_desc):
    return(math.sqrt(angle_x**2 + angle_y**2) <= angle_desc)
        
#--------------------------------------------------
#-------------- CONNECTION  
#--------------------------------------------------    
#-- Connect to the vehicle
#print('Connecting...')
#vehicle = connect(args.connect)  
#vehicle = connect('/dev/ttyAMA1', baud=115200) 
#vehicle = connect('/dev/serial0', baud=115200)
vehicle = connect('192.168.10.50:14551', wait_ready=True)
#vehicle = connect('192.168.231.240:14551', wait_ready=True)
print("Mode: %s" % vehicle.mode.name)

def arm_and_takeoff(aTargetAltitude):
    while not vehicle.is_armable:
        print("Waiting for vehicle to become armable")
        time.sleep(1)
    vehicle.mode = VehicleMode("GUIDED")
    while vehicle.mode!="GUIDED":
        print("Waiting for vehicle to enter GUIDED mode")
        time.sleep(1)
    print("Vehicle mode changed to GUIDED")   
    #Arm vehicle once GUIDED mode is confirmed
    vehicle.armed=True
    while vehicle.armed==False:
        print("Waiting for vehicle to become armed.")
        time.sleep(1)
    print("Vehicle is now ARMED")     

    vehicle.simple_takeoff(aTargetAltitude)

    while True:
        print("Current Altitude: %d"%vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt>=aTargetAltitude*.95:
            break
        time.sleep(1)
    print("Target altitude reached")
    return None

vehicle.mode=VehicleMode("GUIDED")
time.sleep(1)
arm_and_takeoff(7) 
#vehicle.airspeed = 5

#--------------------------------------------------
#-------------- PARAMETERS  
#-------------------------------------------------- 
rad_2_deg   = 180.0/math.pi
deg_2_rad   = 1.0/rad_2_deg 

#--------------------------------------------------
#-------------- LANDING MARKER  
#--------------------------------------------------    
#--- Define Tag
#id_to_find      = 72 #################################################################
id_to_find      = 317
marker_size     = 20 #[cm] ######################################################################
freq_send       = 1 #- Hz

#land_alt_cm         = 25.0  #[cm]
land_alt_cm         = 100.0  #[cm]
#angle_descend       = 20*deg_2_rad  #[rad]
angle_descend       = 10*deg_2_rad  #[rad]
land_speed_cms      = 30.0  #[cm/s]
#land_speed_cms      = 20.0  #[cm/s]


#--- Get the camera calibration path
# Find full directory path of this script, used for loading  config and other files
cwd                 = path.dirname(path.abspath(__file__))
#calib_path          = calib_path  = "/home/pi/marker_test/camera_01/"
calib_path          = calib_path  = "/home/pi/marker_test/camera_pi/"
camera_matrix       = np.loadtxt(calib_path+'cameraMatrix.txt', delimiter=',')
camera_distortion   = np.loadtxt(calib_path+'cameraDistortion.txt', delimiter=',')                                      
aruco_tracker       = ArucoSingleTracker(id_to_find=id_to_find, marker_size=marker_size, show_video=False, 
                camera_matrix=camera_matrix, camera_distortion=camera_distortion)
                
#-- Save the time                
time_0 = time.time()

# RC override incase mode changes
"""
import threading
class RC_Override(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self.daemon = True
    def run(self):
        while True:
            try:
                if vehicle.mode == "LOITER" or vehicle.mode =="STABILIZE" or vehicle.mode =="ALT_HOLD":
                    print("Vehicle mode changed to " + str(vehicle.mode).split(':')[-1])
                    print("RC Overriding")
                    vehicle.channels.overrides['3'] = 1500
                    time.sleep(1)  
            except:
                pass

rc_override = RC_Override()
rc_override.start()
"""

while True:         
    #-- Find the marker
    marker_found, x_cm, y_cm, z_cm = aruco_tracker.track(loop=False)
    if marker_found:  
        x_cm, y_cm  = camera_to_uav(x_cm, y_cm)  #-- convert to UAV frame
        uav_location = vehicle.location.global_relative_frame 
        
        #-- If high altitude, use barometric altitude rather than visual
        if uav_location.alt >= 5.0:
            print("-------------Higher than 2 meters---------------");
            z_cm = uav_location.alt*100.0
        
        #-- convert the marker location to angles    
        angle_x, angle_y  = marker_position_to_angle(x_cm, y_cm, z_cm)

        #-- send the message
        if time.time() >= time_0 + 1.0/freq_send:
            time_0 = time.time()
            
            print("Altitude = %.0fcm"%z_cm)
            print("Marker found x = %5.0f cm  y = %5.0f cm -> angle_x = %5f  angle_y = %5f"%(x_cm, y_cm, angle_x*rad_2_deg, angle_y*rad_2_deg))
            
            #--Convert to North East
            north, east = uav_to_ne(x_cm, y_cm, vehicle.attitude.yaw)
            print("Marker N = %5.0f cm   E = %5.0f cm   Yaw = %.0f deg"%(north, east, vehicle.attitude.yaw*rad_2_deg))
            
            marker_lat, marker_lon  = get_location_metres(uav_location, north*0.01, east*0.01) 
             
            #-- If angle is good, descend
            if check_angle_descend(angle_x, angle_y, angle_descend):
                print("Low error: descending")
                location_marker = LocationGlobalRelative(marker_lat, marker_lon, uav_location.alt-(land_speed_cms*0.01/freq_send))
            else:
                location_marker = LocationGlobalRelative(marker_lat, marker_lon, uav_location.alt)
                
            vehicle.simple_goto(location_marker)
            print("UAV Location    Lat = %.7f  Lon = %.7f"%(uav_location.lat, uav_location.lon))
            print("Commanding to   Lat = %.7f  Lon = %.7f"%(location_marker.lat, location_marker.lon))
            
        #--- Command to land
        if z_cm <= land_alt_cm:
            if vehicle.mode == "GUIDED":
                print (" -->>COMMANDING TO LAND<<")
                vehicle.mode = "LAND"
                time.sleep(2)
                
                print("Vehicle mode = " + str(vehicle.mode).split(':')[-1])
                print (" ----------->>Exiting<<--------")
                vehicle.close()
                sys.exit(0)  
                
    if not marker_found:
        uav_location = vehicle.location.global_relative_frame 
        go_higher = LocationGlobalRelative(uav_location.lat, uav_location.lon, uav_location.alt+0.5)
        
        if uav_location.alt > 5.0:
            print("-------Marker not found-------") 
            time.sleep(1)
        else:
            print("--Going 0.5 meter higher looking for the Marker--")
            vehicle.simple_goto(go_higher)
            time.sleep(1)
 