#!/usr/bin/env python
# -*- coding: utf-8 -*-

import time, math
from dronekit import connect, VehicleMode, LocationGlobalRelative

# Connect to the Vehicle
print('Connecting to vehicle')
vehicle = connect("192.168.10.140:55555", wait_ready=True)
#vehicle = connect("/dev/ttyAMA1", wait_ready=True, baud=115200)
####################################################################################
#print(vehicle.rangefinder)
time.sleep(1)
#print("GPS: %s" % vehicle.gps_0)
time.sleep(1)
print(vehicle.location.global_relative_frame)
time.sleep(1)
#print(" Heading: %s" % vehicle.heading)

home_lat = vehicle.location.global_relative_frame.lat
home_lon = vehicle.location.global_relative_frame.lon
print(home_lat)
print(home_lon)
################### Funtctions ##############################################################
def arm_and_takeoff(aTargetAltitude):
    print("Basic pre-arm checks")
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)
    print("Arming motors")    
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True
    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)
    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)

# distance between two points       
def get_distance_meters(Loc1_lat, Loc1_lon, Loc2_lat, Loc2_lon):
    dlat = Loc2_lat - Loc1_lat
    dlong = Loc2_lon - Loc1_lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5      
################Arm and Takeoff################################################
arm_and_takeoff(15) # 10 meters
print("Set default/target airspeed to 5")
vehicle.airspeed = 5
################ Point 1#######################
print("Going towards point 1 ...")
point1_lat, point1_lon, point1_alt = -35.362959, 149.163624, 20
point1 = LocationGlobalRelative(point1_lat, point1_lon, point1_alt)
vehicle.simple_goto(point1)

time.sleep(10)
print("Ground Speed %s" %vehicle.groundspeed)
dist1 = get_distance_meters(point1_lat, point1_lon, home_lat, home_lon)
print("Distance = {}" .format(dist1))
req_time = dist1/vehicle.groundspeed
print("Required time = {}" .format(req_time))

time.sleep(abs(req_time - 5))

print(vehicle.location.global_relative_frame) #Lat, lon, alt
print("Heading: %s" % vehicle.heading)
print("------------------------------------------------------------")     
####################### Point 2#######################
print("Going towards point 2...")
point2_lat, point2_lon, point2_alt = -35.361791, 149.165008, 30
point2 = LocationGlobalRelative(point2_lat, point2_lon, point2_alt)
vehicle.simple_goto(point2, groundspeed=10)

time.sleep(10)
print("Ground Speed %s" %vehicle.groundspeed)
dist2 = get_distance_meters(point2_lat, point2_lon, point1_lat, point1_lon)
print("Distance = {}" .format(dist2))
req_time = dist2/vehicle.groundspeed
print("Required time = {}" .format(req_time))

time.sleep(abs(req_time - 5))

print(vehicle.location.global_relative_frame) #Lat, lon, alt
print("Heading: %s" % vehicle.heading)
print("------------------------------------------------------------")
####################### Point 3#######################
print("Going towards point 3...")
point3_lat, point3_lon, point3_alt = -35.363121, 149.166880, 20
point3 = LocationGlobalRelative(point3_lat, point3_lon, point3_alt)
vehicle.simple_goto(point3, groundspeed=5)

time.sleep(10)
print("Ground Speed %s" %vehicle.groundspeed)
dist3 = get_distance_meters(point2_lat, point2_lon, point3_lat, point3_lon)
print("Distance = {}" .format(dist3))
req_time = dist3/vehicle.groundspeed
print("Required time = {}" .format(req_time))

time.sleep(abs(req_time - 5))

print(vehicle.location.global_relative_frame) #Lat, lon, alt
print("Heading: %s" % vehicle.heading)
print("------------------------------------------------------------")
####################### Home Point#######################

print("Coming towards Home Point")
home = LocationGlobalRelative(home_lat, home_lon, 6)
vehicle.simple_goto(home, groundspeed=5)

time.sleep(10)
print("Ground Speed %s" %vehicle.groundspeed)
dist_home = get_distance_meters(home_lat, home_lon, point3_lat, point3_lon)
print("Distance = {}" .format(dist_home))
req_time = dist_home/vehicle.groundspeed
print("Required time = {}" .format(req_time))

time.sleep(abs(req_time - 5))
########################RTL#######################
print("Returning to Launch")
#vehicle.mode = VehicleMode("RTL")
#vehicle.mode = VehicleMode("LAND")
########################################################

# Close vehicle object before exiting script
print("Close vehicle object")
vehicle.close()

