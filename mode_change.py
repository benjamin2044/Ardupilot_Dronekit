####Dependencies###################

from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
import socket
import argparse

###Function definitions for mission####

##Function to connect script to drone
def connectMyCopter():
    parser = argparse.ArgumentParser(description='commands')
    parser.add_argument('--connect')
    args = parser.parse_args()

    connection_string = args.connect

    #vehicle = connect(connection_string, wait_ready=True)
    vehicle = connect("192.168.10.50:14551", wait_ready=True)

    return vehicle


#### Mission################################

vehicle=connectMyCopter()
print("Connected..")
print(vehicle.mode)

time.sleep(5)

while True: 
    vehicle.channels.overrides['3'] = 1500
    time.sleep(1)
    if vehicle.mode.name=='LAND' or vehicle.mode.name=='RTL':
        break

vehicle.close()
### End of script
