import setup_path 
import airsim

import numpy as np
import os
import tempfile
import pprint
import cv2
import time
from nav import Nav

veh_1_name = 'Lead'
veh_2_name = 'Follow'
waypoints_path_list = 'waypoints.json'
CONFIRMATION_DISTANCE = 1.5

# Load route
route = Nav(waypoints_path_list)
route.build_path()
print(len(route.primary_route))
print(route.primary_route[0])
print(route.primary_route[route.numb_waypoints - 1])

# connect to the AirSim simulator
client = airsim.MultirotorClient()
print(client)
client.confirmConnection()
print('Connection Confirmed')
client.enableApiControl(True, veh_1_name)
client.enableApiControl(True, veh_2_name)
client.armDisarm(True, veh_1_name)
client.armDisarm(True, veh_2_name)

airsim.wait_key('Press any key to takeoff')
f1 = client.takeoffAsync(vehicle_name=veh_1_name)
f2 = client.takeoffAsync(vehicle_name=veh_2_name)
f1.join()
f2.join()

state = client.getMultirotorState(vehicle_name=veh_1_name)
comms_data = client.getCommunicationsData(vehicle_name=veh_1_name)
print(veh_1_name, "\n")
print("State: %s" % pprint.pformat(state))
print("Comms Data: %s" % pprint.pformat(comms_data))

state = client.getMultirotorState(vehicle_name=veh_2_name)
print(veh_2_name, "\n")
print("State: %s" % pprint.pformat(state))

airsim.wait_key('Press any key to begin flying the route:')

# Fly the waypoint route. Ensure that the second vehicle triggers the join()
# so as the halt the loop.
for i, waypoint in enumerate(route.primary_route):
    try:
        next_point = route.primary_route[i + 1]
    except Exception as error:
        print('Final point!')
        next_point = False
    f1 = client.moveToPositionAsync(waypoint[0], waypoint[1], waypoint[2], waypoint[3], vehicle_name=veh_1_name)
    if (i % 10 == 0):
        f2 = client.moveToPositionAsync(waypoint[0], waypoint[1], waypoint[2] - 0.1, waypoint[3] + 1, vehicle_name=veh_2_name)
    else:
        f2 = client.moveToPositionAsync(waypoint[0], waypoint[1], waypoint[2], waypoint[3], vehicle_name=veh_2_name)

    if (i % 5 == 0):
        print(waypoint)

    # Instead of join, pull state and get GPS data to confirm where the drone is at
    kinematics = client.simGetGroundTruthKinematics(vehicle_name=veh_1_name)
    if next_point:
        # For the first point, the drones need to fly to a normalized location
        if i == 0:
            f2.join()
        else:
            while (abs(kinematics.position.x_val - abs(next_point[0])) - 3) > CONFIRMATION_DISTANCE and (abs(kinematics.position.y_val - abs(next_point[1])) - 3) > CONFIRMATION_DISTANCE and abs(abs(kinematics.position.z_val) - abs(next_point[2])) > CONFIRMATION_DISTANCE:
                kinematics = client.simGetGroundTruthKinematics(vehicle_name=veh_1_name)
                print('\n')
                print(kinematics.position.x_val)
                print(kinematics.position.y_val)
                print(kinematics.position.z_val)
                print((abs(kinematics.position.x_val - abs(next_point[0])) - 3))
                print((abs(kinematics.position.y_val - abs(next_point[0])) - 3))
                print(abs(abs(kinematics.position.z_val) - abs(next_point[2])))
                print('\n')
                time.sleep(0.2)
       
   # f1.join()

client.hoverAsync().join()

airsim.wait_key('Press any key to reset to original state')

client.armDisarm(False, veh_1_name)
client.armDisarm(False, veh_2_name)
client.reset()

# that's enough fun for now. let's quit cleanly
client.enableApiControl(False, veh_1_name)
client.enableApiControl(False, veh_2_name)
