import setup_path
import airsim
import pprint
import time

from nav import Nav
from airsim import KinematicsState


DEBUG = False


def calculate_position_difference(kinematics: KinematicsState, next_point: list):
    x_diff = float((abs(kinematics.position.x_val - abs(next_point[0]))))
    y_diff = float(abs(kinematics.position.y_val - abs(next_point[1])))
    z_diff = float(abs(abs(kinematics.position.z_val) - abs(next_point[2])))
    return x_diff, y_diff, z_diff


def print_kinematics_information(kinematics: KinematicsState, next_point: list, drone_name: str) -> None:
    x_diff, y_diff, z_diff = calculate_position_difference(kinematics, next_point)
    print('\n')
    print("Drone Name: {}".format(drone_name))
    print("X: {}".format(kinematics.position.x_val))
    print("Y: {}".format(kinematics.position.y_val))
    print("Z: {}".format(kinematics.position.z_val))
    print("Distance to Next Point: {} {} {} m".format(x_diff, y_diff, z_diff))
    print('\n')


def determine_if_at_position(kinematics: KinematicsState, next_point: list) -> bool:
    x_diff, y_diff, z_diff = calculate_position_difference(kinematics, next_point)
    return (x_diff <= CONFIRMATION_DISTANCE
            and y_diff <= CONFIRMATION_DISTANCE
            and z_diff <= CONFIRMATION_DISTANCE)


veh_1_name = 'Lead'
veh_2_name = 'Follow'
waypoints_path_list = 'waypoints.json'
CONFIRMATION_DISTANCE = 3.0 # Meters

# Load route
route = Nav(waypoints_path_list)
route.build_path()

print("Route was generated with {} waypoints".format(len(route.primary_route)))
print("First Point [X, Y, Z, Vel]: {}".format(route.primary_route[0]))
print("Last Point [X, Y, Z, Vel]: {}".format(route.primary_route[49]))

airsim.wait_key('If this is correct, press any key to run AirSim')

# connect to the AirSim simulator
client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True, veh_1_name)
client.enableApiControl(True, veh_2_name)
client.armDisarm(True, veh_1_name)
client.armDisarm(True, veh_2_name)

airsim.wait_key('Press any key to takeoff')
f1 = client.takeoffAsync(vehicle_name=veh_1_name)
f2 = client.takeoffAsync(vehicle_name=veh_2_name)
f1.join()
f2.join()

if DEBUG:
    state = client.getMultirotorState(vehicle_name=veh_1_name)
    print(veh_1_name, "\n")
    print("State: %s" % pprint.pformat(state))


    state = client.getMultirotorState(vehicle_name=veh_2_name)
    print(veh_2_name, "\n")
    print("State: %s" % pprint.pformat(state))

airsim.wait_key('Press any key to begin flying the route:')

# Fly the waypoint route. Ensure that the second vehicle triggers the join()
# so as the halt the loop.
for i, waypoint in enumerate(route.primary_route):
    if (i + 1) < len(route.primary_route):
        next_point = route.primary_route[i + 1]
    else:
        print('Final point!')
        next_point = False
    client.moveToPositionAsync(
        waypoint[0], waypoint[1], waypoint[2], waypoint[3], vehicle_name=veh_1_name)
    if (i % 10 == 0):
        f2 = client.moveToPositionAsync(
            waypoint[0], waypoint[1], waypoint[2] - 0.1, waypoint[3] + 1, vehicle_name=veh_2_name)
    else:
        f2 = client.moveToPositionAsync(
            waypoint[0], waypoint[1], waypoint[2], waypoint[3], vehicle_name=veh_2_name)

    if (i % 5 == 0):
        print("Current waypoint: {}".format(waypoint))

    # Instead of join, pull state and get GPS data to confirm where the drone is at
    kinematics = client.simGetGroundTruthKinematics(vehicle_name=veh_1_name)
    if next_point:
        # For the first point, the drones need to fly to a normalized location
        if i == 0:
            f2.join()
        else:
            at_position = False
            while not at_position:
                kinematics = client.simGetGroundTruthKinematics(
                    vehicle_name=veh_1_name)
                at_position = determine_if_at_position(kinematics, next_point)
                print_kinematics_information(kinematics, next_point)
                time.sleep(0.1)


client.hoverAsync(vehicle_name=veh_1_name).join()
client.hoverAsync(vehicle_name=veh_2_name).join()

airsim.wait_key('Press any key to reset to original state')

client.armDisarm(False, veh_1_name)
client.armDisarm(False, veh_2_name)
client.reset()

# that's enough fun for now. let's quit cleanly
client.enableApiControl(False, veh_1_name)
client.enableApiControl(False, veh_2_name)
