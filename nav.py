# Library containing methods for navigation of drones to given waypoints

import json
from math import ceil

class Nav:
    def __init__(self, waypoint_list_path, numb_waypoints=50):
        # Load waypoints for lead to follow and chase to receive
        self.waypoints = json.load(open(waypoint_list_path))
        self.primary_route = None
        self.numb_waypoints = numb_waypoints
        self.default_velocity = 5
        self.x_distance_increment = None
        self.y_distance_increment = None
        self.z_distance_increment = None

    def build_path(self):
        start_point = self.waypoints["points"][0] # array [x, y, z, vel]
        end_point = self.waypoints["points"][1] # array [x, y, z, vel]
        self.calculate_distances(start_point, end_point)
        self.primary_route = [start_point]
        print(self.x_distance_increment)
        print(self.y_distance_increment)
        print(self.z_distance_increment)
        for i in range(1, self.numb_waypoints):
            previous_point = self.primary_route[i-1]
            self.primary_route.append([previous_point[0] + self.x_distance_increment,
                                       previous_point[1] + self.y_distance_increment,
                                       previous_point[2] + self.z_distance_increment,
                                       self.default_velocity])
            #print(i)

    def calculate_distances(self, start_point, end_point):
        x_distance = Nav.find_distance(0, start_point, end_point)
        print(x_distance)
        self.x_distance_increment = Nav.find_distance_increment(x_distance, self.numb_waypoints)
        y_distance = Nav.find_distance(1, start_point, end_point)
        print(y_distance)
        self.y_distance_increment = Nav.find_distance_increment(y_distance, self.numb_waypoints)
        z_distance = Nav.find_distance(2, start_point, end_point) * -1
        print(z_distance)
        self.z_distance_increment = Nav.find_distance_increment(z_distance, self.numb_waypoints) * -1

    def find_distance_increment(distance, numb_waypoints):
        return abs(distance/numb_waypoints)

    def find_distance(coordinate_index, start_point, end_point):
        return abs(abs(end_point[coordinate_index]) - abs(start_point[coordinate_index]))

if __name__ == "__main__":
    new_nav = Nav('waypoints.json')
    new_nav.build_path()
    print(len(new_nav.primary_route))
    print(new_nav.primary_route[0])
    print(new_nav.primary_route[49])
    