# Library containing methods for navigation of drones to given waypoints

import json
from math import ceil

class Nav():
    DEBUG = True

    def __init__(self, waypoint_list_path: str, numb_waypoints: int = 50) -> None:
        """
            Core navigation Class that generates a route given a set of
            way points. In general, the number of waypoints determines 
            the overall trajectory of the agent. A linear path is 
            currently supported, with work on curves, dubbin paths, etc. 
            to follow on in the future.

            ## Args:
            - waypoint_list_path [string] - path to the json file that
                                            contains the waypoints
            - numb_waypoints [integer] - the number of intermediary
                                         waypoints to create. Default is
                                         50 waypoints.
        """
        try:
            with open(waypoint_list_path, "r") as file:
                self.waypoints = json.load(file)
        except Exception as error:
            print("There was an issue loading the waypoints")
            exit()
        self.primary_route = None
        self.numb_waypoints = numb_waypoints
        self.default_velocity = 5.0 # m / s
        self.x_distance_increment = None
        self.y_distance_increment = None
        self.z_distance_increment = None

    def build_path(self):
        """
            Given 2 waypoints, start and end, generate a linear path
            that is broken into segments of a specific distance
            incremement. Currently, the only supported path generation
            is a linear path of points.

            ## Inputs:
            - None

            ## Outputs:
            - Primary Route attribute with list of "numb_waypoints"
              [X, Y, Z, Vel] waypoints in the NED coordinate frame
        """
        print("Generating path with {} intermediary waypoints.".format(
            self.numb_waypoints
        ))
        # array [x, y, z, vel]
        start_point = self.waypoints["points"][0]
        # array [x, y, z, vel]
        end_point = self.waypoints["points"][1]
        self.calculate_distances(start_point, end_point)
        self.primary_route = [start_point]
        if self.DEBUG:
            print("X Distance Increment: {}".format(self.x_distance_increment))
            print("Y Distance Increment: {}".format(self.y_distance_increment))
            print("Z Distance Increment: {}".format(self.z_distance_increment))
        for i in range(1, self.numb_waypoints):
            previous_point = self.primary_route[i-1]
            self.primary_route.append(
                [previous_point[0] + self.x_distance_increment,
                 previous_point[1] + self.y_distance_increment,
                 previous_point[2] + self.z_distance_increment,
                 self.default_velocity]
            )
        print("Route generation has been completed.")

    def calculate_distances(self, start_point: list, end_point: list):
        x_distance = Nav.find_difference_in_distance(
                        start_point[0],
                        end_point[0]
                    )
        self.x_distance_increment = Nav.find_distance_increment(
                                        x_distance,
                                        self.numb_waypoints
                                    )

        y_distance = Nav.find_difference_in_distance(
                        start_point[1],
                        end_point[1]
                    )
        self.y_distance_increment = Nav.find_distance_increment(
                                        y_distance,
                                        self.numb_waypoints
                                    )

        z_distance = (Nav.find_difference_in_distance(
                        start_point[2],
                        end_point[2]
                    ) * -1
                )
        self.z_distance_increment = (Nav.find_distance_increment(
                                        z_distance,
                                        self.numb_waypoints
                                    ) * -1)

    def find_distance_increment(distance: float, numb_waypoints: int) -> float:
        """
            Calculate the distance incrememnt based upon the number of
            waypoints.

            ## Inputs:
                - distance [float] The distance between 2 waypoints
                - numb_waypoints [integer] The total number of waypoints
                                           to generate.
            
            ## Outputs:
                - A distance increment in Meters to separate the
                  the individual waypoints.
        """
        return float(abs(distance / numb_waypoints))

    def find_difference_in_distance(start_point: float, end_point: float) -> float:
        return float(
            abs(
                abs(end_point)
                - abs(start_point)
            )
        )

if __name__ == "__main__":
    new_nav = Nav('waypoints.json')
    new_nav.build_path()
    print("Route was generated with {} waypoints".format(len(new_nav.primary_route)))
    print("First Point [X, Y, Z, Vel]: {}".format(new_nav.primary_route[0]))
    print("Last Point [X, Y, Z, Vel]: {}".format(new_nav.primary_route[49]))
    
