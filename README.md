# Multi Agent Routing
A development repository to control multiple drones through the use of a navigation library. This utilizes the [Airsim](https://github.com/microsoft/AirSim) simulation plugin for Unreal Engine and is currently only tested for that environment. More updates will be forthcoming.

## Background
This work came out of research conducted with Professor Shreyas Sundaram of Purdue University. Currently the `Nav` library produces a set of waypoints based upon the input and output coordinates. While not fully tested, this was used to simulate a Lead-Chase scenario for two drones. This project will be updated periodically with updates as we work on different routing protocols.

### The overall objectives of this project will be to:
1. Create a standard routing library to generate routes, similar to ArduPilot.
2. Create a multi-agent routing library to aid researchers in utilizing multiple UAVs.
3. Provide open-source software that is an outgrowth for the SWARM simulator (currently in development).

## How to Use

Currently, `main.py` contains the execution script for allowing two drones to execute a lead-follow manuever. To utilize this behavior:
1. Set the start and stop coordinates in the `waypoints.json`.
2. Set the number of waypoints that you want to generate in `main.py`. This will only generate a straight path for now.
```
route = Nav(file_name, numb_waypoints=50) # This is an example of what ya'll can do with it.
```

Run the script and use the breakpoint keys to control the simulation. Turns, orbits, etc. are in the works!

Please email xxEoD2242 at tyler.fedrizzi@gmail.com if you have any specific questions or open a GitHub issue.
