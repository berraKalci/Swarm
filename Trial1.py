import numpy as np
from airsimneurips import MultirotorClient
import time
import airsim


# Connect to the AirSim simulator client
client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)


# Obtain control over the drone and arm it
client.enableApiControl(vehicle_name="drone_1")
client.arm(vehicle_name="drone_1")
# List all objects in the simulation environment to identify gates and start block
scene_objects = client.simListSceneObjects()
print("Scene Objects: ", scene_objects)

# Define the relevant gate names and starting block
gates = ['Gate00', 'Gate01', 'Gate02', 'Gate03', 'Gate04', 'Gate05', 'Gate06', 
         'Gate07', 'Gate08', 'Gate09', 'Gate10_21', 'Gate11_23', 'StartBlock']

# Retrieve and print pose (position and orientation) for each gate and the starting block
gate_poses = {}
for obj in gates:
    pose = client.simGetObjectPose(obj)
    gate_poses[obj] = pose
    print(f"{obj} Pose: {pose}")

# Define a basic list of waypoints (x, y, z positions) and velocity for the drone to visit
# Modify these coordinates based on the actual gate positions
move_list = [
    [0, 2, 2, 5],  # Start position
    [1.6, 10.8, 2, 5],  # Waypoint 1 (Gate 01)
    [8.9, 18.5, 2, 5],  # Waypoint 2 (Gate 02)
    [18.7, 22.2, 2, 5],  # Waypoint 3 (Gate 03)
    [30, 22.2, 2, 5],  # Waypoint 4 (Gate 04)
    [39, 19.2, 2, 5],  # Waypoint 5 (Gate 05)
    [45.7, 11.7, 2, 5],  # Waypoint 6 (Gate 06)
    [48.3, 4.8, 2, 5],  # Waypoint 7 (Gate 07)
    [48.7, -7.9, 2, 5],  # Waypoint 8 (Gate 08)
    [45.7, -15.2, 2, 5],  # Waypoint 9 (Gate 09)
    [39, -19.2, 2, 5],  # Waypoint 10 (Gate 10)
    [30, -22.2, 2, 5],  # Waypoint 11 (Gate 11)
    [18.7, -22.2, 2, 5],  # Waypoint 12 (Gate 12)
    [8.9, -18.5, 2, 5],  # Waypoint 13 (Gate 13)
    [1.6, -10.8, 2, 5],  # Waypoint 14 (Gate 14)
    [0, -2, 2, 5]  # Final position
]

# Start the simulation race (optional based on your setup)
client.simStartRace()

# Move through each of the waypoints defined in the move_list
for waypoint in move_list:
    x = waypoint[0]
    y = waypoint[1]
    z = waypoint[2]
    velocity = waypoint[3]
    timeout = 5  # Set a timeout value for movement
    
    # Move to each position asynchronously with defined speed
    client.moveToPositionAsync(x, y, z, velocity, timeout).join()  # Use `.join()` to wait for each move completion
    print(f"Moved to position: ({x}, {y}, {z}) with velocity {velocity}")
    
# After reaching the final waypoint, disarm and release control
client.armDisarm(False)
client.enableApiControl(False)
