# Import required libraries
import numpy as np
from airsimneurips import MultirotorClient
import time
import airsim
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Set up and connect the client
client = MultirotorClient()
client.confirmConnection()
client.simLoadLevel('Soccer_Field_Easy')

# Obtain control over the drone and arm it
client.enableApiControl(vehicle_name="drone_1")
client.arm(vehicle_name="drone_1")

# Attempt to initiate the race, handling any issues that arise
try:
    client.simStartRace()
except Exception as e:
    print("Could not start the race or it's already active:", e)

# Command the drone to take off and reach a preset altitude
client.takeoffAsync(vehicle_name="drone_1").join()
client.moveToZAsync(-2, velocity=5, vehicle_name="drone_1").join()  # Ascend to 2 meters at a moderate speed

# Define a function to obtain updated gate coordinates
def get_gate_positions():
    gate_positions = []
    scene_objects = client.simListSceneObjects()
    gate_objects = sorted([obj for obj in scene_objects if "Gate" in obj])
    print("Detected gates:", gate_objects)

    for gate in gate_objects:
        try:
            pose = client.simGetObjectPose(gate).position  # nested loop for output
            if pose:
                gate_positions.append(pose)
        except Exception as e:
            print(f"Couldn't retrieve position for {gate}: {e}")
    return gate_positions

# Retrieve the gate locations
gate_positions = get_gate_positions()

# Optimized movement parameters
velocity = 5
threshold_distance = 3  # Optimized for a balance between speed and precision

# Initialize lists to store drone positions for plotting
x_positions = []
y_positions = []
z_positions = []

# Define function for accurate movement to each position
def navigate_to_position(target_pos):
    while True:
        # Retrieve the current location of the drone
        current_pose = client.simGetVehiclePose(vehicle_name="drone_1").position
        
        # Store the current position for plotting
        x_positions.append(current_pose.x_val)
        y_positions.append(current_pose.y_val)
        z_positions.append(current_pose.z_val)

        distance = np.sqrt(
            (target_pos.x_val - current_pose.x_val) ** 2 +
            (target_pos.y_val - current_pose.y_val) ** 2 +
            (target_pos.z_val - current_pose.z_val) ** 2
        )

        # Verify if the drone has reached the desired position
        if distance <= threshold_distance:
            print(f"Arrived at target: {target_pos}")
            break  # Move on to the next position

        # Move towards the target position with slight adjustments for smoother approach
        client.moveToPositionAsync(
            target_pos.x_val, target_pos.y_val, target_pos.z_val,
            velocity, vehicle_name="drone_1"
        )
        time.sleep(0.1)  # Brief pause to allow frequent position updates

# Guide the drone through each gate
for gate_pos in gate_positions:
    print(f"Navigating to gate located at: x={gate_pos.x_val}, y={gate_pos.y_val}, z={gate_pos.z_val}")
    navigate_to_position(gate_pos)

# Land the drone after completing all gates
client.landAsync(vehicle_name="drone_1").join()

# Disarm the drone and relinquish control
client.disarm(vehicle_name="drone_1")
client.disableApiControl(vehicle_name="drone_1")
print("Mission accomplished")

# Plot the drone's trajectory in 3D space
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot(x_positions, y_positions, z_positions, label='Drone Path', marker='o')
ax.scatter(
    [gate.x_val for gate in gate_positions],
    [gate.y_val for gate in gate_positions],
    [gate.z_val for gate in gate_positions],
    c='r', label='Gate Positions', marker='x'
)

# Label the axes
ax.set_xlabel('X Position')
ax.set_ylabel('Y Position')
ax.set_zlabel('Z Position')
ax.set_title('Drone Navigation Path')
ax.legend()
plt.show()


