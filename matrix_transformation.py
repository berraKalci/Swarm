import airsimneurips
import numpy as np
import math


# This establishes a connection between our Python script and the simulation environment.
client = airsimneurips.MultirotorClient()
client.confirmConnection()
print('Connection confirmed')  # Confirms that connection to the simulation is successful

def quaternion_to_yaw(q):
    """
        Convert a quaternion to yaw (rotation around Z-axis).
        The formula is:
        yaw = arctan2 (2(wz+xy),1−2(y^2 + z^2))

    """
    # AirSim uses (w, x, y, z) order for quaternions. Convert using the formula provided
    yaw = np.arctan2(2.0 * (q.w_val * q.z_val + q.x_val * q.y_val),
                     1.0 - 2.0 * (q.y_val * q.y_val + q.z_val * q.z_val))
    return yaw

def get_ned_to_world_transform(client, drone_name="drone_1"):
    """
        Retrieves the drone's position and orientation from AirSimNeurIPS functions.
        Computes the transformation matrix from NED to world coordinates.
    """
    # Get the multirotor state
    state = client.getMultirotorState(vehicle_name = drone_name)

    # Extract position in NED coordinates
    position = state.kinematics_estimated.position
    x, y, z = position.x_val, position.y_val, position.z_val

    # Extract orientation as a quaternion
    orientation = state.kinematics_estimated.orientation
    # print(orientation)

    kinematics = client.simGetGroundTruthKinematics(vehicle_name=drone_name)
    # print(kinematics)

    # Convert quaternion to yaw angle
    yaw = quaternion_to_yaw(orientation)
    print(yaw)

    # Subtract 90 degrees from the yaw
    print(f" Rotated by (yaw): {np.degrees(yaw)}")
    # yaw_rad = np.radians(yaw)
    # print(f" Rotated by before (yaw_rad): {np.degrees(yaw_rad)}")
    # yaw_rad = yaw_rad - (np.pi / 2)
    if (yaw > 0):
        yaw = yaw - (np.pi / 2)
    else:
        yaw = yaw + (np.pi / 2)

    # Limit the yaw values between 0 and 2 pi
    if (np.degrees(yaw) < 0):
        # Subtract the yaw from 360
        yaw = (2 * np.pi) - yaw
        # Subtract 360 if yaw is greater than 360
        if (np.degrees(yaw) > 360):
            yaw = yaw - (2 * np.pi)
        
    # Compute the rotation matrix components
    cos_val = np.cos(yaw)
    sin_val = np.sin(yaw)

    # print(f" Rotated by (yaw_rad): {np.degrees(yaw_rad)}")
    print(f" Rotated by (yaw): {np.degrees(yaw)}")

    # Convert radians to degrees
    # cos_val = np.degrees(cos_val)
    # sin_val = np.degrees(sin_val)

    # Define the rotation matrix (flipping the z-axis)
    R = np.array([
        [cos_val, -sin_val, 0],
        [sin_val, cos_val, 0],
        [0, 0, -1]
    ])


    # Build the full 4x4 transformation matrix
    T = np.eye(4)
    T[:3, :3] = R   # Insert the rotation matrix in the top-left
    T[:3, 3] = [x, y, z]    # Insert the vector in the rightmost column

    return T





# Call the functions for testing
transform_matrix = get_ned_to_world_transform(client, drone_name="drone_1")
print("NED to World Transformation Matrix before takeoff:")
print(transform_matrix)

client.enableApiControl(vehicle_name="drone_1")  # Enable API control for the drone
client.arm(vehicle_name="drone_1")  # Arm the drone so it can take off
client.simStartRace()  # Start the race
client.takeoffAsync(vehicle_name="drone_1").join()

transform_matrix = get_ned_to_world_transform(client, drone_name="drone_1")
print("NED to World Transformation Matrix:")
print(transform_matrix)

# Move the drone
client.moveByRollPitchYawZAsync(0, 0, np.pi*2, -10, duration=10, vehicle_name="drone_1").join()
# client.moveToPositionAsync(50, 20, -10, 5, vehicle_name="drone_1").join()
transform_matrix = get_ned_to_world_transform(client, drone_name="drone_1")
# transform_matrix_2 = get_ned_to_world_transform(client, drone_name="drone_2")
print("NED to World Transformation Matrix after move:")
print(transform_matrix)
# print(transform_matrix_2)