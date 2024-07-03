import numpy as np

def avoid_obstacles(pos, theta, vel, map, lidar_range, num_beams, radii, other_robot_pos=None, other_robot_vel=None):
    """
    Adjust velocity to avoid obstacles and other robots.
    
    Parameters:
    pos (list): Current position [x, y]
    theta (float): Current orientation
    vel (tuple): Desired velocity (linear, angular)
    map (np.ndarray): Map with obstacles
    lidar_range (float): LiDAR range
    num_beams (int): Number of LiDAR beams
    radii (float): Radius of the robot
    other_robot_pos (list): Position of the other robot (not used in single robot case)
    other_robot_vel (list): Velocity of the other robot (not used in single robot case)
    
    Returns:
    tuple: Adjusted linear and angular velocities
    """
    v_desired, omega_desired = vel

    # Use LiDAR to detect obstacles
    lidar_data = simulate_lidar(pos, theta, map, lidar_range, num_beams)

    # Check if there are obstacles in the path
    for distance in lidar_data:
        if distance < radii:
            # Calculate the avoidance velocity
            avoidance_angle = np.arctan2(pos[1], pos[0])
            v_avoid = -v_desired
            omega_avoid = np.sign(avoidance_angle - theta) * omega_desired
            return v_avoid, omega_avoid

    # If no collision, use desired velocities
    return v_desired, omega_desired

def simulate_lidar(pos, theta, map, lidar_range, num_beams):
    """
    Simulate LiDAR data for obstacle detection.
    
    Parameters:
    pos (list): Current position [x, y]
    theta (float): Current orientation
    map (np.ndarray): Map with obstacles
    lidar_range (float): LiDAR range
    num_beams (int): Number of LiDAR beams
    
    Returns:
    list: Distances to obstacles detected by LiDAR
    """
    distances = []
    angles = np.linspace(-np.pi/2, np.pi/2, num_beams)
    for angle in angles:
        x, y = pos
        for r in np.linspace(0, lidar_range, int(lidar_range / 0.1)):
            xi = int(x + r * np.cos(theta + angle))
            yi = int(y + r * np.sin(theta + angle))
            if xi < 0 or yi < 0 or xi >= map.shape[0] or yi >= map.shape[1] or map[xi, yi] == 1:
                distances.append(r)
                break
        else:
            distances.append(lidar_range)
    return distances
