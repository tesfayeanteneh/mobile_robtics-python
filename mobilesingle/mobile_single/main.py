import numpy as np
import matplotlib.pyplot as plt
from create_map import create_map
from generate_path import generate_path
from odo_kalman import odo_kalman
from avoid_obstacles import avoid_obstacles
from gradient_optimization import gradient_optimization

# Parameters
map_size = 20
total_time = 20
dt = 0.1
R = 0.1  # Wheel radius (meters)
L = 0.25  # Half of the baseline (meters)
Q_k = np.diag([0.01, 0.01, 0.01])  # Process noise covariance
R_k = np.diag([0.1, 0.1, 0.1])  # Measurement noise covariance
vel_max = 1  # Maximum velocity
radii = 0.5  # Radius of the robot
lidar_range = 5  # LiDAR range
num_beams = 31  # Number of LiDAR beams
learning_rate = 0.01  # Learning rate for gradient descent
num_iterations = 1000  # Number of iterations for optimization

def compute_desired_velocity(pos, goal, vel_max, params):
    """
    Compute the desired velocity towards the goal.
    
    Parameters:
    pos (np.ndarray): Current position of the robot
    goal (np.ndarray): Goal position of the robot
    vel_max (float): Maximum velocity
    params (np.ndarray): Control parameters
    
    Returns:
    np.ndarray: Desired velocity
    """
    direction = goal - pos
    direction = direction.astype(float)  # Ensure direction is a float array
    direction_norm = np.linalg.norm(direction)
    if direction_norm > 0:
        direction /= direction_norm
    v_desired = (direction * vel_max) + params[0] * np.sin(pos[0]) + params[1] * np.cos(pos[1]) + params[2]
    
    # Clamp the velocity to ensure it doesn't exceed vel_max
    v_desired = np.clip(v_desired, -vel_max, vel_max)
    return v_desired

def main():
    # Initialize the map with obstacles
    map = create_map(map_size)
    
    # Generate the desired path
    path_x, path_y = generate_path()
    desired_path = np.column_stack((path_x, path_y))

    # Initial control parameters (e.g., PID gains)
    initial_params = np.random.randn(3)

    # Optimize control parameters using gradient descent
    optimized_params, mse_history = gradient_optimization(desired_path, initial_params, learning_rate, num_iterations)

    # Initialize the robot
    robot = {'pos': [0, 0], 'vel': [1, 0.5], 'goal': [18, 18], 'theta': 0}

    # Initialize storage for plotting
    num_steps = int(total_time / dt)
    pos_hist = np.zeros((2, num_steps))
    vel_hist = np.zeros(num_steps)
    omega_hist = np.zeros(num_steps)

    # Main simulation loop
    for t in range(num_steps):
        # Compute desired velocity using optimized control parameters
        v_desired = compute_desired_velocity(np.array(robot['pos']), np.array(robot['goal']), vel_max, optimized_params)
        
        # Use avoid_obstacles to get the new velocities
        new_v, new_omega = avoid_obstacles(
            robot['pos'], robot['theta'], (v_desired[0], robot['vel'][1]), 
            map, lidar_range, num_beams, radii, None, None
        )
        
        # Apply damping to reduce aggressiveness
        new_v = np.clip(new_v, -vel_max, vel_max)
        new_omega = np.clip(new_omega, -np.pi, np.pi)  # Angular velocity should be within -pi to pi

        # Update robot state using odo_kalman
        robot['pos'][0], robot['pos'][1], robot['theta'] = \
            odo_kalman(robot['pos'][0], robot['pos'][1], robot['theta'], new_v, new_omega, dt, Q_k, R_k)
        
        # Store the new velocities
        robot['vel'][0] = new_v
        robot['vel'][1] = new_omega

        # Store positions, velocities, and omegas for plotting
        pos_hist[:, t] = robot['pos']
        vel_hist[t] = robot['vel'][0]
        omega_hist[t] = robot['vel'][1]

        # Plot the robot and obstacles
        plt.figure(1)
        plt.clf()
        plt.imshow(map.T, cmap='gray', origin='lower')
        plt.plot(path_x, path_y, 'g-', linewidth=2)  # Desired path
        plt.plot(pos_hist[0, :t+1], pos_hist[1, :t+1], 'b-', linewidth=2)  # Robot trajectory

        # Plot robot's position
        circle = plt.Circle(robot['pos'], radii, color='b', fill=False)
        plt.gca().add_artist(circle)

        plt.xlabel('X Position')
        plt.ylabel('Y Position')
        plt.title('Map with Obstacles and Robot Trajectory')
        plt.legend(['Desired Path', 'Robot'], loc='best')
        plt.grid(True)
        plt.pause(0.1)  # Pause for visualization

    # Plot velocities and omegas
    plt.figure(2)
    plt.subplot(2, 1, 1)
    plt.plot(np.arange(0, total_time, dt), vel_hist, 'b-', linewidth=2)
    plt.xlabel('Time (s)')
    plt.ylabel('Linear Velocity (m/s)')
    plt.title('Linear Velocity of the Robot')
    plt.legend(['Robot'])
    plt.grid(True)

    plt.subplot(2, 1, 2)
    plt.plot(np.arange(0, total_time, dt), omega_hist, 'b-', linewidth=2)
    plt.xlabel('Time (s)')
    plt.ylabel('Angular Velocity (rad/s)')
    plt.title('Angular Velocity (Omega) of the Robot')
    plt.legend(['Robot'])
    plt.grid(True)

    plt.show()

if __name__ == "__main__":
    main()
