import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
from map_creation import create_map
from odometry import calculate_odometry, wheel_velocities, update_position
from generate_path import generate_path
from compute_mse import compute_mse, adjust_orientation_adagrad

def main():
    # Create a 20x20 grid map
    map_size = 20
    grid_map = create_map(map_size)

    # Initialize robot positions (x, y, theta)
    robot_positions = [[0, 0, 0], [0, 1, 0]]

    # Generate the path
    waypoints = generate_path()

    # Differential drive robot parameters
    L = 0.2  # 20 cm between the wheels
    dt = 0.1  # Time step
    v1 = 0.5  # Linear speed for Robot 1
    v2 = 0.3  # Linear speed for Robot 2
    learning_rate = 0.1  # Learning rate for gradient descent

    # Initialize AdaGrad sum of squares of gradients
    gradient_sum_sqs = [0.0, 0.0]

    # Move the robots along the path and estimate velocities
    velocities = [[], []]
    wheel_speeds = [[], []]
    robot_paths = [[robot_positions[0][:2]], [robot_positions[1][:2]]]
    
    plt.figure(figsize=(10, 10))
    plt.ion()  # Enable interactive mode
    plt.grid(True)
    plt.xlim(0, map_size)
    plt.ylim(0, map_size)
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('Robot Paths on 20x20 Grid Map')

    # Plot the desired path initially
    for i in range(len(waypoints) - 1):
        plt.plot([waypoints[i][0], waypoints[i + 1][0]], [waypoints[i][1], waypoints[i + 1][1]], 'x--', label='Desired Path' if i == 0 else "")

    plt.legend()
    plt.pause(0.1)

    # Initialize goal indices for each robot
    goal_indices = [1, 1]

    # Main loop for both robots to move simultaneously
    all_robots_reached_goal = False
    while not all_robots_reached_goal:
        all_robots_reached_goal = True
        for i in range(2):  # For each robot
            if goal_indices[i] < len(waypoints):
                goal_position = waypoints[goal_indices[i]]
                if np.linalg.norm([goal_position[0] - robot_positions[i][0], goal_position[1] - robot_positions[i][1]]) > 0.5:
                    all_robots_reached_goal = False
                    # Proportional control for the angular velocity
                    k_omega = 2.0  # Angular velocity gain

                    # Calculate the error in position
                    dx = goal_position[0] - robot_positions[i][0]
                    dy = goal_position[1] - robot_positions[i][1]
                    distance = np.sqrt(dx**2 + dy**2)
                    angle_to_goal = np.arctan2(dy, dx)
                    heading_error = angle_to_goal - robot_positions[i][2]

                    # Normalize heading error to the range [-pi, pi]
                    heading_error = np.arctan2(np.sin(heading_error), np.cos(heading_error))

                    # Calculate the angular velocity
                    omega = k_omega * heading_error

                    # Calculate wheel velocities for each robot
                    if i == 0:
                        v = v1
                    else:
                        v = v2

                    v_l, v_r = wheel_velocities(v, omega, L)
                    velocities[i].append((v, omega))
                    wheel_speeds[i].append((v_l, v_r))

                    # Update the robot position
                    robot_positions[i] = update_position(robot_positions[i], v, omega, dt)
                    robot_paths[i].append(robot_positions[i][:2])

                    # Calculate MSE
                    mse = compute_mse(robot_paths[i], waypoints)
                    if mse > 0.01:  # If MSE is not close to zero, adjust orientation
                        robot_positions[i], gradient_sum_sqs[i], adjusted_learning_rate = adjust_orientation_adagrad(robot_positions[i], heading_error, learning_rate, gradient_sum_sqs[i])

                else:
                    # Move to the next goal position
                    goal_indices[i] += 1

        # Clear the plot and redraw the robots' positions
        plt.clf()
        plt.grid(True)
        plt.xlim(0, map_size)
        plt.ylim(0, map_size)
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.title('Robot Paths on 20x20 Grid Map')
        for i in range(len(waypoints) - 1):
            plt.plot([waypoints[i][0], waypoints[i + 1][0]], [waypoints[i][1], waypoints[i + 1][1]], 'x--', label='Desired Path' if i == 0 else "")
        plt.plot(robot_positions[0][0], robot_positions[0][1], 'ro', label='Robot 1')
        plt.plot(robot_positions[1][0], robot_positions[1][1], 'bo', label='Robot 2')
        plt.legend()
        plt.pause(0.1)

    # Final plot of the desired path and actual paths
    plt.ioff()  # Disable interactive mode
    plt.plot([pos[0] for pos in waypoints], [pos[1] for pos in waypoints], 'x--', label='Desired Path')
    plt.plot([pos[0] for pos in robot_paths[0]], [pos[1] for pos in robot_paths[0]], 'ro-', label='Actual Path 1', linewidth=1)
    plt.plot([pos[0] for pos in robot_paths[1]], [pos[1] for pos in robot_paths[1]], 'bo-', label='Actual Path 2', linewidth=1)
    plt.plot(robot_paths[0][-1][0], robot_paths[0][-1][1], 'ro', markersize=8)  # Plot the first robot's final position
    plt.plot(robot_paths[1][-1][0], robot_paths[1][-1][1], 'bo', markersize=8)  # Plot the second robot's final position
    plt.grid(True)
    plt.xlim(0, map_size)
    plt.ylim(0, map_size)
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('Robot Paths on 20x20 Grid Map')
    plt.legend()
    plt.show()

    # Print final learning rates
    print(f"Final learning rate for Robot 1: {adjusted_learning_rate}")
    print(f"Final learning rate for Robot 2: {adjusted_learning_rate}")

    # Display velocities and wheel speeds for both robots
    for i in range(2):
        velocity_df = pd.DataFrame(velocities[i], columns=['Linear Velocity (v)', 'Angular Velocity (omega)'])
        wheel_speed_df = pd.DataFrame(wheel_speeds[i], columns=['Left Wheel Speed (v_l)', 'Right Wheel Speed (v_r)'])
        print(f"Robot {i+1} Velocities:")
        print(velocity_df)
        print(f"Robot {i+1} Wheel Speeds:")
        print(wheel_speed_df)

# Execute the main function
if __name__ == "__main__":
    main()
