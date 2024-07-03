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

    # Initialize robot position (x, y, theta)
    robot_position = [0, 0, 0]
    robot_position1 = [0, 1,0]

    # Generate the path
    waypoints = generate_path()

    # Differential drive robot parameters
    L = 0.2  # 20 cm between the wheels
    dt = 0.1  # Reduced time step to slow down the movement
    constant_speed = 0.5  # Reduced constant linear speed
    learning_rate = 0.1  # Learning rate for gradient descent

    # Initialize AdaGrad sum of squares of gradients
    gradient_sum_sq = 0.0

    # Move the robot along the path and estimate velocities
    velocities = []
    wheel_speeds = []
    robot_positions = [robot_position[:2]]
    
    plt.figure(figsize=(10, 10))
    plt.ion()  # Enable interactive mode
    plt.grid(True)
    plt.xlim(0, map_size)
    plt.ylim(0, map_size)
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('Robot Path on 20x20 Grid Map')

    # Plot the desired path initially
    for i in range(len(waypoints) - 1):
        plt.plot([waypoints[i][0], waypoints[i + 1][0]], [waypoints[i][1], waypoints[i + 1][1]], 'x--', label='Desired Path' if i == 0 else "")

    plt.legend()
    plt.pause(0.1)

    for goal_position in waypoints[1:]:
        while np.linalg.norm([goal_position[0] - robot_position[0], goal_position[1] - robot_position[1]]) > 0.5:
            # Proportional control for the angular velocity
            k_omega = 2.0  # Angular velocity gain
            
            # Calculate the error in position
            dx = goal_position[0] - robot_position[0]
            dy = goal_position[1] - robot_position[1]
            distance = np.sqrt(dx**2 + dy**2)
            angle_to_goal = np.arctan2(dy, dx)
            heading_error = angle_to_goal - robot_position[2]

            # Normalize heading error to the range [-pi, pi]
            heading_error = np.arctan2(np.sin(heading_error), np.cos(heading_error))

            # Calculate the angular velocity
            omega = k_omega * heading_error

            # Calculate wheel velocities
            v_l, v_r = wheel_velocities(constant_speed, omega, L)
            velocities.append((constant_speed, omega))
            wheel_speeds.append((v_l, v_r))

            # Update the robot position
            robot_position = update_position(robot_position, constant_speed, omega, dt)
            robot_positions.append(robot_position[:2])

            # Calculate MSE
            mse = compute_mse(robot_positions, waypoints)
            if mse > 0.01:  # If MSE is not close to zero, adjust orientation
                robot_position, gradient_sum_sq, adjusted_learning_rate = adjust_orientation_adagrad(robot_position, heading_error, learning_rate, gradient_sum_sq)

            # Update plot with current position
            plt.plot(robot_position[0], robot_position[1], 'ro')  # Plot the robot's current position
            plt.pause(0.1)  # Pause to create the animation effect

    # Final plot of the desired path and actual path
    plt.ioff()  # Disable interactive mode
    plt.plot([pos[0] for pos in robot_positions], [pos[1] for pos in robot_positions], 'o-', label='Actual Path', linewidth=1)
    plt.plot([pos[0] for pos in waypoints], [pos[1] for pos in waypoints], 'x--', label='Desired Path')
    plt.plot(robot_positions[-1][0], robot_positions[-1][1], 'ro', markersize=8)  # Plot the robot's final position
    plt.grid(True)
    plt.xlim(0, map_size)
    plt.ylim(0, map_size)
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('Robot Path on 20x20 Grid Map')
    plt.legend()
    plt.show()

    # Print final learning rate
    print(f"Final learning rate: {adjusted_learning_rate}")

    # Display velocities and wheel speeds
    velocity_df = pd.DataFrame(velocities, columns=['Linear Velocity (v)', 'Angular Velocity (omega)'])
    wheel_speed_df = pd.DataFrame(wheel_speeds, columns=['Left Wheel Speed (v_l)', 'Right Wheel Speed (v_r)'])
    print("Robot Velocities:")
    print(velocity_df)
    print("Wheel Speeds:")
    print(wheel_speed_df)

# Execute the main function
if __name__ == "__main__":
    main()
