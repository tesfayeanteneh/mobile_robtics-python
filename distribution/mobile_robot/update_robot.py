from odometry import odometry
from is_collision import is_collision
from kinematics import kinematics

def update_robot(robot, v_l, v_r, dt, b, map):
    # Update robot position and orientation using odometry
    x_new, y_new, theta_new = odometry(v_l, v_r, robot['x'], robot['y'], robot['theta'], dt, b)
    
    # Check for collisions at the new position
    if not is_collision(x_new, y_new, map):
        # If no collision, update the robot's state
        robot['x'], robot['y'], robot['theta'] = kinematics((v_l + v_r) / 2, (v_r - v_l) / (2 * b), robot['x'], robot['y'], robot['theta'], dt)
    else:
        # Handle collision (e.g., stop the robot or implement avoidance)
        robot['x'], robot['y'], robot['theta'] = robot['x'], robot['y'], robot['theta']
