import numpy as np

def calculate_odometry(v_l, v_r, L):
    v = (v_r + v_l) / 2
    omega = (v_r - v_l) / L
    return v, omega

def wheel_velocities(v, omega, L):
    v_l = v - (omega * L) / 2
    v_r = v + (omega * L) / 2
    return v_l, v_r

def update_position(robot_position, v, omega, dt):
    x, y, theta = robot_position
    
    # Calculate new position
    x_new = x + v * np.cos(theta) * dt
    y_new = y + v * np.sin(theta) * dt
    theta_new = theta + omega * dt
    
    return [x_new, y_new, theta_new]
