import numpy as np

def odometry(v_l, v_r, x, y, theta, dt, b):
    v = (v_r + v_l) / 2
    omega = (v_r - v_l) / (2 * b)

    x_new = x + v * np.cos(theta) * dt
    y_new = y + v * np.sin(theta) * dt
    theta_new = theta + omega * dt

    return x_new, y_new, theta_new
