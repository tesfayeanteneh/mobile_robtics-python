import numpy as np
from .pid_controller import PIDController

# Initialize PID controller for angular velocity
pid_omega = PIDController(Kp=0.5, Ki=0.01, Kd=0.1, dt=0.05)

def control_law_follower2(follower1, follower2):
    dx = follower1['x'] - follower2['x']
    dy = follower1['y'] - follower2['y']

    distance = np.sqrt(dx**2 + dy**2)
    angle_to_follower1 = np.arctan2(dy, dx)
    angle_error = angle_to_follower1 - follower2['theta']

    v = distance  # Linear velocity directly proportional to the distance
    omega = pid_omega.update(angle_error)

    return v, omega
