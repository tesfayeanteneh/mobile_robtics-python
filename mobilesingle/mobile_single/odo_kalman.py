import numpy as np

def odo_kalman(x, y, theta, v, omega, dt, Q_k, R_k):
    """
    Perform odometry and Kalman filter update for a differential drive robot.
    
    Parameters:
    x (float): X position
    y (float): Y position
    theta (float): Orientation
    v (float): Linear velocity
    omega (float): Angular velocity
    dt (float): Time step
    Q_k (np.ndarray): Process noise covariance
    R_k (np.ndarray): Measurement noise covariance
    
    Returns:
    tuple: Updated x, y, and theta
    """
    # State prediction
    A_k = np.array([
        [1, 0, -v * dt * np.sin(theta)],
        [0, 1,  v * dt * np.cos(theta)],
        [0, 0, 1]
    ])
    B_k = np.array([
        [dt * np.cos(theta), 0],
        [dt * np.sin(theta), 0],
        [0, dt]
    ])
    u_k = np.array([v, omega])
    
    state_pred = A_k @ np.array([x, y, theta]) + B_k @ u_k
    P_pred = A_k @ np.eye(3) @ A_k.T + Q_k
    
    # Simulated measurement (e.g., from LiDAR)
    z_k = state_pred[:2] + np.sqrt(R_k[:2, :2]) @ np.random.randn(2)
    
    # Measurement update
    H_k = np.array([
        [1, 0, 0],
        [0, 1, 0]
    ])
    y_k = z_k - H_k @ state_pred
    S_k = H_k @ P_pred @ H_k.T + R_k[:2, :2]
    K_k = P_pred @ H_k.T @ np.linalg.inv(S_k)
    
    state_update = state_pred + K_k @ y_k
    x, y, theta = state_update[0], state_update[1], state_update[2]
    
    # Ensure theta is between -pi and pi
    theta = np.arctan2(np.sin(theta), np.cos(theta))
    
    return x, y, theta

if __name__ == "__main__":
    x, y, theta = odo_kalman(0, 0, 0, 1, 0.1, 0.1, np.diag([0.01, 0.01, 0.01]), np.diag([0.1, 0.1, 0.1]))
    print(f"x: {x}, y: {y}, theta: {theta}")
