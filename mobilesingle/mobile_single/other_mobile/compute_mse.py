import numpy as np

def compute_mse(actual_path, desired_path):
    min_length = min(len(actual_path), len(desired_path))
    actual_path = np.array(actual_path[:min_length])
    desired_path = np.array(desired_path[:min_length])
    mse = np.mean((actual_path - desired_path) ** 2)
    return mse

def adjust_orientation_adagrad(robot_position, heading_error, learning_rate, gradient_sum_sq):
    # Gradient descent adjustment with AdaGrad for orientation
    gradient = heading_error  # Simple gradient calculation for demonstration
    gradient_sum_sq += gradient ** 2
    adjusted_learning_rate = learning_rate / (np.sqrt(gradient_sum_sq) + 1e-8)
    robot_position[2] -= adjusted_learning_rate * gradient
    return robot_position, gradient_sum_sq, adjusted_learning_rate
