import numpy as np

def gradient_optimization(desired_path, initial_params, learning_rate, num_iterations):
    """
    Perform gradient optimization to minimize the mean square error (MSE) between
    the desired path and the actual path followed by the robot.
    
    Parameters:
    desired_path (np.ndarray): The desired path as an array of waypoints
    initial_params (np.ndarray): Initial control parameters
    learning_rate (float): Learning rate for gradient descent
    num_iterations (int): Number of iterations for optimization
    
    Returns:
    tuple: Optimized control parameters and the history of MSE values
    """
    params = initial_params
    mse_history = []

    for iter in range(num_iterations):
        # Simulate the actual trajectory with current parameters
        actual_path = simulate_trajectory(desired_path, params)
        
        # Compute the error
        error = desired_path - actual_path
        
        # Compute the Mean Square Error (MSE)
        mse = np.mean(error ** 2)
        mse_history.append(mse)
        
        # Compute the gradient of the MSE with respect to the control parameters
        grad = np.zeros_like(params)
        delta = 1e-5
        for i in range(len(params)):
            params_temp = params.copy()
            params_temp[i] += delta
            actual_path_temp = simulate_trajectory(desired_path, params_temp)
            mse_temp = np.mean((desired_path - actual_path_temp) ** 2)
            grad[i] = (mse_temp - mse) / delta
        
        # Update the control parameters using gradient descent
        params -= learning_rate * grad

        # Print the iteration and current MSE
        if iter % 100 == 0:
            print(f"Iteration {iter}: MSE = {mse:.4f}")

    return params, mse_history

def simulate_trajectory(desired_path, params):
    """
    Simulate the robot's trajectory based on the current control parameters.
    
    Parameters:
    desired_path (np.ndarray): The desired path as an array of way
        Parameters:
    desired_path (np.ndarray): The desired path as an array of waypoints
    params (np.ndarray): Current control parameters
    
    Returns:
    np.ndarray: Simulated actual path
    """
    num_waypoints = len(desired_path)
    actual_path = np.zeros_like(desired_path)

    # Initial state
    x, y = desired_path[0]
    theta = 0

    for i in range(num_waypoints):
        # Compute control input based on parameters
        v = params[0] + params[1] * np.sin(theta) + params[2]
        omega = params[0] + params[1] * np.cos(theta) + params[2]

        # Update state using simple kinematic model
        x += v * np.cos(theta)
        y += v * np.sin(theta)
        theta += omega

        # Store the actual path
        actual_path[i] = [x, y]

    return actual_path
