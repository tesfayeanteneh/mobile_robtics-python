import numpy as np

def create_map(map_size=20):
    """
    Initialize the map and place obstacles.
    
    Parameters:
    map_size (int): The size of the map (map_size x map_size)
    
    Returns:
    np.ndarray: The map with obstacles
    """
    # Initialize the map
    map = np.zeros((map_size, map_size))

    # Define the obstacles with start coordinates and length
    obstacles = np.array([
        [4, 10, 4],  # [length, start row, start column]
        [4, 10, 10], # [length, start row, start column]
        [4, 10, 15]  # [length, start row, start column]
    ])

    # Place the obstacles on the map
    for obstacle in obstacles:
        length, start_row, start_col = obstacle
        if start_row + length - 1 <= map_size:
            map[start_row:start_row+length, start_col] = 1
    
    return map
