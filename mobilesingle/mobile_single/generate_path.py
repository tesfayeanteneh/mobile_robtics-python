def generate_path():
    """
    Generate a path with waypoints.
    
    Returns:
    tuple: Arrays of x and y coordinates of the path
    """
    waypoints = [
        (0, 0),
        (2, 18),
        (6, 18),
        (6, 2),
        (13, 2),
        (13, 18),
        (18, 18),
        (18, 2)
    ]
    path_x, path_y = zip(*waypoints)
    return path_x, path_y
