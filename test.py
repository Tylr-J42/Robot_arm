import numpy as np

def points_along_line(p1, p2, num_points=10):
    """
    Generate an array of points along the line connecting two 3D points.

    Parameters:
    p1 (list or tuple): The starting 3D point [x, y, z].
    p2 (list or tuple): The ending 3D point [x, y, z].
    num_points (int): The number of points to generate (default is 10).

    Returns:
    list: A list of lists, each containing [x, y, z] coordinates of the points.
    """
    if num_points < 1:
        raise ValueError("num_points must be at least 1")
    
    p1 = np.array(p1)
    p2 = np.array(p2)
    
    # Generate parameter t from 0 to 1
    t = np.linspace(0, 1, num_points)
    
    # Linear interpolation: points = p1 + t * (p2 - p1)
    points = p1 + t[:, np.newaxis] * (p2 - p1)
    
    return points.tolist()

print(points_along_line([0, 0, 0], [0.5, 0.5, 0.5], 50))