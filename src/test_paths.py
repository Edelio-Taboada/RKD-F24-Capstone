import matplotlib.pyplot as plt
import numpy as np
import paths
from scipy.interpolate import CubicSpline

def discretize_vector_path(vector_path, resolution=100):
    """
    Discretizes a vector path into points and tracks pen state.
    
    Args:
        vector_path (list): The vector path from `glyph_data.get(letter)`.
        resolution (int): The number of points for each curve segment.
    
    Returns:
        list: A list of points and their corresponding pen state (True for down, False for up).
    """
    points = []
    pen_down = False  # Initial pen state is up
    current_pos = (0, 0)  # Starting position

    for command in vector_path:
        cmd, *args = command
        args = args[0]
        
        if cmd == "moveTo":
            current_pos = args[0]
            pen_down = False  # Pen is up on moveTo
            points.append((current_pos, pen_down))
        elif cmd == "lineTo":
            target_pos = args[0]
            # Interpolate between current_pos and target_pos
            x_vals = np.linspace(current_pos[0], target_pos[0], resolution)
            y_vals = np.linspace(current_pos[1], target_pos[1], resolution)
            for x, y in zip(x_vals, y_vals):
                points.append(((x, y), True))  # Pen is down for lineTo
            current_pos = target_pos
        elif cmd == "curveTo":
            if len(args) == 3:
                control1, control2, target_pos = args
                # print(args)
                t = np.linspace(0, 1, resolution) # <<<<<<<<<<<<<<<<<<<<<<<<< CHANGE THIS TO MAKE IT NON LINEAR
                x_vals = (
                    (1 - t) ** 3 * current_pos[0]
                    + 3 * (1 - t) ** 2 * t * control1[0]
                    + 3 * (1 - t) * t ** 2 * control2[0]
                    + t ** 3 * target_pos[0]
                )
                y_vals = (
                    (1 - t) ** 3 * current_pos[1]
                    + 3 * (1 - t) ** 2 * t * control1[1]
                    + 3 * (1 - t) * t ** 2 * control2[1]
                    + t ** 3 * target_pos[1]
                )
                control1, control2, target_pos = args
                # Create cubic splines for the x and y coordinates
                t = np.linspace(0, 1, resolution)
                
                # # Fit cubic splines for x and y
                # spline_x = CubicSpline([0, 0.5, 1], [current_pos[0], control1[0], control2[0], target_pos[0]])
                # spline_y = CubicSpline([0, 0.5, 1], [current_pos[1], control1[1], control2[1], target_pos[1]])
                
                # # Evaluate the splines
                # x_vals = spline_x(t)
                # y_vals = spline_y(t)

                for x, y in zip(x_vals, y_vals):
                    points.append(((x, y), True))  # Pen is down for curveTo
                current_pos = target_pos
            else:
                raise ValueError(f"Unexpected number of arguments for 'curveTo': {args}")
        elif cmd == "closePath":
            pass  # No additional points are needed for closePath
    print("\nLETTER PATH:   ",points)
    return points

def plot_glyph_points(points):
    """
    Plots a list of points representing a glyph with colors indicating pen state.
    
    Args:
        points (list): A list of (x, y) coordinates along with their pen state (True/False).
    """
    x_vals_down = []
    y_vals_down = []
    x_vals_up = []
    y_vals_up = []

    # Separate points based on pen state (down or up)
    for (x, y), pen_down in points:
        if pen_down:
            x_vals_down.append(x)
            y_vals_down.append(y)
        else:
            x_vals_up.append(x)
            y_vals_up.append(y)

    # Plot the points
    plt.figure(figsize=(6, 6))
    plt.plot(x_vals_down, y_vals_down, marker="o", markersize=2, linestyle="-", color='blue')  # Pen down (blue)
    plt.plot(x_vals_up, y_vals_up, marker="o", markersize=2, linestyle="-", color='red')    # Pen up (red)
    plt.axis("equal")
    plt.title("Discretized Glyph Path with Pen State")
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.show()

# Example usage
if __name__ == "__main__":
    # otf_path = "./fonts/Branchery.otf"
    otf_path = "./fonts/Milanello.otf"
    glyph_data = paths.extract_glyph_paths(otf_path)
    
    Alphabet = "ABCDEFG"
    for letter in Alphabet:
        vector_path = glyph_data.get(letter)
        if vector_path:
            discretized_points = discretize_vector_path(vector_path, resolution=100)
            plot_glyph_points(discretized_points)
        else:
            print(f"No vector path found for letter '{letter}'.")
