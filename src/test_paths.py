import matplotlib.pyplot as plt
import numpy as np
import paths
from scipy.interpolate import CubicSpline


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
    otf_path = "./fonts/Branchery.otf"
    # otf_path = "./fonts/Milanello.otf"
    glyph_data = paths.extract_glyph_paths(otf_path)
    
    Alphabet = "ABCDEFG"
    for letter in Alphabet:
        vector_path = glyph_data.get(letter)
        if vector_path:
            discretized_points = paths.discretize_vector_path(vector_path, resolution=100)
            plot_glyph_points(discretized_points)
        else:
            print(f"No vector path found for letter '{letter}'.")
