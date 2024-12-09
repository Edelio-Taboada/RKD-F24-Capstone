from fontTools.ttLib import TTFont
from fontTools.pens.recordingPen import RecordingPen
import numpy as np

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
                print(args)
                t = np.linspace(0, 1, resolution) # <<<<<<<<<<<<<<<<<<<<<<<<< CHANGE THIS TO MAKE IT NON LINEAR
                t = (np.cos(np.pi*t) + 1)*0.5
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
                # control1, control2, target_pos = args
                # # Create cubic splines for the x and y coordinates
                # t = np.linspace(0, 1, resolution)
                
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

    return points

def extract_glyph_paths(otf_file):
    """
    Extract vector paths for each glyph in the font file and store in a dictionary.
    
    Args:
        otf_file (str): Path to the .otf file.
    
    Returns:
        dict: A dictionary where keys are characters and values are vector paths.
    """
    # Load the font file
    # otf_file = "Branchery.otf"
    font = TTFont(otf_file)
    glyph_set = font.getGlyphSet()
    cmap = font.getBestCmap()  # Unicode to glyph name mapping

    glyph_paths = {}

    for char_code, glyph_name in cmap.items():
        pen = RecordingPen()  # Pen to record drawing commands
        glyph = glyph_set[glyph_name]

        # Extract vector path using the recording pen
        glyph.draw(pen)
        glyph_paths[chr(char_code)] = pen.value

    return glyph_paths

# Example usage
if __name__ == "__main__":
    otf_path = "Branchery.otf"  # Update with the path to your .otf file
    glyph_data = extract_glyph_paths(otf_path)

    # Example: Print vector paths for 'A'
    print("Vector paths for 'A':", glyph_data.get('A'))
