from fontTools.ttLib import TTFont
from fontTools.pens.recordingPen import RecordingPen

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
