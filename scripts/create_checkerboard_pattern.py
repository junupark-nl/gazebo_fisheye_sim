import numpy as np
from PIL import Image

def create_checkerboard(size, squares):
    board = np.zeros((size, size, 3), dtype=np.uint8)
    square_size = size // squares
    for i in range(squares):
        for j in range(squares):
            if (i + j) % 2 == 0:
                board[i*square_size:(i+1)*square_size, j*square_size:(j+1)*square_size] = 255
    return Image.fromarray(board)

# Create an 1000x1000 pixel checkerboard with 9x9 squares
img = create_checkerboard(1000, 9)
img.save('../models/checkerboard/materials/textures/checkerboard.png')