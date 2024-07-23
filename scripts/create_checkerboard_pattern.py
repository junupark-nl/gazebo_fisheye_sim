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

# Create an 1200x1200 pixel checkerboard with 10x10 squares
img = create_checkerboard(1200, 10)
img.save('/root/catkin_ws/src/gazebo_fisheye_sim/models/checkerboard/materials/textures/checkerboard.png')