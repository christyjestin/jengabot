import numpy as np

NUM_LAYERS = 18 # actual number is 18, but can use a smaller number to speed up sim setup
NUM_BLOCKS_PER_LAYER = 3
BLOCK_WIDTH = 0.025
BLOCK_LENGTH = 0.075
BLOCK_HEIGHT = 0.015
BLOCK_POSITIONS = np.array([-1, 0, 1])
LEFT_POSITIONS = BLOCK_POSITIONS - 0.5
RIGHT_POSITIONS = BLOCK_POSITIONS + 0.5
assert BLOCK_POSITIONS.shape == (NUM_BLOCKS_PER_LAYER,)