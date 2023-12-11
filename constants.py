import numpy as np

NUM_LAYERS = 4 # actual number is 18, but can use a smaller number to speed up sim setup
NUM_BLOCKS_PER_LAYER = 3

# block dimensions
BLOCK_WIDTH = 0.05
BLOCK_LENGTH = 0.15
BLOCK_HEIGHT = 0.03

# table dimensions
TABLE_HEIGHT = 0.03

# helper arrays for center of mass calculations
BLOCK_POSITIONS = np.array([-1, 0, 1])
LEFT_POSITIONS = BLOCK_POSITIONS - 0.5
RIGHT_POSITIONS = BLOCK_POSITIONS + 0.5
assert BLOCK_POSITIONS.shape == (NUM_BLOCKS_PER_LAYER,)

# tunable parameters
FRICTION_THRESHOLD = 0.5

# motion parameters
PUSH_FRACTION = 0.2