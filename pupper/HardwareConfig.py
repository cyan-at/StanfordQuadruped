"""
Per-robot configuration file that is particular to each individual robot, not just the type of robot.
"""
import numpy as np

MICROS_PER_RAD = 11.333 * 180.0 / np.pi  # Must be calibrated

# NEUTRAL_ANGLE_DEGREES = np.array([
#   [ -0,   7,   2,   3],
#   [ 17,  57, 46,  52],
#   [-39, -35, -33, -64]]
# )
NEUTRAL_ANGLE_DEGREES = np.array([
  [ 7,   -3,   5,   3],
  [ 59,  59, 50,  49],
  [-39, -22, -34, -32]]
)
# not sure about first leg rows [1], [2]

PS4_COLOR = {"red": 0, "blue": 0, "green": 255}
PS4_DEACTIVATED_COLOR = {"red": 0, "blue": 0, "green": 50}