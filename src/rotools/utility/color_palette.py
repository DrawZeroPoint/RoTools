import numpy as np
import matplotlib.pyplot as plt
from matplotlib import cm
from matplotlib.colors import ListedColormap, LinearSegmentedColormap

# https://matplotlib.org/stable/tutorials/colors/colormaps.html
bwr_color_palette = cm.get_cmap("bwr", 15)
# print(bwr_color_palette(0.0))  # Blue:  (0.0, 0.0, 1.0, 1.0)
# print(bwr_color_palette(0.5))  # White: (1.0, 1.0, 1.0, 1.0)
# print(bwr_color_palette(1.0))  # Red:   (1.0, 0.0, 0.0, 1.0)
