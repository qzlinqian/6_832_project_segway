# Created by qian at 5/3/22

# Description: for visualization

import matplotlib.pyplot as plt
import numpy as np


def draw_states(states, ax):
    for state in states:
        x = state[0]
        y = state[1]
        psi = state[2]
        ax.arrow(x, y, 0.1 * np.cos(psi), 0.1 * np.sin(psi))
