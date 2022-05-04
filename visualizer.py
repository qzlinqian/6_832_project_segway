# Created by qian at 5/3/22

# Description: for visualization

import matplotlib.pyplot as plt
import numpy as np


def draw_trajectory(states, ax):
    for state in states:
        x = state[0]
        y = state[1]
        psi = state[2]
        ax.arrow(x, y, 0.1 * np.cos(psi), 0.1 * np.sin(psi))


def draw_states(states, torque, t):
    plt.figure(figsize=(30, 24))

    ax1 = plt.subplot(3, 2, 1)
    ax1.plot(t, states[:, 3])
    ax1.set_title('velocity')
    ax1.set_ylabel('v(m/s)')
    ax1.set_xlabel('t(s)')

    ax2 = plt.subplot(3, 2, 2)
    ax2.plot(t, states[:, 4])
    ax2.set_title('angular velocity')
    ax2.set_ylabel('psi_dot(rad/s)')
    ax2.set_xlabel('t(s)')

    ax3 = plt.subplot(3, 2, 3)
    ax3.plot(t, states[:, 5])
    ax3.set_title('rod angle')
    ax3.set_ylabel('alpha(rad)')
    ax3.set_xlabel('t(s)')

    ax4 = plt.subplot(3, 2, 4)
    ax4.plot(t, states[:, 6])
    ax4.set_title('rod angular velocity')
    ax4.set_ylabel('alpha_dot(rad/s)')
    ax4.set_xlabel('t(s)')

    ax5 = plt.subplot(3, 1, 3)
    ax5.plot(t[:-1], torque[:, 0], c='r', label='left')
    ax5.plot(t[:-1], torque[:, 1], c='g', label='right')
    ax5.legend()
    ax5.set_title('torque')
    ax5.set_ylabel('angular acceleration(rad/s^2)')
    ax5.set_xlabel('t(s)')

    plt.show()


if __name__ == '__main__':
    state = np.zeros([101, 7])
    torque = np.zeros([100, 2])
    t = np.linspace(0, 101, 101)

    draw_states(state, torque, t)
