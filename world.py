# Created by qian at 4/24/22

# Description: some basic class

import numpy as np
import matplotlib.pyplot as plt


# class to store segway physical data
class Segway(object):

    def __init__(
            self,
            m_wheel,
            m_rod,
            b,
            r,
            length,
            torque_limit,
            velocity_limit,
    ):
        self.m_wheel = m_wheel
        self.m_rod = m_rod
        self.b = b
        self.r = r
        self.len = length
        self.torque_limit = torque_limit
        self.velocity_limit = velocity_limit
        self.safe_radius = b * 1.2


class StaticRoundObstacle(object):

    def __init__(
            self,
            position,
            radius
    ):
        self.position = position
        self.radius = radius


class World(object):

    def __init__(self, num_obs=5):
        self.segway = Segway(2, 4, 0.4, 0.1, 0.6, [-1.0, 1.0], [-2.0, 5.0])
        self.ref_line_points = []
        self.control_points = []
        self.obstacles = []

        # add control points for reference trajectory
        for i in range(5):
            self.control_points.append([i, 0, 0])
        self.control_points.append([4.5, 0.5, np.pi/2])
        for i in range(5):
            self.control_points.append([(4 - i), 1, np.pi])

        for i in range(100):
            self.ref_line_points.append([i / 25, 0])
        for i in range(100):
            self.ref_line_points.append([4 + np.sin(i * np.pi * 0.01) * 0.5, 0.5 - np.cos(i * np.pi * 0.01) * 0.5])
        for i in range(101):
            self.ref_line_points.append([4 - i / 25, 1])

        for i in range(num_obs):
            x = 1 + i
            y = np.random.uniform(-1, 2)
            radius = np.random.randint(5, 10) * 0.05
            if radius < 0.01:
                continue
            self.obstacles.append(StaticRoundObstacle([x, y], radius))

    def visualize(self):
        figure, axes = plt.subplots()
        positions = np.asarray(self.ref_line_points)
        plt.plot(positions[:, 0], positions[:, 1], c='c')
        ctrl_pnts = np.asarray(self.control_points)
        plt.plot(ctrl_pnts[:, 0], ctrl_pnts[:, 1], c='r', marker='o')
        for obs in self.obstacles:
            circle = plt.Circle(obs.position, radius=obs.radius)
            axes.set_aspect(1)
            axes.add_artist(circle)
        plt.ylim([-2, 2])
        plt.xlim([-0.1, 10])
        plt.show()
        return figure, axes

    def segway_dynamics(self, state, state_next, torque, time_interval):
        heading = state[2]
        vel = state[3]
        state_dot = [vel * np.cos(heading) * time_interval,
                     vel * np.sin(heading) * time_interval,
                     state[4] * time_interval,
                     (torque[0] + torque[1]) / 2 * self.segway.r * time_interval,
                     (torque[1] - torque[0]) * self.segway.r / 2 / self.segway.b * time_interval]

        residuals = state_next - state - state_dot

        return residuals


if __name__ == '__main__':
    world = World(4)
    world.visualize()
