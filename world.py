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
            l,
            thrust_limit,
            velocity_limit
    ):

        self.m_wheel = m_wheel
        self.m_rod = m_rod
        self.b = b
        self.r = r
        self.l = l
        self.thrust_limit = thrust_limit
        self.velocity_limit = velocity_limit


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
        self.segway = Segway(2, 4, 0.4, 0.1, 0.6, 1.0, 5.0)
        self.ref_line_points = []
        self.obstacles = []

        for i in range(100):
            self.ref_line_points.append([i / 25, 0])
        for i in range(100):
            self.ref_line_points.append([4 + np.sin(i * np.pi * 0.01) * 0.5, 0.5 - np.cos(i * np.pi * 0.01) * 0.5])
        for i in range(101):
            self.ref_line_points.append([4 - i / 25, 1])
        for i in range(num_obs):
            x = 1 + i
            y = np.random.uniform(-1, 2)
            radius = np.random.randint(5, 10) * 0.03
            if radius < 0.01:
                continue
            self.obstacles.append(StaticRoundObstacle([x, y], radius))

    def visualize(self):
        figure, axes = plt.subplots()
        positions = np.asarray(self.ref_line_points)
        plt.plot(positions[:, 0], positions[:, 1])
        for obs in self.obstacles:
            circle = plt.Circle(obs.position, radius=obs.radius)
            axes.set_aspect(1)
            axes.add_artist(circle)
        plt.ylim([-2, 2])
        plt.xlim([-0.1, 10])
        plt.show()


if __name__ == '__main__':
    world = World(4)
    world.visualize()
