# Created by qian at 4/24/22

# Description: some basic class

import numpy as np


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

    def __init__(self, num_obs):
        self.segway = Segway(2, 4, 0.4, 0.1, 0.6, 1.0, 5.0)
        self.obstacles = []

        for i in range(num_obs):
            x = 1 + 2 * i
            y = np.random.uniform(-1, 1)
            radius = np.random.randint(0, 10) * 0.1
            if radius < 0.01:
                continue
            self.obstacles.append(StaticRoundObstacle([x, y], radius))

