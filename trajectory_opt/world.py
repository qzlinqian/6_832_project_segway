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


class StaticObstacle(object):

    def __init__(
            self,
            position,
            radius
    ):
        self.position = position
        self.radius = radius

    def distance(self, state, safe_radius):
        return 1

    def visualize(self, ax):
        return


class StaticRoundObstacle(StaticObstacle):

    def __init__(self, position, radius):
        super().__init__(position, radius)

    def distance(self, state, safe_radius):
        dis = (state[0] - self.position[0]) ** 2 + (state[1] - self.position[1]) ** 2
        return dis - (self.radius + safe_radius) ** 2

    def visualize(self, ax):
        circle = plt.Circle(self.position, radius=self.radius)
        ax.add_artist(circle)


class StaticSquareObstacle(StaticObstacle):
    def __init__(self, position, radius):
        super().__init__(position, radius)

    def distance(self, state, safe_radius):
        dis = np.abs(state[0] - self.position[0]) + np.abs(state[1] - self.position[1])
        return dis - (self.radius + safe_radius) * 2

    def visualize(self, ax):
        position = [self.position[0] - self.radius / 2,
                    self.position[1] - self.radius / 2]
        square = plt.Rectangle(position, self.radius, self.radius)
        ax.add_artist(square)


class World(object):

    def __init__(self, num_obs=5):
        self.segway = Segway(2, 4, 0.4, 0.1, 0.6, [-10.0, 10.0], [-3.0, 5.0])
        self.ref_line_points = []
        self.control_points = []
        self.obstacles = []
        self.g = 9.8

        for i in range(num_obs):
            x = 1 + i
            y = np.random.uniform(-0.5, 1.5)
            radius = np.random.randint(5, 10) * 0.1
            if radius < 0.01:
                continue
            self.obstacles.append(StaticRoundObstacle([x, y], radius))

    def visualize(self, figure, axes):
        if len(self.ref_line_points) > 0:
            positions = np.asarray(self.ref_line_points)
            plt.plot(positions[:, 0], positions[:, 1], c='c')
        if len(self.control_points) > 0:
            ctrl_pnts = np.asarray(self.control_points)
            plt.scatter(ctrl_pnts[:, 0], ctrl_pnts[:, 1], c='r', marker='o')
        axes.set_aspect(1)
        for obs in self.obstacles:
            obs.visualize(axes)
        plt.ylim([-2, 2])
        plt.xlim([-0.1, 10])

    def segway_dynamics(self, state, state_next, torque, time_interval):
        heading = state[2]
        vel = state[3]
        delta_state = [vel * np.cos(heading) * time_interval,
                       vel * np.sin(heading) * time_interval,
                       state[4] * time_interval,
                       (torque[0] + torque[1]) / 2 * self.segway.r * time_interval,
                       (torque[1] - torque[0]) * self.segway.r / 2 / self.segway.b * time_interval,
                       state[6] * time_interval,
                       ((torque[0] + torque[1]) / 2 * self.segway.r * np.cos(state[5])
                        + self.g * np.sin(state[5])) * time_interval]

        residuals = state_next - state - delta_state

        return residuals

    def save_to_file(self, file):
        world_info = {}
        if len(self.ref_line_points) > 0:
            world_info['ref_points'] = self.ref_line_points
        if len(self.control_points) > 0:
            world_info['control_points'] = self.control_points
        if len(self.obstacles) > 0:
            obs_list = []
            for obstacle in self.obstacles:
                obs_list.append([obstacle.position[0], obstacle.position[1], obstacle.radius])
            world_info['obstacle'] = obs_list
        file.write(str(world_info))


class TurningWorld(World):

    def __init__(self, obs_num=5):
        super().__init__(obs_num)

        # add control points for reference trajectory
        for i in range(5):
            self.control_points.append([i, 0, 0])
        self.control_points.append([4.5, 0.5, np.pi / 2])
        for i in range(5):
            self.control_points.append([(4 - i), 1, np.pi])

        for i in range(100):
            self.ref_line_points.append([i / 25, 0])
        for i in range(100):
            self.ref_line_points.append([4 + np.sin(i * np.pi * 0.01) * 0.5, 0.5 - np.cos(i * np.pi * 0.01) * 0.5])
        for i in range(101):
            self.ref_line_points.append([4 - i / 25, 1])


class ForwardWorld(World):

    def __init__(self, obs_num=0):
        super().__init__(obs_num)

        self.control_points.append([0., 0., 0.])
        self.control_points.append([6., -2., 0.])
        self.control_points.append([10., 2., 0.])

        # self.obstacles.append(StaticRoundObstacle([1, -2], 1))
        # self.obstacles.append(StaticRoundObstacle([2, 1], 1))
        self.obstacles.append(StaticRoundObstacle([5, 0.5], 1.5))
        self.obstacles.append(StaticRoundObstacle([9.5, -0.5], 1.5))


class CircleWorld(World):

    def __init__(self, obs_num=0):
        super().__init__(0)

        self.control_points.append([1., 4., 0.])
        self.control_points.append([1., 2., 0.])
        self.control_points.append([2., 1., 0.])
        self.control_points.append([3., 2., 0.])
        self.control_points.append([2., 3., 0.])
        self.control_points.append([0., 3., 0.])

        for i in range(10):
            self.ref_line_points.append([1., 4 - i/5])
        for i in range(75):
            self.ref_line_points.append([2 - np.cos(i * np.pi / 50), 2 - np.sin(i * np.pi / 50)])
        for i in range(10):
            self.ref_line_points.append([2 - i/5, 3])
        self.ref_line_points.append([0., 3.])

        self.obstacles.append(StaticRoundObstacle([2, 2], 0.45))
        for i in range(obs_num):
            alpha = np.random.uniform(0, np.pi * 2)
            x = 2 - np.cos(alpha) * 2
            y = 2 - np.sin(alpha) * 2
            radius = np.random.randint(5, 10) * 0.05
            self.obstacles.append(StaticRoundObstacle([x, y], radius))

    def visualize(self, figure, axes):
        super().visualize(figure, axes)
        plt.ylim([0, 4])
        plt.xlim([-0.1, 4.1])

    def deviation_with_ref(self, state):
        x = state[0]
        y = state[1]
        return (x - 2) ** 2 + (y - 2) ** 2 - 1


if __name__ == '__main__':
    world = CircleWorld(4)
    fig, ax = plt.subplots()
    world.visualize(fig, ax)
    plt.show()
