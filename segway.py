# Created by qian at 4/24/22

# Description: class to store segway physical data

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
