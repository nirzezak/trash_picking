import pybullet as p

from environment import Environment


class BackgroundEnv(Environment):
    def __init__(self, connection_mode=p.DIRECT):
        """"
        :param connection_mode: pybullet simulation connection mode. e.g.: pybullet.GUI, pybullet.DIRECT
        """
        super().__init__(connection_mode, conveyor_speed=0)
