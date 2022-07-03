import math
import os

import pybullet as p
from pybullet_utils import bullet_client as bc
import pybullet_data

from conveyor import Conveyor

from multiarm_planner import UR5, multiarm_environment
from multiarm_planner.rrt import pybullet_utils

from trash_bin import Bin
from trash_generator import TrashGenerator
from trash_types import TrashTypes

URDF_FILES_PATH = "models"
CONVEYOR_LOCATION = [0, 2, 0.25]
BINS_LOCATIONS = [[1.5, 0.0, 0.1], [-1.5, 0.0, 0.1]]
UR5_LOCATIONS = [
    ([1, 0, 1], p.getQuaternionFromEuler([math.pi, 0, 0])),
    ([1, 1, 1], p.getQuaternionFromEuler([math.pi, 0, 0])),
    ([-1, 0, 1], p.getQuaternionFromEuler([math.pi, 0, 0])),
    ([-1, 1, 1], p.getQuaternionFromEuler([math.pi, 0, 0])),
]
ARMS_IDX_PAIRS = [[0, 1], [2, 3]]

TRASH_SUMMON_INTERVAL = 1000
FRAME_RATE = 1 / 240.


class Environment(object):
    def __init__(self, connection_mode, conveyor_speed, set_pybullet_utils_p=False):
        """"
        @param connection_mode: pybullet simulation connection mode. e.g.: pybullet.GUI, pybullet.DIRECT
        @param set_pybullet_utils_p : if set to True,
        p variable of pybullet_utils (multiarm_planner.rrt) will be set to be self.p_simulation
        """
        self.p_simulation = bc.BulletClient(connection_mode=connection_mode)

        if set_pybullet_utils_p:
            pybullet_utils.p = self.p_simulation

        self.p_simulation.setGravity(0, 0, -9.8)

        # Creating the environment
        self.plane = self.p_simulation.loadURDF(os.path.join(pybullet_data.getDataPath(), "plane.urdf"))
        self.bins = [Bin(self.p_simulation, bin_loc, TrashTypes.PLASTIC) for bin_loc in BINS_LOCATIONS]
        self.arms = [UR5.UR5(self.p_simulation, ur5_loc) for ur5_loc in UR5_LOCATIONS]
        self.arms_idx_pairs = ARMS_IDX_PAIRS
        self.arms_manager = multiarm_environment.MultiarmEnvironment(self.p_simulation, self.arms, gui=False, visualize=False)
        self.conveyor = Conveyor(self.p_simulation, CONVEYOR_LOCATION, speed=conveyor_speed, arms=self.arms)

        self.trash_generator = TrashGenerator(self.p_simulation, TRASH_SUMMON_INTERVAL, [1, 2, 0.5], CONVEYOR_LOCATION)
