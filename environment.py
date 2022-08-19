import json
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
ARMS_IDX_PAIRS = [[0, 1], [2, 3], [4, 5], [6, 7]]

TRASH_SUMMON_INTERVAL = 2500
FRAME_RATE = 1 / 240.


class Environment(object):
    def __init__(self, connection_mode, conveyor_speed, arms_path, trash_bins_path, set_pybullet_utils_p=False):
        """"
        @param connection_mode: pybullet simulation connection mode. e.g.: pybullet.GUI, pybullet.DIRECT
        @param set_pybullet_utils_p : if set to True,
        p variable of pybullet_utils (multiarm_planner.rrt) will be set to be self.p_simulation
        """
        self.p_simulation = bc.BulletClient(connection_mode=connection_mode)
        self.p_simulation.configureDebugVisualizer(self.p_simulation.COV_ENABLE_GUI, 0)

        if set_pybullet_utils_p:
            pybullet_utils.p = self.p_simulation

        self.p_simulation.setGravity(0, 0, -9.8)

        # Creating the environment
        self.plane = self.p_simulation.loadURDF(os.path.join(pybullet_data.getDataPath(), "plane.urdf"))
        self.arms_path = arms_path
        self.trash_bins_path = trash_bins_path
        self.bins = self._load_bins()
        self.arms = self._load_arms()
        self.arms_idx_pairs = ARMS_IDX_PAIRS
        self.arms_manager = multiarm_environment.MultiarmEnvironment(self.p_simulation, self.arms, gui=False, visualize=False)
        self.conveyor = Conveyor(self.p_simulation, CONVEYOR_LOCATION, speed=conveyor_speed, arms=self.arms)

        self.trash_generator = TrashGenerator(self.p_simulation, TRASH_SUMMON_INTERVAL, [1, 2, 0.5], CONVEYOR_LOCATION)

    def _load_bins(self):
        """
        Load the bins from the 'trash_bins_locations.json' file.
        """
        with open(self.trash_bins_path, 'r') as f:
            bins_data = json.load(f)

        bins = []
        bins_type = {
            'PLASTIC': TrashTypes.PLASTIC,
            'PAPER': TrashTypes.PAPER,
            'ELECTRONIC': TrashTypes.ELECTRONIC,
        }
        for b in bins_data:
            loc = b['loc']
            trash_type = bins_type[b['type']]
            bins.append(Bin(self.p_simulation, loc, trash_type))

        return bins

    def _load_arms(self):
        """
        Load the arms from the 'arms_locations.json' file.
        """
        with open(self.arms_path, 'r') as f:
            arms_data = json.load(f)

        ur5_list = []
        for arm in arms_data:
            orientation = p.getQuaternionFromEuler([0, 0, 0])

            if arm['loc'][0] > 0:
                ur5_arm = UR5.UR5(self.p_simulation, (arm['loc'], orientation))
                ur5_arm.set_arm_joints([math.pi - 0.6, -0.4, 0.6, 1.35, 1.55, 0.95])

            else:

                ur5_arm = UR5.UR5(self.p_simulation, (arm['loc'], orientation))
                ur5_arm.set_arm_joints([-0.6, -0.4, 0.6, 1.35, 1.55, 0.95])

            ur5_list.append(ur5_arm)

        return ur5_list
