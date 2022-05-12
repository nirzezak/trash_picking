import os
import time

import pybullet as p
import pybullet_data

from conveyor import Conveyor
from multiarm_planner.UR5 import UR5

URDF_FILES_PATH = "models"
CONVEYOR_LOCATION = [0, 0, 0.5]
BINS_LOCATIONS = [[1.5, 0.0, 0.1], [-1.5, 0.0, 0.1]]
UR5_LOCATIONS = [([1, 0, 1], [0, 0, 1, 0]), ([-1, 0, 1], [0, 0, 1, 0])]
FRAME_RATE = 1 / 240.


class Environment(object):
    def __init__(self):
        p.connect(p.GUI)
        p.setGravity(0, 0, -9.8)

        # Creating the environment
        self.plane = p.loadURDF(os.path.join(pybullet_data.getDataPath(), "plane.urdf"))

        bins_path = os.path.join(URDF_FILES_PATH, "bin.urdf")
        self.bins = [p.loadURDF(bins_path, bin_loc, flags=p.URDF_USE_INERTIA_FROM_FILE,
                                useFixedBase=True) for bin_loc in BINS_LOCATIONS]
        self.arms = [UR5(ur5_loc) for ur5_loc in UR5_LOCATIONS]
        self.conveyor = Conveyor(CONVEYOR_LOCATION)

    def step(self):
        # TODO: Could be converted to an event loop
        p.stepSimulation()
        self.conveyor.convey()
        time.sleep(FRAME_RATE)
