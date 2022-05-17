import math
import os
import time

import pybullet as p
import pybullet_data

from conveyor import Conveyor
from multiarm_planner.UR5 import UR5
from score import Score
from trash import MUSTARD_CONFIG
from trash_generator import TrashGenerator

URDF_FILES_PATH = "models"
CONVEYOR_LOCATION = [0, 0, 0.25]
BINS_LOCATIONS = [[1.5, 0.0, 0.1], [-1.5, 0.0, 0.1]]
UR5_LOCATIONS = [([1, 0, 1], [0, 0, 1, 0]), ([-1, 0, 1], [0, 0, 1, 0])]
FRAME_RATE = 1 / 240.
TRASH_SUMMON_INTERVAL = 1


class Environment(object):
    def __init__(self):
        p.connect(p.GUI)
        p.setGravity(0, 0, -9.8)

        # Creating the environment
        bins_path = os.path.join(URDF_FILES_PATH, "bin.urdf")
        self.plane = p.loadURDF(os.path.join(pybullet_data.getDataPath(), "plane.urdf"))
        self.bins = [p.loadURDF(bins_path, bin_loc, flags=p.URDF_USE_INERTIA_FROM_FILE,
                                useFixedBase=True) for bin_loc in BINS_LOCATIONS]
        self.arms = [UR5(ur5_loc) for ur5_loc in UR5_LOCATIONS]
        self.conveyor = Conveyor(CONVEYOR_LOCATION, speed=0.25)

        # Manage the environment: trash generator, clocks, and scoreboard
        self.trash_generator = TrashGenerator(TRASH_SUMMON_INTERVAL, [1, 2, 0.5], CONVEYOR_LOCATION)
        self.current_tick = 0
        self.summon_tick = math.floor(TRASH_SUMMON_INTERVAL / FRAME_RATE)
        self.score = Score()

    def step(self):
        # TODO: Could be converted to an event loop

        # Summon trash every couple of seconds
        if self.current_tick == self.summon_tick:
            self.trash_generator.summon_trash(MUSTARD_CONFIG)
            self.current_tick = 0
        p.stepSimulation()
        self.conveyor.convey()
        self.remove_uncaught_trash()
        time.sleep(FRAME_RATE)
        self.current_tick += 1

    def remove_uncaught_trash(self):
        contact_points = p.getContactPoints(bodyA=self.plane)
        body_uids = set([point[2] for point in contact_points])
        for body_uid in body_uids:
            if body_uid not in [self.conveyor, *self.bins]:
                self.trash_generator.remove_trash(body_uid)
