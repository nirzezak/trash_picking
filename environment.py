import math
import os
import time

import pybullet as p
from pybullet_utils import bullet_client as bc
import pybullet_data

from conveyor import Conveyor
from multiarm_planner.UR5 import UR5
from score import Score
from trash import TrashType
from trash_generator import TrashGenerator

from utils import add_element_wise

URDF_FILES_PATH = "models"
CONVEYOR_LOCATION = [0, 0, 0.25]
BINS_LOCATIONS = [[1.5, 0.0, 0.1], [-1.5, 0.0, 0.1]]
UR5_LOCATIONS = [
    ([1, 0, 1], p.getQuaternionFromEuler([math.pi, 0, 0])),
    ([-1, 0, 1], p.getQuaternionFromEuler([math.pi, 0, 0]))
]

FRAME_RATE = 1 / 240.
TRASH_SUMMON_INTERVAL = 1


class Environment(object):
    def __init__(self, connection_mode, world_origin_point=[0, 0, 0]):
        """"
        @param connection_mode: pybullet simulation connection mode. e.g.: pybullet.GUI, pybullet.DIRECT
        @param world_origin_point: the "zero" point, the center of this env coordinate system
        """
        self.p_simulation = bc.BulletClient(connection_mode=connection_mode)

        self.p_simulation.setGravity(0, 0, -9.8)

        # Creating the environment
        bins_path = os.path.join(URDF_FILES_PATH, "bin.urdf")
        self.plane = self.p_simulation.loadURDF(os.path.join(pybullet_data.getDataPath(), "plane.urdf"))
        self.bins = [self.p_simulation.loadURDF(bins_path, add_element_wise(bin_loc, world_origin_point),
                                                flags=p.URDF_USE_INERTIA_FROM_FILE, useFixedBase=True)
                     for bin_loc in BINS_LOCATIONS]
        self.arms = [UR5(self.p_simulation, (add_element_wise(ur5_loc[0], world_origin_point), ur5_loc[1])) for ur5_loc in UR5_LOCATIONS]
        conveyor_loc = add_element_wise(CONVEYOR_LOCATION, world_origin_point)
        self.conveyor = Conveyor(self.p_simulation, conveyor_loc, speed=0.25, arms=self.arms)

        # Manage the environment: trash generator, clocks, and scoreboard
        self.trash_generator = TrashGenerator(self.p_simulation, TRASH_SUMMON_INTERVAL, [1, 2, 0.5], conveyor_loc)
        self.current_tick = 0
        self.summon_tick = math.floor(TRASH_SUMMON_INTERVAL / FRAME_RATE)
        self.score = Score()

    def step(self):
        # TODO: Could be converted to an event loop

        # Summon trash every couple of seconds
        if self.current_tick == self.summon_tick:

            self.trash_generator.summon_trash(TrashType.MUSTARD)
            self.current_tick = 0
        self.p_simulation.stepSimulation()
        self.conveyor.convey()
        self.remove_uncaught_trash()
        time.sleep(FRAME_RATE)
        self.current_tick += 1

    def remove_uncaught_trash(self):
        contact_points = self.p_simulation.getContactPoints(bodyA=self.plane)
        body_uids = set([point[2] for point in contact_points])
        for body_uid in body_uids:
            if body_uid not in [self.conveyor, *self.bins]:
                self.trash_generator.remove_trash(body_uid)
