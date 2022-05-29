import math
import os
import time

import pybullet as p
import pybullet_data

from conveyor import Conveyor
from multiarm_planner.UR5 import UR5
from score import Score
from task_manager import TaskManager
from trash import MUSTARD_CONFIG
from trash_bin import Bin
from trash_generator import TrashGenerator
from trash_types import TrashTypes

URDF_FILES_PATH = "models"
CONVEYOR_LOCATION = [0, 0, 0.25]
BINS_LOCATIONS = [[1.5, 0.0, 0.1], [-1.5, 0.0, 0.1]]
UR5_LOCATIONS = [
    ([1, 0, 1], p.getQuaternionFromEuler([math.pi, 0, 0])),
    ([1, 1, 1], p.getQuaternionFromEuler([math.pi, 0, 0])),
    ([-1, 0, 1], p.getQuaternionFromEuler([math.pi, 0, 0])),
    ([-1, 1, 1], p.getQuaternionFromEuler([math.pi, 0, 0])),
]

FRAME_RATE = 1 / 240.
TRASH_SUMMON_INTERVAL = 1


class Environment(object):
    def __init__(self):
        p.connect(p.GUI)
        p.setGravity(0, 0, -9.8)

        # Creating the environment
        bins_path = os.path.join(URDF_FILES_PATH, "bin.urdf")
        self.plane = p.loadURDF(os.path.join(pybullet_data.getDataPath(), "plane.urdf"))
        self.bins = [Bin(bin_loc, TrashTypes.PLASTIC) for bin_loc in BINS_LOCATIONS]
        self.arms = [UR5(ur5_loc) for ur5_loc in UR5_LOCATIONS]
        self.conveyor = Conveyor(CONVEYOR_LOCATION, speed=0.25, arms=self.arms)

        # Manage the environment: trash generator, clocks, and scoreboard
        self.trash_generator = TrashGenerator(TRASH_SUMMON_INTERVAL, [1, 2, 0.5], CONVEYOR_LOCATION)
        self.task_manager = TaskManager(self.arms, self.bins, self.conveyor.speed)
        self.current_tick = 0
        self.summon_tick = math.floor(TRASH_SUMMON_INTERVAL / FRAME_RATE)
        self.score = Score()

    def step(self):
        # TODO: Could be converted to an event loop

        # Summon trash every couple of seconds
        if self.current_tick == self.summon_tick:
            trash = self.trash_generator.summon_trash(MUSTARD_CONFIG)
            self.task_manager.add_trash(trash)
            self.current_tick = 0

        # Call managing methods
        self.task_manager.try_dispatch_tasks()
        self.task_manager.notify_arms()
        self.task_manager.remove_completed_tasks()

        # Simulate the arms
        for arm in self.arms:
            # TODO: Uncomment
            # arm.ur5_step()
            pass

        # Simulate the environment
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
                self.task_manager.remove_uncaught_trash_task(body_uid)
