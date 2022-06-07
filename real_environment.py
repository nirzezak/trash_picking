import math

import pybullet as p

import environment
from environment import Environment
from score import Score
from task_manager import TaskManager
from trash_configs import TrashConfig


class RealEnv(Environment):
    def __init__(self, connection_mode=p.GUI):
        """"
        :param connection_mode: pybullet simulation connection mode. e.g.: pybullet.GUI, pybullet.DIRECT
        """
        super().__init__(connection_mode, conveyor_speed=0.025)

        # Manage the real environment: clocks, and scoreboard
        self.task_manager = TaskManager(self.arms, self.bins, self.conveyor.speed)
        self.current_tick = 0
        self.summon_tick = math.floor(environment.TRASH_SUMMON_INTERVAL)
        self.score = Score()

    def step(self):
        # TODO: Could be converted to an event loop

        # Summon trash every couple of seconds
        if self.current_tick == self.summon_tick:
            trash = self.trash_generator.summon_trash(TrashConfig.MUSTARD)
            self.task_manager.add_trash(trash)
            self.current_tick = 0
        self.p_simulation.stepSimulation()

        # Call managing methods
        self.task_manager.try_dispatch_tasks()
        self.task_manager.notify_arms()
        self.task_manager.remove_completed_tasks()

        # Simulate the environment
        for arm in self.arms:
            arm.ur5_step()

        self.p_simulation.stepSimulation()
        self.conveyor.convey()
        self.remove_uncaught_trash()
        self.current_tick += 1

    def remove_uncaught_trash(self):
        contact_points = self.p_simulation.getContactPoints(bodyA=self.plane)
        body_uids = set([point[2] for point in contact_points])
        for body_uid in body_uids:
            if body_uid not in [self.conveyor, *self.bins]:
                self.trash_generator.remove_trash(body_uid)
                self.task_manager.remove_uncaught_trash_task(body_uid)
