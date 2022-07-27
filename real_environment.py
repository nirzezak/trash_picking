import math

import pybullet as p

import environment
import ticker
from environment import Environment
from score import Score
from summon_component import RandomSummonComponent, DeterministicSummonComponent, FixedAmountSummonComponent
from task_manager import AdvancedTaskManager, SimpleTaskManager
from trash_configs import TrashConfig


class RealEnv(Environment):
    def __init__(self, connection_mode=p.GUI):
        """"
        :param connection_mode: pybullet simulation connection mode. e.g.: pybullet.GUI, pybullet.DIRECT
        """
        super().__init__(connection_mode, conveyor_speed=0.075)

        # Manage the real environment: clocks, and scoreboard
        self.task_manager = SimpleTaskManager(self.arms, self.bins, self.conveyor.speed)
        self.summon_tick = math.floor(environment.TRASH_SUMMON_INTERVAL)
        self.score = Score()
        self.summon_component = FixedAmountSummonComponent(self.trash_generator, self.task_manager, self.summon_tick, trash=TrashConfig.METAL_CAN)

    def step(self):
        self.summon_component.step()

        # Call managing methods
        self.task_manager.step()

        # Simulate the environment
        for arm in self.arms:
            arm.ur5_step()

        self.p_simulation.stepSimulation()
        self.conveyor.convey()
        self.remove_lost_cause_trash()
        ticker.tick()

    def remove_lost_cause_trash(self):
        contact_points = self.p_simulation.getContactPoints(bodyA=self.plane)
        body_uids = set([point[2] for point in contact_points])
        for body_uid in body_uids:
            if body_uid not in [self.conveyor, *self.bins]:
                self.trash_generator.remove_trash(body_uid)
                self.task_manager.remove_trash(body_uid)
