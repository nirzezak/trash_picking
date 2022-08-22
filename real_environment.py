import math
import time

import pybullet as p

import environment
import ticker
from background_environment import BackgroundEnv
from environment import Environment, EnvironmentArgs
from score import Score
from summon_component import RandomSummonComponent, DeterministicSummonComponent, FixedAmountSummonComponent
from task_manager import AdvancedTaskManager, SimpleTaskManager, ParallelTaskManager
from configs.trash_configs import TrashConfig
from multiarm_planner.UR5 import ArmState


class RealEnv(Environment):
    def __init__(self, env_args: EnvironmentArgs):
        """"
        :param connection_mode: pybullet simulation connection mode. e.g.: pybullet.GUI, pybullet.DIRECT
        """
        super().__init__(env_args.connection_mode, 0.075, env_args.arms_path, env_args.trash_bins_path)

        # Manage the real environment: clocks, and scoreboard
        back_connection_mode = p.DIRECT if env_args.connection_mode == p.GUI else p.GUI
        back_env_args = EnvironmentArgs(back_connection_mode, env_args.arms_path, env_args.trash_bins_path)
        self.task_manager = ParallelTaskManager(self.arms, self.bins, self.conveyor.speed, back_env_args)
        self.summon_tick = math.floor(environment.TRASH_SUMMON_INTERVAL)
        self.score = Score()
        self.summon_component = FixedAmountSummonComponent(self.trash_generator, self.task_manager, self.summon_tick,
                                                           trash=TrashConfig.METAL_CAN, amount=2)
        time.sleep(3)

    def step(self):
        self.summon_component.step()

        # Call managing methods
        self.task_manager.step()

        # Simulate the environment
        for arm in self.arms:
            arm.ur5_step()
            if arm.state == ArmState.PICKING_TRASH:
                # Stop conveying trash that is being picked up
                self.conveyor.unconvey(arm.curr_task.trash.get_id())

        self.p_simulation.stepSimulation()
        self.conveyor.convey()
        self.remove_lost_cause_trash()
        if ticker.now() % 100 == 0:
            self.check_inserted_trash()
        ticker.tick()

    def remove_lost_cause_trash(self):
        contact_points = self.p_simulation.getContactPoints(bodyA=self.plane)
        body_uids = set([point[2] for point in contact_points])
        for body_uid in body_uids:
            if body_uid not in [self.conveyor, *self.bins]:
                self.trash_generator.remove_trash(body_uid)
                self.task_manager.remove_trash(body_uid)

    def check_inserted_trash(self):
        good_trash = set()
        bad_trash = set()
        for trash_bin in self.bins:
            contact_points = self.p_simulation.getContactPoints(bodyA=trash_bin.id)
            for point in contact_points:
                contact_point_on_trash = point[5]
                trash_id = point[2]
                if not trash_bin.is_inside_trash_bin(contact_point_on_trash):
                    continue
                elif self.trash_generator.get_trash(trash_id).trash_type != trash_bin.trash_type:
                    bad_trash.add(trash_id)
                else:
                    good_trash.add(trash_id)

        total_score_change = len(good_trash) * 1 - len(bad_trash) * 3
        self.score.increase_score(total_score_change)

        for trash in good_trash:
            self.trash_generator.remove_trash(trash)
            self.task_manager.remove_trash(trash)
        for trash in bad_trash:
            self.trash_generator.remove_trash(trash)
            self.task_manager.remove_trash(trash)
