import math
import time

import pybullet as p

import environment
import ticker
from environment import Environment, EnvironmentArgs
from score import Score
from summon_component import RandomSummonComponent, DeterministicSummonComponent, FixedAmountSummonComponent, \
    TrashListSummonComponent, AdvancedRandomSummonComponent
from task_manager import AdvancedTaskManager, SimpleTaskManager, ParallelTaskManager, AdvancedParallelTaskManager
from multiarm_planner.ur5 import ArmState


class RealEnv(Environment):
    def __init__(self, env_args: EnvironmentArgs, debug: bool):
        """"
        :param env_args: arguments on how to initialize the environment
        :param debug: print debug messages flag
        """
        super().__init__(env_args.connection_mode, 0.075, env_args.arms_path, env_args.trash_bins_path)

        # Manage the real environment: clocks, and scoreboard
        back_connection_mode = p.DIRECT if env_args.connection_mode == p.GUI else p.GUI
        back_env_args = EnvironmentArgs(back_connection_mode, env_args.arms_path, env_args.trash_bins_path)
        self.task_manager = AdvancedParallelTaskManager(self.arms, self.bins, self.conveyor.speed, back_env_args, debug)
        self.summon_tick = math.floor(environment.TRASH_SUMMON_INTERVAL)
        self.correct = Score('correct', color=[0.133, 0.545, 0.133], location=[0, 0, 2])
        self.wrong = Score('wrong', color=[1, 0, 0], location=[0, 0, 2.2])
        self.lost = Score('lost', color=[0, 0, 1], location=[0, 0, 2.4])
        self.summon_component = AdvancedRandomSummonComponent(self.trash_generator, self.task_manager, self.summon_tick)
        time.sleep(5)

    def step(self):
        """
        Main event loop of the simulation
        """
        self.summon_component.step()

        # Call managing methods
        self.task_manager.step()

        # Simulate the environment
        for arm in self.arms:
            arm.ur5_step()
            if arm.state == ArmState.PICKING_TRASH:
                # Stop conveying trash that is being picked up
                self.conveyor.unconvey(arm.curr_task.trash.get_id())

            if arm.state == ArmState.MOVING_TO_BIN:
                gripped_ids = arm.get_gripped_ids()
                trash_id = arm.curr_task.trash.get_id()

                if trash_id not in gripped_ids and trash_id in self.conveyor.dont_convey:
                    # Convey trash that was missed
                    self.conveyor.dont_convey.remove(trash_id)

        self.p_simulation.stepSimulation()
        self.conveyor.convey()
        self.remove_lost_cause_trash()
        if ticker.now() % 100 == 0:
            self.check_inserted_trash()
        ticker.tick()

    def remove_lost_cause_trash(self):
        """
        Remove trash that fell on the floor. Will also increase the lost trash score
        """
        contact_points = self.p_simulation.getContactPoints(bodyA=self.plane)
        body_uids = set([point[2] for point in contact_points])

        lost_trash_count = 0
        for body_uid in body_uids:
            if body_uid not in [self.conveyor, *self.bins]:
                self.trash_generator.remove_trash(body_uid)
                self.task_manager.remove_trash(body_uid)
                lost_trash_count += 1

                if body_uid in self.conveyor.dont_convey:
                    # Remove ID from dont_convey list because newly spawned trash
                    # can have the same ID as old removed trash
                    self.conveyor.dont_convey.remove(body_uid)

        self.lost.increase_score(lost_trash_count)

    def check_inserted_trash(self):
        """
        Check trash that was inserted to a trash bin. Will also increase the correct/wrong
        trash score

        WARNING: The computation here is expensive, and will SIGNIFICANTLY slow the simulation.
        Use only once every few steps instead of every step
        """
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

        self.correct.increase_score(len(good_trash))
        self.wrong.increase_score(len(bad_trash))

        for trash_id in good_trash:
            self.trash_generator.remove_trash(trash_id)
            self.task_manager.remove_trash(trash_id)
            if trash_id in self.conveyor.dont_convey:
                self.conveyor.dont_convey.remove(trash_id)
        for trash_id in bad_trash:
            self.trash_generator.remove_trash(trash_id)
            self.task_manager.remove_trash(trash_id)
            if trash_id in self.conveyor.dont_convey:
                self.conveyor.dont_convey.remove(trash_id)
