import math
import uuid
from abc import ABC, abstractmethod
from dataclasses import dataclass
import logging

import numpy as np

from typing import List, Dict, Tuple, Optional, Union

import ticker
from environment import EnvironmentArgs
from configs import trash_configs
from multiarm_planner.multiarm_environment import split_arms_conf_lst
from background_environment import BackgroundEnv, ParallelEnv
from multiarm_planner.ur5 import Robotiq2F85, UR5, ArmState
from task import Task, TaskState
from trash import Trash
from trash_bin import Bin
from trash_types import TrashTypes

# max distance (in x,y axes) between the arm base and a picking point
# in which the arm can get to the picking point
ARM_TO_TRASH_MAX_DIST = [0.73, 0.4]
# Lower bound for number of tick it takes for the arm to finish a task from the moment it picked the trash
ARMS_SAFETY_OFFSET = [0, 0.15, 0]

# lower bound estimation for number of ticks it takes for an arm to move to a
# trash on the conveyor. Used to determine whether an arm can possibly do some task
TICKS_TO_TRASH_LOW_BOUND = 700
# lower bound estimation for number of ticks it takes for an arm to do a full task.
# Used to determine whether an arm can possibly do some task
TICKS_FULL_TASK_LOW_BOUND = 2000
# offset to be added to the bin location, when both arms go to the same bin
TICKS_AFTER_GETTING_TO_TRASH_LOW_BOUND = TICKS_FULL_TASK_LOW_BOUND - TICKS_TO_TRASH_LOW_BOUND


class TaskManagerComponent(ABC):
    """
    Interface to manage tasks
    """
    @abstractmethod
    def add_trash(self, trash: Trash):
        """
        Function to add trash to manage
        """
        pass

    @abstractmethod
    def step(self):
        """
        Function to call in each simulation step
        """
        pass

    @staticmethod
    def _find_closest_bins(arm: UR5, bins: List[Bin]) -> Dict:
        """
        Finds the closest bins to the arm, for each trash type.

        @param arm: The arm that we will use.
        @param bins: The bins in the environment.

        @returns The location of the closest bins, with some offset to avoid
        collisions when both arms need to work on the same trash bin.
        """
        arm_loc = np.array(arm.pose[0])
        closest_bins = {}

        for trash_type in TrashTypes:
            closest_bin_distance = math.inf
            closest_bin = None
            for trash_bin in bins:
                if trash_bin.trash_type == trash_type:
                    # Calculate distance
                    trash_bin_loc = np.array(trash_bin.location)
                    distance = np.linalg.norm(arm_loc - trash_bin_loc)
                    if distance < closest_bin_distance:
                        closest_bin_distance = distance
                        closest_bin = trash_bin

            closest_bins[trash_type] = closest_bin

        return closest_bins

    def _get_ticks_for_full_task_heuristic(self, path_to_trash_len: int, path_to_bin_len: int) -> int:
        """"
        Get an estimation for number of ticks for a full task (moving to trash, picking, moving to bin, dropping)

        :param path_to_trash_len: length of path to trash
        :param path_to_bin_len: length of path to bin

        :returns: an estimation of the number of ticks it would take to do the full task
        """
        total_ticks = self._get_ticks_for_path_to_trash_heuristic(path_to_trash_len)
        total_ticks += 2 * Robotiq2F85.TICKS_TO_CHANGE_GRIP
        total_ticks += self._get_ticks_for_path_to_bin_heuristic(path_to_bin_len)
        return total_ticks

    @staticmethod
    def _get_ticks_for_path_to_trash_heuristic(path_len: int) -> int:
        """
        Get an estimation for number of ticks for path to trash

        :param path_len: length of path to trash

        :returns: an estimation of the number of ticks it would take to do the path

        Explanation:
        The path is built from ur5 configuration list,
        each configuration change takes some number of ticks (not fixed).
        For each path we can calculate the series of #tick it took for each configuration change.
        We noticed that for MOVING_TO_TRASH path, this "#ticks per configuration" series has a fixed pattern
        (even when using different trash locations)
        The pattern we found: 2,x,...,x,1,x,... where x is 47.4 in expectation (#ticks per conf series)

        """
        if path_len < 10:
            print('WARNING: unexpected path to trash length, enhance the get_ticks_for_path_to_trash_heuristic')
            return path_len * 40  # arbitrary, we didn't see such examples
        return math.ceil(1 + 2 + 47.4 * (path_len - 2))

    @staticmethod
    def _get_ticks_for_path_to_bin_heuristic(path_len: int) -> int:
        """
        Get an estimation for number of ticks for path to bin

        :param path_len: length of path to bin

        :returns: an estimation of the number of ticks it would take to do the path

        Explanation:
        The path is built from ur5 configuration list,
        each configuration change takes some number of ticks (not fixed).
        For each path we can calculate the series of #tick it took for each configuration change.
        We noticed that for MOVING_TO_BIN path, this "#ticks per configuration" series has a fixed pattern
        (even when using different trash locations)
        The pattern we found: 1,47,48,48,... (#ticks per conf series)
        """

        if path_len < 3:
            print('WARNING: unexpected path to trash length, enhance the get_ticks_for_path_to_bin_heuristic')
            return path_len * 40  # arbitrary, we didn't see such examples
        return 1 + 47 + 48 * (path_len - 2)

    @staticmethod
    def _calc_ticks_to_destination_on_conveyor(trash: Trash, trash_dest: List[int], trash_velocity: float) -> int:
        """
        Calculate the number of ticks it takes to the trash object to get to the
        destination.
        Note that since the conveyor only moves objects in the Y axis, the
        distance is calculated based on that.

        :param trash: the trash object to calculate for
        :param trash_dest: destination of the trash object
        :param trash_velocity: speed of the trash object
        """
        # Note: We got this constant from measuring different speeds, not
        # from something official from pybullet
        distance_per_tick = 0.0042 * trash_velocity
        curr_location = trash.get_curr_position()
        diff = abs(curr_location[1] - trash_dest[1])
        return math.ceil(diff / distance_per_tick)


class AdvancedTaskManager(TaskManagerComponent):
    """
    WARNING: THIS MODULE IS DEPRECATED. Use with caution!
    """
    def __init__(self, arms: List[UR5], arms_idx_pairs: List[List[int]], bins: List[Bin], trash_velocity: float,
                 background_env: BackgroundEnv):
        """
        @param arms: A list of all of the available arms
        @param arms_idx_pairs: a list of arm pairs, each pair is represented as a list (length=2),
        containing the indices of the arms of that pair.
        arm pair should be from the same side of the conveyor and without other arms between.
        we assume that the distance between each pair is the same
        @param bins: A list of all of the available bins
        @param trash_velocity: The velocity of the trash, we assume velocity
        only on the Y axis.
        axis 0 - x, axis 1 - y, axis 2 - z

        Invariants:
        - Each list of tasks from self.arms_to_tasks is sorted by task.start_tick
        """
        self.arms = arms
        self.arms_idx_pairs = arms_idx_pairs
        self.sort_arms_pairs_by_y_axis()
        self.bins = bins
        self.trash_velocity = trash_velocity

        self.background_env = background_env

        self.single_trash = []  # unassigned trash, that couldn't be paired yet
        self.arms_to_tasks: Dict[UR5, List[Task]] = {arm: [] for arm in self.arms}  # maps arms to a list of their current tasks, ordered by task.start_tick

        # calculate distance between arm pair in y axis, assuming this distance is the same for each pair
        arm_pair0_y_axis = [self.arms[idx].get_pose()[0][1] for idx in arms_idx_pairs[0]]  # list of the y axis value of the arms in pair 0
        arm_pair_dist_y_axis = abs(arm_pair0_y_axis[0] - arm_pair0_y_axis[1])

        self.max_dist_between_trash_pair_y_axis = 2 * ARM_TO_TRASH_MAX_DIST[1] + arm_pair_dist_y_axis

    def add_trash(self, trash: Trash):
        """
        @param trash: The trash to add
        Try to find a trash pair for @param trash,
        if a pair is found, give this 2 trash task to a pair of arms (if possible)
        otherwise, add @param trash to self.single_trash list
        """
        for older_trash in self.single_trash:
            if self.can_be_trash_pair(trash, older_trash):
                # try to find pair of arms for this trash pair
                if self.add_trash_task_to_arms_group([trash, older_trash], ticker.now()):
                    # this pair trash is assigned to some pair of arms
                    self.single_trash.remove(older_trash)
                    return
        # failed to assign this trash, add trash to self.single_trash
        self.single_trash.append(trash)

    def step(self):
        self.handle_single_trash_that_passed_pnr()
        self.notify_arms_and_remove_completed_tasks()

    def get_arm_pair(self, arm_idx: int) -> List[int]:
        for pair in self.arms_idx_pairs:
            if arm_idx in pair:
                return pair

    def get_index_for_new_task(self, arm: UR5, task_start_tick: int) -> int:
        """
        Returns the appropriate index in self.arms_to_tasks[arm] to add a new task with start_tick=task_start_tick,
        while preserving the invariant about the list order.
        """
        tasks_lst = self.arms_to_tasks[arm]
        for i in range(len(tasks_lst)):
            if tasks_lst[i].start_tick > task_start_tick:
                return i
        return len(tasks_lst)

    def get_task_prev_and_next(self, arm: UR5, task_start_tick: int) -> Tuple[Optional[Task], Optional[Task]]:
        """
        Returns 2 tasks from self.arms_to_tasks[arm]:
        - Prev task - the task with the highest start_tick that is still smaller (or =) than task_start_tick,
        if there is no such task, returns None
        - Next task - the task with the lowest start_tick that is still bigger than task_start_tick,
        if there is no such task, returns None
        """
        tasks_lst = self.arms_to_tasks[arm]
        prev_task = None
        for i in range(len(tasks_lst)):
            if tasks_lst[i].start_tick > task_start_tick:
                return prev_task, tasks_lst[i]
            prev_task = tasks_lst[i]
        return prev_task, None

    def sort_arms_pairs_by_y_axis(self):
        """
        Sort each pair in self.arms_idx_pairs by the y-axis order of the arms (the arm with the lower y value first)
        """
        for arm_pair_idx in self.arms_idx_pairs:
            arm_pair_idx.sort(key=lambda idx: self.arms[idx].get_pose()[0][1])

    def _find_closest_bin(self, trash: Trash, arm: UR5) -> List[int]:
        """
        Finds the closest bin to the arm, that handles this type of trash.

        @param trash: The trash object we want to recycle.
        @param arm: The arm that we will use.

        @returns The location of the closest bin,
        for shared bins, add some offset to avoid collisions when both arms need to work on that trash bin.
        """
        arm_loc = arm.pose[0]
        arm_loc = np.array(arm_loc)

        # Find the closest bin
        closest_bin_distance = math.inf
        closest_bin = None
        for trash_bin in self.bins:
            if trash_bin.trash_type == trash.trash_type:
                # Calculate distance
                trash_bin_loc = np.array(trash_bin.location)
                distance = np.linalg.norm(arm_loc - trash_bin_loc)
                if distance < closest_bin_distance:
                    closest_bin_distance = distance
                    closest_bin = trash_bin

        return closest_bin.location.copy()

    def find_bin_loc_for_arms(self, trash_lst: List[Trash], arms: List[UR5]) -> List[List[int]]:
        """
        @param trash_lst: list of trash objects we want to recycle.
        @param arms: list of arms that will be used (accordingly)
        assumes len(trash_lst) = len(arms) < 3

        @returns The locations of the closest bin for each arm-trash,
        if both arms need to use the same bin, add some offset to avoid collisions.
        """
        bin_loc_list = [self._find_closest_bin(trash, arm) for trash, arm in zip(trash_lst, arms)]

        if len(bin_loc_list) == 2 and bin_loc_list[0] == bin_loc_list[1]:
            # add some offset to avoid collisions
            for i in range(2):
                arm_loc = arms[i].pose[0]
                arm_loc = np.array(arm_loc)
                bin_loc = bin_loc_list[i]
                if arm_loc[1] > bin_loc[1]:
                    # The arm is ahead than the bin, therefore the arm needs to go a
                    # little bit further ahead in the y axis
                    for dim in range(3):
                        bin_loc[dim] += ARMS_SAFETY_OFFSET[dim]
                elif arm_loc[1] < bin_loc[1]:
                    # The bin is ahead than the arm, therefore the arm needs to go a
                    # little bit further back in the y axis
                    for dim in range(3):
                        bin_loc[dim] -= ARMS_SAFETY_OFFSET[dim]
                else:
                    # Shouldn't get here
                    # The arm and the bin are at the same Y offset, so no need for
                    # any safety offset (in this case, only 1 arm uses this bin)
                    pass
        return bin_loc_list

    def can_be_trash_pair(self, trash1, trash2):
        """"
        Return True if trash1, trash2 can be a trash pair, False otherwise.
        trash1, trash2 can be a trash pair if:
        1. trash1 y axis != trash2 y axis
        2. distance(trash1 y axis, trash2 y axis) < self.max_dist_between_trash_pair_y_axis
        """
        trash1_y_axis, trash2_y_axis = trash1.get_curr_position()[1], trash2.get_curr_position()[1]
        return trash1_y_axis != trash2_y_axis and \
               abs(trash1_y_axis - trash2_y_axis) < self.max_dist_between_trash_pair_y_axis

    def calc_trash_picking_point(self, arms, trash_lst):
        """
        Returns best possible picking points (as list) for the given arms to pick *together* the trash from trash_lst
        (according to the order in @param trash_lst).

        Assumes len(trash_lst) == 1 or 2

        If len(trash_lst) == 1, the first arm in arms is used for calculating the picking point
        """
        if len(arms) == 1:
            return [self.calc_single_trash_picking_point(arms[0], trash_lst[0])]
        if len(arms) == 2:
            return self.calc_trash_pair_picking_point(arms, trash_lst)

    def calc_single_trash_picking_point(self, arm, trash):
        """"
        Returns best picking point (from the trash path on the conveyor) for the given arm to pick the trash.
        Best picking point: when y axis value of the trash == y axis value of the arm
        """
        trash_picking_point = list(trash.get_curr_position())
        trash_picking_point[1] = arm.get_pose()[0][1]
        return trash_picking_point

    def calc_trash_pair_picking_point(self, arms, trash_pair):
        """"
        @param arms: list of 2 arms
        @param trash_pair: list of 2 trash to be picked by the arms accordingly (arms[i] picks trash_pair[i])
        Returns best possible picking points (as list) for the given arms to pick the trash pair *together*
        (according to the order in @param trash_pair).
        Best picking point: when the average y axis value of the pair trash == average y axis value of the arms
        """
        arms_y_avg = sum([arm.get_pose()[0][1] for arm in arms]) / 2
        # dst position is same as current position, except for the y axis (the trash moves only on the y axis)
        trash_dst = [list(trash.get_curr_position()) for trash in trash_pair]
        trash_curr_y_loc = [trash.get_curr_position()[1] for trash in trash_pair]
        trash_dst[0][1] = arms_y_avg + (trash_curr_y_loc[0] - trash_curr_y_loc[1]) / 2
        trash_dst[1][1] = arms_y_avg + (trash_curr_y_loc[1] - trash_curr_y_loc[0]) / 2

        return trash_dst

    def add_trash_task_to_arms_group(self, trash_lst: List[Trash], curr_tick: int):
        """"
        @param trash_lst: list of trash objects (len = 1 or 2), sorted by the trash y axis value
        @param curr_tick: the current tick count
        Try to assign trash to some group of arms (defined in self.arms_idx_pairs).
        Returns True if the trash is assigned, False otherwise.

        In one trash case, always gives the task to the first arm in the pair (the first in the pair's list),
        therefore, to check if a pair is free, it's enough to check the first arm of the pair.
        """
        n_trash = len(trash_lst)
        if n_trash > 2:
            print("ERROR: task manager supports max 2 trash for a task")
            return False

        for arm_pair_idx in self.arms_idx_pairs:
            # arm_pair_idx - arms candidate for doing this task
            arms = [self.arms[arm_idx] for arm_idx in arm_pair_idx]
            arms = arms[:n_trash]
            # the match between arms and trash is by their order on the y-axis --> arms[i] picks trash_lst[i]

            # 1. find what will be the picking tick of the task if this arm pair will do the task
            # picking tick := the tick in which the arms will pick the trash
            trash_group_picking_points = self.calc_trash_picking_point(arms, trash_lst)

            # check if the arms can get to the picking points
            for arm, picking_point in zip(arms, trash_group_picking_points):
                if not self.can_arm_get_to_point(arm, picking_point):
                    # arm can't get to the trash
                    continue

            # picking tick is the same for all arms in the group
            picking_tick = curr_tick + self.calculate_ticks_to_destination_on_conveyor(trash_lst[0],
                                                                                       trash_group_picking_points[0])
            # assume that it's enough to check if the first arm of the pair is free
            # start_tick_upper_bound - upper bound for the task's start tick
            start_tick_upper_bound = picking_tick - TICKS_TO_TRASH_LOW_BOUND

            # 2. find the prev_task, next_task with the estimation for the start tick for the first arm
            prev_task, next_task = self.get_task_prev_and_next(arms[0], start_tick_upper_bound)

            # 3. check if the arms are possibly available to do this task
            # 3.1 check if this task needs to begin before prev_task is done
            if prev_task is not None and start_tick_upper_bound <= prev_task.start_tick + prev_task.len_in_ticks:
                # this task needs to begin before prev_task is done, can't assigned it to this arm
                continue
            # 3.2 check if this task will be finished after next_task begins
            if next_task is not None and picking_tick + TICKS_AFTER_GETTING_TO_TRASH_LOW_BOUND >= next_task.start_tick:
                # this task won't finish in time, can't assigned it to this arm
                continue

            # this arms group is possibly free to do this task
            # 4. find motion plan for arms
            bin_dst_loc = self.find_bin_loc_for_arms(trash_lst, arms)

            index_for_arm_tasks_lst = [self.get_index_for_new_task(arm, start_tick_upper_bound) for arm in arms]
            arm_start_conf = []
            for arm, idx in zip(arms, index_for_arm_tasks_lst):
                if idx == 0 or len(self.arms_to_tasks[arm][idx - 1].path_to_bin) == 0:
                    arm_start_conf.append(arm.get_arm_joint_values())

                else:
                    arm_start_conf.append(self.arms_to_tasks[arm][idx - 1].path_to_bin[-1])

            trash_conf = [trash_lst[i].get_trash_config_at_loc(trash_group_picking_points[i])
                          for i in range(n_trash)]
            motion_plan_res = self.background_env.compute_motion_plan(arm_pair_idx[:n_trash],
                                                                      trash_conf,
                                                                      bin_dst_loc, arm_start_conf, self.arms)
            if motion_plan_res is None:
                # couldn't find a path
                continue

            # 4. check if the arms are available to do this task by the motion plan result
            path_to_trash, path_to_bin = motion_plan_res
            len_in_ticks = self.get_ticks_for_full_task_heuristic(len(path_to_trash), len(path_to_bin))

            start_tick = picking_tick - self.get_ticks_for_path_to_trash_heuristic(len(path_to_trash))

            # 4.1 check if this task needs to begin before prev_task is done
            if prev_task is not None and prev_task.start_tick + prev_task.len_in_ticks >= start_tick:
                # this task needs to begin before prev_task is done, can't assigned it to this arm
                continue

            # 4.2 check if this task will be finished after next_task begins
            if next_task is not None and next_task.start_tick <= start_tick + len_in_ticks:
                # this task won't finish in time, can't assigned it to this arm
                continue

            # this arm pair can do the new task!

            path_to_trash_per_arm = split_arms_conf_lst(path_to_trash, n_trash)
            path_to_bin_per_arm = split_arms_conf_lst(path_to_bin, n_trash)

            # 5. add the task to arms
            for i in range(n_trash):
                task = Task(trash_lst[i], arms[i], start_tick, len_in_ticks, path_to_trash_per_arm[i],
                            path_to_bin_per_arm[i], arms)
                self.arms_to_tasks[arms[i]].insert(index_for_arm_tasks_lst[i], task)
            print("Found paths for task!")
            return True
        # no available arm group found
        return False

    def handle_single_trash_that_passed_pnr(self):
        """"
        - Remove all trash from self.single_trash that passed the PNR (point of no return- a point in which this trash
        cannot be paired in the future, and will stay forever alone),
        - Try to assign each of those lonely trash to some arm. If it can't be assigned, give up on this trash,
        it considered as a lost cause, and we won't pick it.
        - the PNR in this case is a value on the 2nd axis.
        """
        for trash in self.single_trash:
            if trash.get_curr_position()[1] - trash_configs.TRASH_INIT_Y_VAL >= self.max_dist_between_trash_pair_y_axis:
                self.single_trash.remove(trash)
                # try to assign a single trash task
                self.add_trash_task_to_arms_group([trash], ticker.now())

    def notify_arms_and_remove_completed_tasks(self) -> None:
        """
        Notify arms about tasks that they should perform in the current tick
        and remove done tasks
        """
        for arm in self.arms:
            idx_of_first_not_done = 0
            for task in self.arms_to_tasks[arm]:
                if task.state == TaskState.DONE:
                    idx_of_first_not_done += 1
                else:
                    if task.start_tick <= ticker.now() and task.state == TaskState.WAIT:
                        # start executing this task
                        task.state = TaskState.DISPATCHED
                        arm.start_task(task)
                    # 3 cases possible here:
                    # - we started executing this task
                    # - this task was already in execute (TaskState.DISPATCHED state)
                    # - this task start_tick > curr_tick (--> all next tasks in the list will also
                    # satisfy this condition)
                    #
                    # in all cases we can stop iterating over this tasks list
                    break
            # remove done tasks
            self.arms_to_tasks[arm] = self.arms_to_tasks[arm][idx_of_first_not_done:]

    def remove_trash(self, trash_id):
        """
        Removes trash:
        - Removes from self.single_trash list (if exists)
        - Removes the Task for this trash (if exists)
        """
        # Removes from self.single_trash
        for idx in range(len(self.single_trash)):
            if self.single_trash[idx].id == trash_id:
                self.single_trash.pop(idx)
                return  # if this trash was in self.single_trash, there is no Task for it
        # Removes the Task for this trash
        for tasks in self.arms_to_tasks.values():
            for task in tasks:
                if task.trash.id == trash_id:
                    # the trash has a task, we want to remove the task
                    # case 1: it's a 1-trash task (involves only one arm)
                    if len(task.arms_involved) == 1:
                        tasks.remove(task)  # simply remove trash
                    # case 2: it's a 2-trash task
                    else:
                        pass
                    return

    def calculate_ticks_to_destination_on_conveyor(self, trash: Trash, trash_dest: List[int]) -> int:
        """
        Calculate the number of ticks it takes to the trash object to get to the
        destination.
        Note that since the conveyor only moves objects in the Y axis, the
        distance is calculated based on that.
        """
        distance_per_tick = 0.0042 * self.trash_velocity
        curr_location = trash.get_curr_position()
        diff = abs(curr_location[1] - trash_dest[1])
        return math.ceil(diff / distance_per_tick)

    def get_ticks_for_full_task_heuristic(self, path_to_trash_len: int, path_to_bin_len: int) -> int:
        """"
        Get estimation for number of ticks for a full task (moving to trash, picking, moving to bin, dropping)
        """
        return self.get_ticks_for_path_to_trash_heuristic(path_to_trash_len) + \
               2 * Robotiq2F85.TICKS_TO_CHANGE_GRIP + \
               self.get_ticks_for_path_to_bin_heuristic(path_to_bin_len)

    @staticmethod
    def can_arm_get_to_point(arm, point):
        arm_loc = arm.get_pose()[0]
        return all([(arm_loc[i] - point[i]) <= ARM_TO_TRASH_MAX_DIST[i] for i in range(2)])

    @staticmethod
    def get_ticks_for_path_to_trash_heuristic(path_len: int) -> int:
        if path_len < 10:
            print('WARNING: unexpected path to trash length, enhance the get_ticks_for_path_to_trash_heuristic')
            return path_len * 40  # arbitrary, we didn't see such examples
        return math.ceil(1 + 2 + 47.4 * (path_len - 2))
        # Explanation:
        # The path is built from ur5 configuration list,
        # each configuration change takes some number of ticks (not fixed).
        # For each path we can calculate the series of #tick it took for each configuration change.
        # We noticed that for MOVING_TO_TRASH path, this "#ticks per configuration" series has a fixed pattern
        # (even when using different trash locations)
        # The pattern we found: 2,x,...,x,1,x,... where x is 47.4 in expectation (#ticks per conf series)

    @staticmethod
    def get_ticks_for_path_to_bin_heuristic(path_len: int) -> int:
        if path_len < 3:
            print('WARNING: unexpected path to trash length, enhance the get_ticks_for_path_to_bin_heuristic')
            return path_len * 40  # arbitrary, we didn't see such examples
        return 1 + 47 + 48 * (path_len - 2)
        # Explanation:
        # The path is built from ur5 configuration list,
        # each configuration change takes some number of ticks (not fixed).
        # For each path we can calculate the series of #tick it took for each configuration change.
        # We noticed that for MOVING_TO_BIN path, this "#ticks per configuration" series has a fixed pattern
        # (even when using different trash locations)
        # The pattern we found: 1,47,48,48,... (#ticks per conf series)


@dataclass
class ArmPair(object):
    """
    Object to handle arm pairs
    """
    arms: List[UR5]
    arms_idx: List[int]
    env: Union[ParallelEnv, BackgroundEnv]


class SimpleTaskManager(TaskManagerComponent):
    """
    This task manager only dispatches tasks to free arms, allowing for an easier
    debugging of everything else.
    """

    def __init__(self, arms: List[UR5], bins: List[Bin], trash_velocity: float, background_env_args: EnvironmentArgs,
                 debug: bool):
        """
        @param arms: A list of all of the available arms
        @param bins: A list of all of the available bins
        @param trash_velocity: The velocity of the trash, we assume velocity only on the Y axis
        @param background_env_args: Arguments to initialize the background environment with
        @param debug: print debug messages flag
        """
        self.arms = arms
        self.bins = bins
        self.trash_velocity = trash_velocity
        self.debug = debug

        self.background_env_args = background_env_args
        self.pairs = self._create_arm_pairs()
        self.closest_bins = {arm: self._find_closest_bins(arm, self.bins) for arm in self.arms}

        self.single_trash = []
        self.arms_to_tasks = {arm: [] for arm in self.arms}

        pair_distance = self.pairs[0].arms[1].get_pose()[0][1] - self.pairs[0].arms[0].get_pose()[0][1]
        self.max_dist_between_trash_pair_y_axis = 2 * ARM_TO_TRASH_MAX_DIST[1] + pair_distance

    def _create_arm_pairs(self) -> List[ArmPair]:
        """
        Create the arms pairs object

        :returns: initialized list of arms pairs
        """
        pairs = []
        assert len(self.arms) % 2 == 0

        background_env = BackgroundEnv(self.background_env_args)
        for i in range(0, len(self.arms), 2):
            arms = [self.arms[i], self.arms[i + 1]]
            arms_idx = [i, i + 1]
            pair = ArmPair(arms, arms_idx, background_env)
            pairs.append(pair)

        return pairs

    def add_trash(self, trash: Trash):
        """
        Try to find a trash pair for @param trash,
        if a pair is found, give this 2 trash task to a pair of arms (if possible)
        otherwise, add @param trash to self.single_trash list

        @param trash: The trash to add
        """
        for older_trash in self.single_trash:
            if self._can_be_trash_pair(trash, older_trash) and self._add_trash_task_to_arms(trash, older_trash):
                self.single_trash.remove(older_trash)
                return

        self.single_trash.append(trash)

    def step(self):
        """
        See TaskManagerComponent
        """
        self._notify_arms_and_remove_completed_tasks()

    def remove_trash(self, trash_uid: int):
        """
        Removes trash from self.single_trash list (if exists). We do not handle
        the case of a trash being a part of a task

        :param trash_uid: ID of the trash object
        """
        for i in range(len(self.single_trash)):
            if self.single_trash[i].id == trash_uid:
                self.single_trash.pop(i)
                return

    def _can_be_trash_pair(self, trash1: Trash, trash2: Trash):
        """"
        Return True if trash1, trash2 can be a trash pair, False otherwise.
        trash1, trash2 can be a trash pair if:
        1. trash1 y axis != trash2 y axis
        2. distance(trash1 y axis, trash2 y axis) < self.max_dist_between_trash_pair_y_axis

        :param trash1: first trash
        :param trash2: second trash

        :returns: True if they can be a pair, False otherwise
        """
        trash1_y = trash1.get_curr_position()[1]
        trash2_y = trash2.get_curr_position()[1]
        diff = abs(trash1_y - trash2_y)

        return 1e-2 < diff < self.max_dist_between_trash_pair_y_axis

    def _add_trash_task_to_arms(self, trash1: Trash, trash2: Trash):
        """
        Try to assign pairs of trash to a group of arms.

        @param trash1: First trash object
        @param trash2: Second trash object

        @return True if the trash objects were assigned, False otherwise
        """
        for pair in self.pairs:
            # Check the arms are free
            if len(self.arms_to_tasks[pair.arms[0]]) != 0:
                continue

            # 1. find what will be the picking tick of the task if this arm pair will do the task
            # picking tick := the tick in which the arms will pick the trash
            trash_pair = [trash1, trash2]
            trash_pair_picking_points = self._calc_trash_group_picking_point(pair.arms, trash_pair)
            # Check if the arms can get to the picking points
            for arm, picking_point in zip(pair.arms, trash_pair_picking_points):
                if not self.can_arm_get_to_point(arm, picking_point):
                    # arm can't get to the trash
                    continue

            picking_tick = ticker.now() + self._calc_ticks_to_destination_on_conveyor(trash_pair[0],
                                                                                      trash_pair_picking_points[0],
                                                                                      self.trash_velocity)

            # 2. find motion plan for arms
            bin_dst_loc = self._find_bin_loc_for_arms(trash_pair, pair.arms)
            arm_start_conf = [arm.get_arm_joint_values() for arm in pair.arms]
            trash_conf = [trash.get_trash_config_at_loc(point) for trash, point in
                          zip(trash_pair, trash_pair_picking_points)]

            real_arm_configs = [arm.get_arm_joint_values() for arm in self.arms]
            motion_plan_res = pair.env.compute_motion_plan(pair.arms_idx,
                                                           trash_conf,
                                                           bin_dst_loc,
                                                           arm_start_conf,
                                                           real_arm_configs)
            if motion_plan_res is None:
                continue

            # 3. this arm pair can do the new task!
            path_to_trash, path_to_bin = motion_plan_res
            len_in_ticks = self._get_ticks_for_full_task_heuristic(len(path_to_trash), len(path_to_bin))
            start_tick = picking_tick - self._get_ticks_for_path_to_trash_heuristic(len(path_to_trash))

            path_to_trash_per_arm = split_arms_conf_lst(path_to_trash, 2)
            path_to_bin_per_arm = split_arms_conf_lst(path_to_bin, 2)

            for i in range(len(pair.arms)):
                task = Task(trash_pair[i], pair.arms[i], start_tick, len_in_ticks, path_to_trash_per_arm[i],
                            path_to_bin_per_arm[i], pair.arms)
                self.arms_to_tasks[pair.arms[i]].append(task)

            print("Found paths for task!")
            return True

        return False

    @staticmethod
    def _calc_trash_group_picking_point(arms: List[UR5], trash_group: List[Trash]):
        """
        :param arms: list of 2 arms
        :param trash_group: list of 2 trash to be picked by the arms accordingly (arms[i] picks trash_pair[i])

        :returns: best possible picking points (as list) for the given arms to pick the trash pair *together*
        (according to the order in @param trash_pair).
        Best picking point: when the average y axis value of the pair trash == average y axis value of the arms
        """
        if len(trash_group) == 1:
            trash_picking_point = list(trash_group[0].get_curr_position())
            trash_picking_point[1] = arms[0].get_pose()[0][1]
            return [trash_picking_point]
        else:
            arms_y_avg = sum([arm.get_pose()[0][1] for arm in arms]) / 2
            # dst position is same as current position, except for the y axis (the trash moves only on the y axis)
            trash_dst = [list(trash.get_curr_position()) for trash in trash_group]
            trash_curr_y_loc = [trash.get_curr_position()[1] for trash in trash_group]
            trash_dst[0][1] = arms_y_avg + (trash_curr_y_loc[0] - trash_curr_y_loc[1]) / 2
            trash_dst[1][1] = arms_y_avg + (trash_curr_y_loc[1] - trash_curr_y_loc[0]) / 2

            return trash_dst

    def _notify_arms_and_remove_completed_tasks(self):
        """
        Notify arms about tasks that they should perform in the current tick
        and remove done tasks
        """
        for arm in self.arms:
            tasks = self.arms_to_tasks[arm]
            completed_tasks = 0
            for task in tasks:
                if task.state == TaskState.DONE:
                    completed_tasks += 1
                elif task.start_tick <= ticker.now() and task.state == TaskState.WAIT and arm.state == ArmState.IDLE:
                    task.state = TaskState.DISPATCHED
                    arm.start_task(task)
            self.arms_to_tasks[arm] = tasks[completed_tasks:]

    def _find_bin_loc_for_arms(self, trash_list: List[Trash], arms: List[UR5]) -> List[List[int]]:
        """
        @param trash_list: list of trash objects we want to recycle.
        @param arms: list of arms that will be used (accordingly)
        assumes len(trash_lst) = len(arms) < 3

        @returns The locations of the closest bin for each arm-trash,
        if both arms need to use the same bin, add some offset to avoid collisions.
        """
        bin_dst_loc = [self.closest_bins[arm][trash.trash_type].location for trash, arm in zip(trash_list, arms)]

        if len(bin_dst_loc) == 2 and bin_dst_loc[0] == bin_dst_loc[1]:
            for i in range(len(bin_dst_loc)):
                arm_loc = arms[i].pose[0]
                arm_loc = np.array(arm_loc)
                bin_loc = bin_dst_loc[i]
                if arm_loc[1] > bin_loc[1]:
                    # The arm is ahead than the bin, therefore the arm needs to go a
                    # little bit further ahead in the y axis
                    for dim in range(3):
                        bin_loc[dim] += ARMS_SAFETY_OFFSET[dim]
                elif arm_loc[1] < bin_loc[1]:
                    # The bin is ahead than the arm, therefore the arm needs to go a
                    # little bit further back in the y axis
                    for dim in range(3):
                        bin_loc[dim] -= ARMS_SAFETY_OFFSET[dim]
                else:
                    # Shouldn't get here
                    # The arm and the bin are at the same Y offset, so no need for
                    # any safety offset (in this case, only 1 arm uses this bin)
                    pass

        return bin_dst_loc

    @staticmethod
    def can_arm_get_to_point(arm: UR5, point: List[float]):
        """
        Check approximately if an arm can get to a point in space

        :param arm: arm to check for
        :param point: destination point of the arm
        """
        arm_loc = arm.get_pose()[0]
        # We only check for x,y and not for z, hence range(2) and not range(3)
        return all([abs(arm_loc[i] - point[i]) <= ARM_TO_TRASH_MAX_DIST[i] for i in range(2)])


class ParallelTaskManager(SimpleTaskManager):
    """
    This task manager dispatches tasks to free arms, but does that in a way
    that doesn't stop the simulation (that is, the path calculation is done
    in parallel to the simulation)
    """
    def __init__(self, arms: List[UR5], bins: List[Bin], trash_velocity: float, background_env_args: EnvironmentArgs,
                 debug: bool):
        """
        @param arms: A list of all of the available arms
        @param bins: A list of all of the available bins
        @param trash_velocity: The velocity of the trash, we assume velocity only on the Y axis
        @param background_env_args: Arguments to initialize the background environment with
        @param debug: print debug messages flag
        """
        super(ParallelTaskManager, self).__init__(arms, bins, trash_velocity, background_env_args, debug)
        self.task_context = {}
        self.sync_back_env = BackgroundEnv(background_env_args)

    def _create_arm_pairs(self) -> List[ArmPair]:
        """
        Create the arms pairs object

        :returns: initialized list of arms pairs
        """
        pairs = []
        assert len(self.arms) % 2 == 0
        for i in range(0, len(self.arms), 2):
            arms = [self.arms[i], self.arms[i + 1]]
            arms_idx = [i, i + 1]
            pair = ArmPair(arms, arms_idx, ParallelEnv(self.background_env_args, arms_idx, self.debug))
            pairs.append(pair)

        return pairs

    def add_trash(self, trash: Trash):
        """
        Try to find a trash pair for @param trash,
        if a pair is found, give this 2 trash task to a pair of arms (if possible)
        otherwise, add @param trash to self.single_trash list

        @param trash: The trash to add
        """
        for older_trash in self.single_trash:
            if self._can_be_trash_pair(trash, older_trash) and self._try_dispatch_trash_to_arms(trash, older_trash):
                self.single_trash.remove(older_trash)
                return

        self.single_trash.append(trash)

    def step(self):
        """
        See TaskManagerComponent
        """
        self._poll_environments()
        self._notify_arms_and_remove_completed_tasks()

    def _poll_environments(self):
        """
        Poll the background environments for completed path calculations
        """
        for pair in self.pairs:
            finished_tasks = pair.env.poll_dispatched_tasks()
            for finished_task in finished_tasks:
                if finished_task.path is None:
                    logging.warning(f'path not found for arms {pair.arms_idx}')
                if finished_task.task_id not in self.task_context:
                    logging.warning(f'wtf is this shit for real???')
                if finished_task.path is not None and finished_task.task_id in self.task_context:
                    # Get all of the data from the task context and the calculation itself
                    path_to_trash, path_to_bin = finished_task.path
                    trash_list = self.task_context[finished_task.task_id]['trash_list']
                    trash_group_picking_points = self.task_context[finished_task.task_id]['trash_group_picking_points']
                    n_trash = len(trash_list)
                    del self.task_context[finished_task.task_id]

                    is_past_picking_point = False
                    for picking_point, trash in zip(trash_group_picking_points, trash_list):
                        if picking_point[1] < trash.get_curr_position()[1]:
                            # trash is past the arm's picking point
                            is_past_picking_point = True
                            logging.warning(f'Picking point was passed, dropping task')
                            break

                    if is_past_picking_point:
                        continue

                    # Calculate heuristics of travel time
                    len_in_ticks = self._get_ticks_for_full_task_heuristic(len(path_to_trash), len(path_to_bin))
                    picking_tick = self._calc_ticks_to_destination_on_conveyor(trash_list[0],
                                                                               trash_group_picking_points[0],
                                                                               self.trash_velocity)
                    picking_tick += ticker.now()

                    path_to_trash_per_arm = split_arms_conf_lst(path_to_trash, 2)
                    path_to_bin_per_arm = split_arms_conf_lst(path_to_bin, 2)
                    start_tick = picking_tick - self._get_ticks_for_path_to_trash_heuristic(len(path_to_trash))

                    for i in range(n_trash):
                        task = Task(trash_list[i], pair.arms[i], start_tick, len_in_ticks, path_to_trash_per_arm[i],
                                    path_to_bin_per_arm[i], pair.arms[:n_trash])
                        self.arms_to_tasks[pair.arms[i]].append(task)

    def _try_dispatch_trash_to_arms(self, trash1: Trash, trash2: Trash):
        """
        Try to dispatch trash pairs to the arms pairs. The task would be
        officially dispatched to a pair when the background environment of that
        pair finished the calculation

        :param trash1: first trash object
        :param trash2: second trash object
        """
        trash_pair = [trash1, trash2]
        task_id = None
        for pair in self.pairs:
            if self.arms_to_tasks[pair.arms[0]] is not None:
                continue

            # 1. find what will be the picking tick of the task if this arm pair will do the task
            # picking tick := the tick in which the arms will pick the trash
            trash_pair_picking_points = self._calc_trash_group_picking_point(pair.arms, trash_pair)

            # 2. Check if the arms can reach the trash
            trash_conf = [trash.get_trash_config_at_loc(point) for trash, point in
                          zip(trash_pair, trash_pair_picking_points)]
            real_arm_configs = [arm.get_arm_joint_values() for arm in self.arms]

            if not self.sync_back_env.can_arms_do_task(pair.arms_idx, trash_conf, real_arm_configs):
                continue

            # 3. Send a motion plan calculation request
            bin_dst_loc = self._find_bin_loc_for_arms(trash_pair, pair.arms)
            arm_start_conf = [arm.get_arm_joint_values() for arm in pair.arms]

            # Generate only once, but only if we actually managed to find suitable arms
            if task_id is None:
                task_id = uuid.uuid4()

            pair.env.dispatch(task_id, pair.arms_idx, trash_conf, bin_dst_loc, arm_start_conf, real_arm_configs)
            self.task_context[task_id] = {
                'trash_list': trash_pair,
                'trash_group_picking_points': trash_pair_picking_points
            }
            return True

        return False


class Timeslot(object):
    """
    Object representing a slot of time
    """
    def __init__(self, start_tick: int, end_tick: int):
        self.start_tick = start_tick
        self.end_tick = end_tick


class AdvancedParallelTaskManager(ParallelTaskManager):
    """
    This task manager dispatches tasks arms in a parallel way, and handles
    both several tasks to some arm, and also arms pairs throwing a single trash
    object
    """
    def __init__(self, arms: List[UR5], bins: List[Bin], trash_velocity: float, background_env_args: EnvironmentArgs,
                 debug: bool):
        """
        @param arms: A list of all of the available arms
        @param bins: A list of all of the available bins
        @param trash_velocity: The velocity of the trash, we assume velocity only on the Y axis
        @param background_env_args: Arguments to initialize the background environment with
        @param debug: print debug messages flag
        """
        super(AdvancedParallelTaskManager, self).__init__(arms, bins, trash_velocity, background_env_args, debug)
        self.timeslots_per_arm: Dict[UR5, List[Timeslot]] = {arm: [] for arm in self.arms}

    def step(self):
        """
        See TaskManagerComponent
        """
        self._poll_environments()
        self._handle_single_trash_that_passed_pnr()
        self._notify_arms_and_remove_completed_tasks()
        self._clear_timeslots()

    def _try_dispatch_trash_to_arms(self, trash1: Trash, trash2: Optional[Trash] = None):
        """
        Try to dispatch trash pairs (or a single trash, if trash2 is None) to
        the arms pairs. The task would be officially dispatched to a pair when
        the background environment of that pair finished the calculation

        :param trash1: first trash object
        :param trash2: second trash object (optional)
        """
        if trash2 is None:
            trash_list = [trash1]
        else:
            trash_list = [trash1, trash2]

        n_trash = len(trash_list)
        task_id = None

        for pair in self.pairs:
            used_arms = pair.arms[:n_trash]

            # 1. find what will be the picking tick of the task if this arm pair will do the task
            # picking tick := the tick in which the arms will pick the trash
            trash_group_picking_points = self._calc_trash_group_picking_point(used_arms, trash_list)
            picking_tick = ticker.now() + self._calc_ticks_to_destination_on_conveyor(trash_list[0],
                                                                                      trash_group_picking_points[0],
                                                                                      self.trash_velocity)
            # Check if the arms can get to the picking points
            for arm, picking_point in zip(pair.arms, trash_group_picking_points):
                if not self.can_arm_get_to_point(arm, picking_point):
                    # arm can't get to the trash
                    continue

            # 2. Check if the arms can reach the trash
            trash_conf = [trash.get_trash_config_at_loc(point) for trash, point in
                          zip(trash_list, trash_group_picking_points)]
            real_arm_configs = [arm.get_arm_joint_values() for arm in self.arms]
            if not self.sync_back_env.can_arms_do_task(pair.arms_idx[:n_trash], trash_conf, real_arm_configs):
                continue

            # 3. Check if the task *approximately* would be possible for the pair
            start_tick_lower_bound = picking_tick - 1000
            end_tick_upper_bound = picking_tick + 1300
            task_timeslot = Timeslot(start_tick_lower_bound, end_tick_upper_bound)
            if not self._can_arms_timeslot_fit(pair, task_timeslot):
                logging.debug(f'Could not fit timeslot')
                continue

            # 4. It is, Send a motion plan calculation request
            bin_dst_loc = self._find_bin_loc_for_arms(trash_list, used_arms)
            arm_start_conf = [arm.get_arm_joint_values() for arm in used_arms]

            if task_id is None:
                task_id = uuid.uuid4()

            pair.env.dispatch(task_id, pair.arms_idx[:n_trash], trash_conf, bin_dst_loc, arm_start_conf, real_arm_configs)
            self.task_context[task_id] = {
                'trash_list': trash_list,
                'trash_group_picking_points': trash_group_picking_points
            }
            for arm in used_arms:
                self.timeslots_per_arm[arm].append(task_timeslot)
            return True

        return False

    def _can_arms_timeslot_fit(self, pair: ArmPair, timeslot: Timeslot):
        """
        Can an arms pair fit this task to its schedule

        :param pair: The pair of arms that might do the task
        :param timeslot: How long the task takes
        """
        pair_timeslots = self.timeslots_per_arm[pair.arms[0]]
        if len(pair_timeslots) == 0:
            return True

        # We assume that timeslots are added in a linear fashion by time
        last_timeslot = pair_timeslots[-1]
        return last_timeslot.end_tick < timeslot.start_tick

    def _handle_single_trash_that_passed_pnr(self):
        """"
        - Remove all trash from self.single_trash that passed the PNR (point of no return- a point in which this trash
        cannot be paired in the future, and will stay forever alone),
        - Try to assign each of those lonely trash to some arm. If it can't be assigned, give up on this trash,
        it considered as a lost cause, and we won't pick it.
        - the PNR in this case is a value on the 2nd axis.
        """
        for trash in self.single_trash:
            if trash.get_curr_position()[1] - trash_configs.TRASH_INIT_Y_VAL >= self.max_dist_between_trash_pair_y_axis:
                self.single_trash.remove(trash)
                # try to assign a single trash task
                self._try_dispatch_trash_to_arms(trash)

    def _clear_timeslots(self):
        """
        Clear timeslots that already finished
        """
        now = ticker.now()
        for pair in self.pairs:
            timeslots = self.timeslots_per_arm[pair.arms[0]]
            while len(timeslots) != 0 and timeslots[0].end_tick < now:
                timeslots.pop(0)
