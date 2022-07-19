import math
import time
from enum import Enum, auto

import numpy as np
import pybullet as p

from background_environment import BackgroundEnv
from multiarm_planner.UR5 import Robotiq2F85, UR5
from trash import Trash
from trash_bin import Bin

ARM_TO_TRASH_MAX_DIST = [0.1, 0.1, 0.1]  # TODO - find real values
TRASH_INIT_Y_VAL = -1  # TODO - make this dynamic
ARMS_SAFETY_OFFSET = 0.1


class TaskState(Enum):
    TASK_WAIT = auto()  # task waits to be executed
    TASK_DISPATCHED = auto()  # task is executed now
    TASK_DONE = auto()  # task finished


class Task(object):
    def __init__(self, trash: Trash, arm: UR5, start_tick: int, len_in_ticks: int, path_to_trash: list,
                 path_to_bin: list):
        """
        @param trash: The trash object
        @param arm: The arm tasked with sorting the trash.
        @param start_tick: The tick number in which the task should be started
        @param len_in_ticks: The amount of ticks it would take to do the task
        @param path_to_trash: list of the arm's configurations for moving to trash from the position before the task
        @param path_to_bin: list of the arm's configurations for moving from trash to bin
        """
        self.trash = trash
        self.arm = arm
        self.start_tick = start_tick
        self.len_in_ticks = len_in_ticks
        self.path_to_trash = path_to_trash
        self.path_to_bin = path_to_bin
        self.state = TaskState.TASK_WAIT


# TODO SHIR - use tick as time instead of real time
class TaskManager(object):
    def __init__(self, arms: list[UR5], arms_idx_pairs: list[list[int]], bins: list[Bin], trash_velocity: float):
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
        """
        self.arms = arms
        # self.available_arms = arms.copy() TODO delete
        self.arms_idx_pairs = arms_idx_pairs
        self.bins = bins
        self.trash_velocity = trash_velocity

        self.background_env = BackgroundEnv(p.DIRECT)

        self.single_trash = []  # unassigned trash, that couldn't be paired yet
        self.waiting_tasks = []  # TODO SHIR- delete this and use arms_to_tasks instead
        self.dispatched_tasks = []  # TODO SHIR- delete this and use arms_to_tasks instead
        self.arms_to_tasks = {arm: [] for arm in
                              self.arms}  # maps arms to a list of their current tasks, ordered by task.start_tick

        # calculate distance between arm pair in y axis, assuming this distance is the same for each pair
        arm_pair0_y_axis = [self.arms[idx].get_pose()[0][1] for idx in
                            arms_idx_pairs[0]]  # list of the y axis value of the arms in pair 0
        arm_pair_dist_y_axis = abs(arm_pair0_y_axis[0] - arm_pair0_y_axis[1])  #

        self.max_dist_between_trash_pair_y_axis = 2 * ARM_TO_TRASH_MAX_DIST[1] + arm_pair_dist_y_axis

    def get_arm_pair(self, arm_idx: int) -> list[int]:
        for pair in self.arms_idx_pairs:
            if arm_idx in pair:
                return pair

    def _find_closest_bin(self, trash: Trash, arm: UR5) -> list[int]:
        """
        Finds the closest bin to the arm, that handles this type of trash.

        @param trash: The trash object we want to recycle.
        @param arm: The arm that we will use.

        @returns The location of the closest bin, with some offset to avoid
        collisions when both arms need to work on that trash bin.
        """
        # TODO: This function is probably useless, we can hardcode it, but I was
        #   too lazy to do it now...
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

        # Return the location of that bin, with some small marginal offset to
        # avoid collisions
        trash_bin_loc = closest_bin.location.copy()
        if arm_loc[1] > trash_bin_loc[1]:
            # The arm is ahead than the bin, therefore the arm needs to go a
            # little bit further ahead
            trash_bin_loc[1] += ARMS_SAFETY_OFFSET
        elif arm_loc[1] < trash_bin_loc[1]:
            # The bin is ahead than the arm, therefore the arm needs to go a
            # little bit further back
            trash_bin_loc[1] -= ARMS_SAFETY_OFFSET
        else:
            # The arm and the bin are at the same Y offset, so no need for
            # any safety offset (in this case, only 1 arm uses this bin)
            pass

        return trash_bin_loc

    # TODO delete ?
    # def _find_adjacent_arms(self):
    #     """
    #     Finds 2 available adjacent arms
    #
    #     @returns the indices of the arms if found, else None
    #     """
    #     for i in range(len(self.available_arms)):
    #         for j in range(i + 1, len(self.available_arms)):
    #             arm1 = self.available_arms[i]
    #             arm2 = self.available_arms[j]
    #             loc1 = arm1.pose[0]
    #             loc2 = arm2.pose[0]
    #             # If they are on the same side, their x coordinates multiplied would be positive
    #             # If they are adjacent, they will have a shift of 1 in the y coordinate
    #             if (loc1[0] * loc2[0] > 0) and abs(loc1[1] - loc2[1]) == 1:
    #                 return i, j
    #     return None

    def _add_task_to_waiting_list(self, task: Task):
        """
        Add the task to the waiting list, sorted by notify time
        """
        for i in range(len(self.waiting_tasks)):
            if task.start_tick < self.waiting_tasks[i].start_tick:
                self.waiting_tasks.insert(i, task)
                return

        self.waiting_tasks.append(task)

    def can_be_trash_pair(self, trash1: Trash, trash2: Trash) -> bool:
        """"
        Return True if trash1, trash2 can be a trash pair, False otherwise.
        trash1, trash2 can be a trash pair if:
        1. trash1 y axis != trash2 y axis
        2. distance(trash1 y axis, trash2 y axis) < self.max_dist_between_trash_pair_y_axis
        """
        trash1_y_axis, trash2_y_axis = trash1.get_curr_position()[1], trash2.get_curr_position()[1]
        return trash1_y_axis != trash2_y_axis and \
               abs(trash1_y_axis - trash2_y_axis) < self.max_dist_between_trash_pair_y_axis

    @staticmethod
    def calc_trash_pair_picking_point(arms: list[UR5], trash_pair: list[Trash]) -> list[list[int]]:
        """"
        @param arms: list of 2 arms
        @param trash_pair: list of 2 trash to be picked by the arms accordingly (arms[i] picks trash_pair[i])
        Returns best possible picking points (as list) for the given arms to pick the trash pair *together*
        (according to the order in @param trash_pair).
        Best picking point: when the average y axis value of the pair trash == average y axis value of the arms
        """
        arms_y_avg = sum([arm.get_pose[0][1] for arm in arms]) / 2
        # dst position is same as current position, except for the y axis (the trash moves only on the y axis)
        trash_dst = [trash.get_curr_position() for trash in trash_pair]
        trash_curr_y_loc = [trash.get_curr_position()[1] for trash in trash_pair]
        trash_dst[0][1] = arms_y_avg + (trash_curr_y_loc[0] - trash_curr_y_loc[1]) / 2
        trash_dst[1][1] = arms_y_avg + (trash_curr_y_loc[1] - trash_curr_y_loc[0]) / 2

        return trash_dst

    def add_trash_pair_task_to_arm_pair(self, trash1: Trash, trash2: Trash):
        """"
        Try to assign trash1, trash2 pair to some pair of arms.
        Returns True if the pair is assigned, False otherwise.
        @pre: trash1 y axis < trash2 y axis
        """
        # TODO SHIR
        pass

    @staticmethod
    def calc_single_trash_picking_point(arm: UR5, trash: Trash) -> list[int]:
        """"
        Returns best picking point (from the trash path on the conveyor) for the given arm to pick the trash.
        Best picking point: when y axis value of the trash == y axis value of the arm
        """
        trash_picking_point = list(trash.get_curr_position())
        trash_picking_point[1] = arm.get_pose()[0][1]
        return trash_picking_point

    def add_single_trash_task_to_arm(self, trash: Trash):
        """"
        Try to assign @param trash to some pair of arms.
        Returns True if the trash is assigned, False otherwise.
        Always gives the task to the first arm in the pair (the first in the pair's list),
        therefore, to check if a pair is free, it's enough to check the first arm of the pair.
        """
        for arm_idx, _ in self.arms_idx_pairs:
            arm = self.arms[arm_idx]
            # find what will be the start tick of the task if this arm will do the task
            trash_picking_point = self.calc_single_trash_picking_point(arm, trash)
            start_tick = self.calculate_ticks_to_destination_on_conveyor(trash, trash_picking_point)
            # check if the arm is unoccupied from start_tick (and after)
            last_assigned_task = None if len(self.arms_to_tasks[arm]) == 0 else self.arms_to_tasks[arm][-1]
            if last_assigned_task is None or start_tick > last_assigned_task.start_tick + last_assigned_task.len_in_ticks:
                # this arm can do the new task!
                # assign this task to the arm
                bin_dst_loc = self._find_closest_bin(trash, arm)
                arm_start_conf = arm.get_arm_joint_values() if last_assigned_task is None else \
                    last_assigned_task.path_to_bin[-1]  # TODO SHIR - verify this is ok
                path_to_trash, path_to_bin = self.background_env.compute_motion_plan([arm_idx],
                                                                                     [trash.get_trash_config_at_loc(
                                                                                         trash_picking_point)],
                                                                                     [bin_dst_loc], [arm_start_conf])
                len_in_ticks = self.get_ticks_for_full_task_heuristic(len(path_to_trash),
                                                                      len(path_to_bin))  # TODO SHIR - check that I got right the conf list structure
                task = Task(trash, arm, start_tick, len_in_ticks, path_to_trash, path_to_bin)
                self.arms_to_tasks[arm].append(task)

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
                if self.add_trash_pair_task_to_arm_pair(trash, older_trash):
                    # this pair trash is assigned to some pair of arms
                    return
        # failed to assign this trash, add trash to self.single_trash
        self.single_trash.append(trash)

    def handle_single_trash_that_passed_pnr(self):
        """"
        - Remove all trash from self.single_trash that passed the PNR (point of no return- a point in which this trash
        cannot be paired in the future, and will stay forever alone),
        - Try to assign each of those lonely trash to some arm. If it can't be assigned, give up on this trash,
        it considered as a lost cause, and we won't pick it.
        - the PNR in this case is a value on the 2nd axis.
        """
        for trash in self.single_trash:
            if trash.get_curr_position()[1] - TRASH_INIT_Y_VAL >= self.max_dist_between_trash_pair_y_axis:
                self.single_trash.remove(trash)
                # try to assign a single trash task
                self.add_single_trash_task_to_arm(trash)

    # TODO - delete?
    # def try_dispatch_tasks(self):
    #     """
    #     Try to dispatch trash objects to arms
    #     """
    #     for i in range(len(self.unassigned_trash)):
    #         # Small trash
    #         if self.unassigned_trash[i].trash_size == 1:
    #             if len(self.available_arms) > 0:
    #                 arm = self.available_arms[0]
    #                 trash = self.unassigned_trash[i]
    #                 trash_bin = self._find_closest_bin(trash, [arm, ])
    #                 task = Task(trash, [arm, ], trash_bin, self.trash_velocity)
    #                 self.unassigned_trash[i] = None
    #                 self.available_arms.pop(0)
    #                 self._add_task_to_waiting_list(task)
    #
    #                 # arm.add_task(task)
    #                 # # TODO: Fix this - trash generation should be dynamic
    #                 # path_to_trash = self.background_env.compute_motion_plan({self.arms.index(arm): (trash_configs.TrashConfig.MUSTARD, task.trash.location)})
    #                 # path_to_bin = self.background_env.path_to_bin([self.arms.index(arm)], task.dest.location, [path_to_trash[-1]])
    #                 # arm.add_path(path_to_trash)
    #                 # arm.add_path(path_to_bin)
    #
    #         # Big trash
    #         else:
    #             pass
    #             # arms_indices = self._find_adjacent_arms()
    #             # if arms_indices is not None:
    #             #     idx1, idx2 = arms_indices
    #             #     arm1 = self.available_arms[idx1]
    #             #     arm2 = self.available_arms[idx2]
    #             #     trash = self.unassigned_trash[i]
    #             #     trash_bin = self._find_closest_bin(trash, [arm1, arm2])
    #             #     task = Task(trash, [arm1, arm2], trash_bin, self.trash_velocity)
    #             #     self.unassigned_trash[i] = None
    #             #     self.available_arms = self.available_arms[:idx1] + \
    #             #                           self.available_arms[idx1 + 1:idx2] + \
    #             #                           self.available_arms[idx2 + 1:]
    #             #     self._add_task_to_waiting_list(task)

    # Remove assigned trash
    # self.single_trash = list(filter(lambda x: x is not None, self.single_trash))

    def notify_arms(self, curr_tick):
        """
        Notify arms about tasks that they should perform
        """
        i = 0
        awakened_tasks = []
        for i in range(len(self.waiting_tasks)):
            # Search for the tasks that should still be waiting
            if self.waiting_tasks[0].notify_time > curr_tick:
                awakened_tasks = self.waiting_tasks[:i]
                self.waiting_tasks = self.waiting_tasks[i:]
                break

        # If all of the tasks should be awakened:
        if i == len(self.waiting_tasks) - 1:
            awakened_tasks = self.waiting_tasks
            self.waiting_tasks = []

        # Change state of tasks here to TASK_DISPATCHED
        for task in awakened_tasks:
            task.state = TaskState.TASK_DISPATCHED
            # for arm in task.arms:
            #     arm.start_task()

        # TODO: Ask Nir how to actually talk to the arms

        # Move tasks to the dispatched tasks list
        if len(awakened_tasks) > 0:
            print(f'new tasks len: {len(awakened_tasks)}')
        self.dispatched_tasks.extend(awakened_tasks)

    def remove_completed_tasks(self):
        """
        Remove tasks that were completed
        """
        self.dispatched_tasks = list(filter(lambda x: x.state != TaskState.TASK_DONE, self.dispatched_tasks))

    def remove_uncaught_trash_task(self, trash_id: int):
        """
        Remove failed tasks from the tasks list
        Technically, this doesn't actually remove the task, but just changes the
        state of the task to TASK_DONE (and `remove_completed_tasks` would
        actually remove it)
        """
        for task in self.dispatched_tasks:
            if task.trash.id == trash_id:
                task.state = TaskState.TASK_DONE
                return

    @staticmethod
    def calculate_ticks_to_destination_on_conveyor(trash: Trash, trash_dest: list[int]) -> float:
        """
        Calculate the number of ticks it takes to the trash object to get to the
        destination.
        Note that since the conveyor only moves objects in the Y axis, the
        distance is calculated based on that.
        """
        curr_location = trash.get_curr_position()
        diff = abs(curr_location[1] - trash_dest[1])
        return math.ceil(diff / 0.00104)
        # TODO - 0.00104 is based on the current conveyor speed, change this to be general for every conveyor speed

    def get_ticks_for_full_task_heuristic(self, path_to_trash_len: int, path_to_bin_len: int) -> int:
        """"
        Get estimation for number of ticks for a full task (moving to trash, picking, moving to bin, dropping)
        """
        # TODO SHIR- verify this is true (I think maybe it should be Robotiq2F85.TICKS_TO_CHANGE_GRIP +1)
        return self.get_ticks_for_path_to_trash_heuristic(path_to_trash_len) + \
               2 * Robotiq2F85.TICKS_TO_CHANGE_GRIP + \
               self.get_ticks_for_path_to_bin_heuristic(path_to_bin_len)

    @staticmethod
    def get_ticks_for_path_to_trash_heuristic(path_len: int) -> float:
        if path_len < 10:
            print('WARNING: unexpected path to trash length, enhance the get_ticks_for_path_to_trash_heuristic')
            return path_len * 40  # arbitrary, we didn't see such examples
        return 1 + 2 + 47.4 * (path_len - 2)
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
