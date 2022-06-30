import math
import time
from enum import Enum, auto

import numpy as np
import pybullet as p

import trash_configs
from background_environment import BackgroundEnv

ARM_TO_TRASH_MAX_DIST = [1, 1, 1]  # TODO - find real values


class TaskState(Enum):
    TASK_WAIT = auto()
    TASK_DISPATCHED = auto()
    TASK_DONE = auto()


# TODO SHIR - fit this for pairs ?
class Task(object):
    def __init__(self, trash, arms, dest, trash_velocity):
        """
        @param trash: The trash object
        @param arms: The arms tasked with sorting the trash. It can be either
        one arm or 2 arms. In both cases, it must be a list!
        @param dest: The destination of the trash - AKA the trash bin
        @param trash_velocity: The velocity of the trash, we assume velocity
        only on the Y axis.
        """
        self.trash = trash
        self.arms = arms
        self.dest = dest

        # Calculate approximate travel time, using the speed of the object, and
        # the distance between the arms and the trash
        if len(self.arms) == 2:
            # Calculate the distance from their halfway point
            y1 = self.arms[0].pose[0][1]
            y2 = self.arms[0].pose[0][1]
            y_arm = (y1 + y2) / 2
        else:
            y_arm = self.arms[0].pose[0][1]

        y_trash = self.trash.location[1]

        self.travel_time = (y_arm - y_trash) / trash_velocity + time.time()
        # Notify 1 second before the travel time to the destination
        self.notify_time = self.travel_time - 1
        self.state = TaskState.TASK_WAIT


# TODO SHIR - use tick as time instead of real time
class TaskManager(object):
    def __init__(self, arms, arms_idx_pairs, bins, trash_velocity):
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
        self.waiting_tasks = []
        self.dispatched_tasks = []

        # calculate distance between arm pair in y axis, assuming this distance is the same for each pair
        arm_pair0_y_axis = [self.arms[idx].get_pose()[0][1] for idx in arms_idx_pairs[0]]  # list of the y axis value of the arms in pair 0
        arm_pair_dist_y_axis = abs(arm_pair0_y_axis[0] - arm_pair0_y_axis[1])  #

        self.max_dist_between_trash_pair_y_axis = 2 * ARM_TO_TRASH_MAX_DIST[1] + arm_pair_dist_y_axis

    def get_arm_pair(self, arm_idx):
        for pair in self.arms_idx_pairs:
            if arm_idx in pair:
                return pair

    def _find_closest_bin(self, trash, arms):
        # TODO: This function is probably useless, we can hardcode it, but I was
        # too lazy to do it now...
        if len(arms) == 2:
            # Calculate the distance from their halfway point
            y1 = arms[0].pose[0][1]
            y2 = arms[0].pose[0][1]
            y_arm = (y1 + y2) / 2
        else:
            y_arm = arms[0].pose[0][1]

        arms_avg_loc = arms[0].pose[0].copy()
        arms_avg_loc[1] = y_arm
        arms_avg_loc = np.array(arms_avg_loc)

        closest_bin_distance = math.inf
        closest_bin = None
        for trash_bin in self.bins:
            if trash_bin.trash_type == trash.trash_type:
                # Calculate distance
                trash_bin_loc = np.array(trash_bin.location)
                distance = np.linalg.norm(arms_avg_loc - trash_bin_loc)
                if distance < closest_bin_distance:
                    closest_bin_distance = distance
                    closest_bin = trash_bin

        return closest_bin

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

    def _add_task_to_waiting_list(self, task):
        """
        Add the task to the waiting list, sorted by notify time
        """
        for i in range(len(self.waiting_tasks)):
            if task.notify_time < self.waiting_tasks[i].notify_time:
                self.waiting_tasks.insert(i, task)
                return

        self.waiting_tasks.append(task)

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

    def add_trash_pair_task_to_arm_pair(self, trash1, trash2):
        """"
        Try to assign trash1, trash2 pair to some pair of arms.
        Returns True if the pair is assigned, False otherwise.
        @pre: trash1 y axis < trash2 y axis
        """
        # TODO SHIR
        pass

    def add_single_trash_task_to_arm(self, trash):
        """"
        Try to assign @param trash to some pair of arms.
        Returns True if the trash is assigned, False otherwise.
        """
        # TODO SHIR
        pass

    def add_trash(self, trash):
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
            if trash.get_curr_position()[2] >= self.max_dist_between_trash_pair_y_axis:
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

    def notify_arms(self):
        """
        Notify arms about tasks that they should perform
        """
        curr_time = time.time()
        i = 0
        awakened_tasks = []
        for i in range(len(self.waiting_tasks)):
            # Search for the tasks that should still be waiting
            if self.waiting_tasks[0].notify_time > curr_time:
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

    def remove_uncaught_trash_task(self, trash_id):
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
    def get_ticks_for_path_to_trash_heuristic(path_len: int):
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
    def get_ticks_for_path_to_bin_heuristic(path_len: int):
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
