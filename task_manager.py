import math
import time
from enum import Enum, auto

import numpy as np


class TaskState(Enum):
    TASK_WAIT = auto()
    TASK_DISPATCHED = auto()
    TASK_DONE = auto()


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


class TaskManager(object):
    def __init__(self, arms, bins, trash_velocity):
        """
        @param arms: A list of all of the available arms
        @param bins: A list of all of the available bins
        @param trash_velocity: The velocity of the trash, we assume velocity
        only on the Y axis.
        """
        self.available_arms = arms.copy()
        self.bins = bins
        self.trash_velocity = trash_velocity

        self.unassigned_trash = []
        self.waiting_tasks = []
        self.dispatched_tasks = []

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

    def _find_adjacent_arms(self):
        """
        Finds 2 available adjacent arms

        @returns the indices of the arms if found, else None
        """
        for i in range(len(self.available_arms)):
            for j in range(i + 1, len(self.available_arms)):
                arm1 = self.available_arms[i]
                arm2 = self.available_arms[j]
                loc1 = arm1.pose[0]
                loc2 = arm2.pose[0]
                # If they are on the same side, their x coordinates multiplied would be positive
                # If they are adjacent, they will have a shift of 1 in the y coordinate
                if (loc1[0] * loc2[0] > 0) and abs(loc1[1] - loc2[1]) == 1:
                    return i, j
        return None

    def _add_task_to_waiting_list(self, task):
        """
        Add the task to the waiting list, sorted by notify time
        """
        for i in range(len(self.waiting_tasks)):
            if task.notify_time < self.waiting_tasks[i].notify_time:
                self.waiting_tasks.insert(i, task)
                return

        self.waiting_tasks.append(task)

    def add_trash(self, trash):
        """
        Add a trash object to the list of trash we need to manage
        @param trash: The trash to add
        """
        self.unassigned_trash.append(trash)

    def try_dispatch_tasks(self):
        """
        Try to dispatch trash objects to arms
        """
        for i in range(len(self.unassigned_trash)):
            # Small trash
            if self.unassigned_trash[i].trash_size == 1:
                if len(self.available_arms) > 0:
                    arm = self.available_arms[0]
                    trash = self.unassigned_trash[i]
                    trash_bin = self._find_closest_bin(trash, [arm, ])
                    task = Task(trash, [arm, ], trash_bin, self.trash_velocity)
                    self.unassigned_trash[i] = None
                    self.available_arms.pop(0)
                    self._add_task_to_waiting_list(task)
            # Big trash
            else:
                arms_indices = self._find_adjacent_arms()
                if arms_indices is not None:
                    idx1, idx2 = arms_indices
                    arm1 = self.available_arms[idx1]
                    arm2 = self.available_arms[idx2]
                    trash = self.unassigned_trash[i]
                    trash_bin = self._find_closest_bin(trash, [arm1, arm2])
                    task = Task(trash, [arm1, arm2], trash_bin, self.trash_velocity)
                    self.unassigned_trash[i] = None
                    self.available_arms = self.available_arms[:idx1] + \
                                          self.available_arms[idx1 + 1:idx2] + \
                                          self.available_arms[idx2 + 1:]
                    self._add_task_to_waiting_list(task)

        # Remove assigned trash
        self.unassigned_trash = list(filter(lambda x: x is not None, self.unassigned_trash))

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

        # TODO: Ask Nir how to actually talk to the arms

        # Move tasks to the dispatched tasks list
        if len(awakened_tasks) > 0:
            print(f'new tasks len: {len(awakened_tasks)}')
        self.dispatched_tasks.extend(awakened_tasks)

    def remove_completed_tasks(self):
        """
        Remove tasks that were completed, and return the arms back to the
        available arms list
        """
        for task in self.dispatched_tasks:
            if task.state == TaskState.TASK_DONE:
                self.available_arms.extend(task.arms)

        # Remove the completed tasks
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
