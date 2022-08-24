from __future__ import annotations

from enum import Enum, auto

from typing import List, Dict

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from multiarm_planner.ur5 import UR5
    from trash import Trash


class TaskState(Enum):
    WAIT = auto()  # task waits to be executed
    DISPATCHED = auto()  # task is executed now
    DONE = auto()  # task finished


class Task(object):
    def __init__(self, trash: Trash, arm: UR5, start_tick: int, len_in_ticks: int, path_to_trash: list,
                 path_to_bin: list, arms_involved: List[UR5]):
        """
        @param trash: The trash object
        @param arm: The arm tasked with sorting the trash.
        @param start_tick: The tick number in which the task should be started
        @param len_in_ticks: The amount of ticks it would take to do the task
        @param path_to_trash: list of the arm's configurations for moving to trash from the position before the task
        @param path_to_bin: list of the arm's configurations for moving from trash to bin
        @param arms_involved: the arms operating in parallel in this motion plan
        (all the arms that were part of the motion plan)
        """
        self.trash = trash
        self.arm = arm
        self.start_tick = start_tick
        self.len_in_ticks = len_in_ticks
        self.path_to_trash = path_to_trash
        self.path_to_bin = path_to_bin
        self.state = TaskState.WAIT
        self.arms_involved = arms_involved


class PendingTask(object):
    def __init__(self, task_id, arms_idx: List[int], trash_conf: List[Dict], bin_locations, start_configs,
                 real_arms_configs=None):
        self.task_id = task_id
        self.arms_idx = arms_idx
        self.trash_conf = trash_conf
        self.bin_locations = bin_locations
        self.start_configs = start_configs
        self.real_arms_configs = real_arms_configs


class PendingTaskResult(object):
    def __init__(self, task_id, path):
        self.task_id = task_id
        self.path = path
