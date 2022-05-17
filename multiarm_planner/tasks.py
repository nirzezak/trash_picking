import os
from time import sleep
from decimal import Decimal
from itertools import product
import numpy as np
from json import load, dump
from pathlib import Path
from os.path import abspath
from random import shuffle as shuffle_f
from os.path import basename, exists
from numpy.linalg import norm


def drange(x, y, jump):
    while x < y:
        yield float(x)
        x += Decimal(jump)


class Task:
    def __init__(self,
                 target_eff_poses,
                 base_poses,
                 start_config,
                 goal_config,
                 obstacles=[],
                 difficulty=None,
                 dynamic_speed=None,
                 start_goal_config=None,
                 task_path=None):
        self.start_config = start_config
        self.base_poses = base_poses
        self.goal_config = goal_config
        self.start_goal_config = start_goal_config  # for dynamic tasks
        if self.start_goal_config is None:
            self.start_goal_config = [None for _ in goal_config]
        self.target_eff_poses = target_eff_poses
        self.obstacles = obstacles
        self.ur5_count = len(self.start_config)
        self.task_path = task_path
        self.dynamic_speed = dynamic_speed
        if self.task_path is not None:
            self.id = str(basename(self.task_path)).split('.')[0]
        else:
            self.id = -1
        self.difficulty = difficulty
        
    def save(self):
        if self.task_path is not None:
            dump(self.to_json(), open(self.task_path, 'w'), indent=4)

    @staticmethod
    def compute_speed(task, fk,
                      episode_time=20,
                      time_slice_count=100):
        speeds = []
        angular_speeds = []
        start_configs = np.array(task.start_goal_config)
        final_configs = np.array(task.goal_config)
        delta_time = episode_time / time_slice_count
        for ur5_idx, (start, final) in enumerate(zip(
                start_configs, final_configs)):
            pose1 = fk(start)
            pose2 = fk(start + (final - start) / time_slice_count)
            displacement = norm(np.array(pose2[0]) - np.array(pose1[0]))
            angular_displacement = abs((
                np.quaternion(*pose2[1])
                * np.quaternion(*pose1[1]).inverse()).angle())
            speeds.append(displacement / delta_time)
            angular_speeds.append(angular_displacement / delta_time)
        return {
            'speed': {
                'max': max(speeds),
                'mean': np.mean(speeds),
                'min': min(speeds),
            },
            'angular-speed': {
                'max': max(angular_speeds),
                'mean': np.mean(angular_speeds),
                'min': min(angular_speeds),
            }
        }

    def __iter__(self):
        self._itr_idx = 0
        return self

    def __next__(self):
        itr_idx = self._itr_idx

        # Stop iteration if limit is reached
        if itr_idx > self.ur5_count:
            raise StopIteration

        # Else increment and return old value
        self._itr_idx = itr_idx + 1
        return {
            'target_eef_pose': self.target_eff_poses[itr_idx],
            'start_goal_config': self.start_goal_config[itr_idx],
            'start_config': self.start_config[itr_idx],
            'goal_config': self.goal_config[itr_idx],
            'base_pose': self.base_poses[itr_idx]
        }

    def to_json(self):
        return {
            'target_eff_poses': self.target_eff_poses,
            'start_goal_config': [None if c is None else list(c)
                                  for c in self.start_goal_config],
            'start_config': [list(c) for c in self.start_config],
            'goal_config': [list(c) for c in self.goal_config],
            'base_poses': self.base_poses,
            'difficulty': self.difficulty,
            'dynamic_speed': self.dynamic_speed,
            'ur5_count': self.ur5_count,
            'id': self.id,
            'task_path': self.task_path,
            'obstacles': self.obstacles
        }

    @staticmethod
    def from_file(task_path):
        try:
            task_file = load(open(task_path))
        except Exception as e:
            print(task_path)
            print(e)
            return None
        return Task(
            start_config=task_file['start_config'],
            base_poses=task_file['base_poses'],
            goal_config=task_file['goal_config'],
            start_goal_config=None
            if 'start_goal_config' not in task_file
            else task_file['start_goal_config'],
            difficulty=None
            if 'difficulty' not in task_file
            else task_file['difficulty'],
            obstacles=[]
            if 'obstacles' not in task_file
            else task_file['obstacles'],
            dynamic_speed=None
            if 'dynamic_speed' not in task_file
            else task_file['dynamic_speed'],
            target_eff_poses=task_file['target_eff_poses'],
            task_path=task_path)

    @staticmethod
    def compute_task_difficulty(
        task,
        w_radius=0.85  # UR5 reach radius
    ):
        w_positions = [np.array(pose[0]) for pose in task.base_poses]
        # Hard code 0.0 difficulty cases
        if len(w_positions) < 2:
            return 0.0
        distances = [norm(pos1 - pos2)
                     for pos1, pos2 in product(w_positions, repeat=2)]
        distances = [d for d in distances if d != 0.0]
        if all([d > 2 * w_radius for d in distances]):
            return 0.0

        resolution = 0.05
        workspace_intersection_percentages = []

        # Sample workspace volume
        for w_pos in w_positions:
            workspace_intersection = 0
            workspace_volume = 0
            others = [p for p in w_positions if norm(p - w_pos) != 0.0]
            x_min = Decimal(w_pos[0] - w_radius)
            x_max = Decimal(w_pos[0] + w_radius)
            y_min = Decimal(w_pos[1] - w_radius)
            y_max = Decimal(w_pos[1] + w_radius)
            for x in drange(x_min, x_max, str(resolution)):
                for y in drange(y_min, y_max, str(resolution)):
                    for z in drange(0, w_radius, str(resolution)):
                        point = np.array([x, y, z])
                        inside_workspace = norm(point - w_pos) < w_radius
                        if not inside_workspace:
                            continue
                        workspace_volume += 1
                        if any([norm(point - other) < w_radius
                                for other in others]):
                            workspace_intersection += 1
            workspace_intersection_percentages.append(
                workspace_intersection / workspace_volume)
        return max(workspace_intersection_percentages)


class TaskLoader:
    def __init__(self, root_dir, shuffle=False, repeat=True):
        if exists(os.path.join(root_dir, 'all.txt')):
            print("[TaskLoader] Getting tasks from all.txt")
            with open(os.path.join(root_dir, 'all.txt')) as f:
                self.files = [abspath(os.path.join(root_dir, file_path.rstrip()))
                              for file_path in f.readlines()]
        else:
            self.files = [abspath(file_name)
                          for file_name in Path(root_dir).rglob('*.json')
                          if 'config' not in str(file_name)]
        self.repeat = repeat
        if not self.repeat:
            print("[TaskLoader] WARNING: not repeating tasks.")
        assert len(self.files) > 0
        print("[TaskLoader] Found {} tasks".format(
            len(self.files)))
        if shuffle:
            shuffle_f(self.files)
        self.current_idx = 0
        self.count = len(self.files)
        self.succeeded = []

    def __len__(self):
        return len(self.files)

    def __iter__(self):
        self.current_idx = 0
        return self

    def __next__(self):
        self.current_idx += 1
        if self.current_idx - 1 == len(self.files):
            raise StopIteration
        return Task.from_file(
            task_path=self.files[self.current_idx - 1])

    def get_num_targets(self):
        return len(self.files)

    def get_next_task(self):
        if self.current_idx >= len(self.files)\
                and not self.repeat:
            print("[TargetLoader] Out of targets")
            while True:
                sleep(5)
        current_file = self.files[self.current_idx]
        self.current_idx = self.current_idx + 1
        if self.repeat:
            self.current_idx = self.current_idx % len(self.files)
        return Task.from_file(task_path=current_file)

    def on_success(self, task_path):
        self.succeeded.append(task_path)
        print(f"[TaskLoader] On Success {len(self.succeeded)/self.count}")
        self.files.remove(task_path)
        if len(self.files) < 5:
            self.files.extend(self.succeeded)
            self.succeeded = []
            print("[TaskLoader] New Level!")

    def set_repeat(self, val):
        self.repeat = val
