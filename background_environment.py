import logging
import multiprocessing as mp
from typing import List, Dict

import pybullet as p
import numpy as np

import utils
from environment import Environment, EnvironmentArgs
from loggers import init_loggers
from multiarm_planner.multiarm_environment import split_arms_conf, split_arms_conf_lst
from task import PendingTask, PendingTaskResult

MAX_ATTEMPTS_TO_FIND_PATH = 3


class BackgroundEnv(Environment):
    def __init__(self, env_args: EnvironmentArgs):
        """"
        :param env_args: arguments on how to initialize the environment
        """
        super().__init__(env_args.connection_mode, 0, env_args.arms_path, env_args.trash_bins_path,
                         set_pybullet_utils_p=True)

    def can_arms_do_task(self, arms_idx, trash, real_arms_configs):
        """
        Check if an arm can do a task, by checking if the arms reach all of the
        trash objects

        @param arms_idx: arm indices to find paths for
        @param trash: list of trash configs, same order as in arms_idx
        @param real_arms_configs: list of arms in the real environment, same order as in the background environment

        @returns True if they reach, False otherwise
        """
        if real_arms_configs is not None:
            self.sync_arm_positions(real_arms_configs)

        arms_to_above_position_configs = []

        for arm_idx, trash_conf in zip(arms_idx, trash):
            trash = self.trash_generator.summon_trash(trash_conf)
            grip_point = trash.get_curr_gripping_points()[0]
            self.trash_generator.remove_trash()

            above_grip_point = grip_point.copy()
            above_grip_point[2] += 0.3

            if self.arms[arm_idx].is_right_arm:
                orientation = p.getQuaternionFromEuler([0, np.pi / 2, -np.pi / 2])

            else:
                orientation = p.getQuaternionFromEuler([0, np.pi / 2, np.pi / 2])

            end_pos = [above_grip_point, orientation]
            arms_to_above_position_configs.append(end_pos)

        return all(
            self.does_arm_reach(arm_idx, arms_to_above_position_configs[idx][0], arms_to_above_position_configs[idx][1])
            for idx, arm_idx in enumerate(arms_idx)
        )

    def compute_motion_plan(self, arms_idx, trash, bin_locations, start_configs, real_arms_configs=None):
        """"
        @param arms_idx: arm indices to find paths for
        @param trash: list of trash configs, same order as in arms_idx
        @param bin_locations: list of bin locations, same order as in arms_idx
        @param start_configs: list of start configs of the arms, same order as in arms_idx
        @param real_arms_configs: list of arms in the real environment, same order as in the background environment

        @returns:
        if path is found, returns paths to trash, paths to bin
        paths to trash - a list of paths to appropriate trash, same order as in arms_idx
        paths to bins - a list of paths to appropriate bins, same order as in arms_idx

        if no path found, return None
        """
        if real_arms_configs is not None:
            self.sync_arm_positions(real_arms_configs)

        paths_to_trash = self.compute_path_to_trash(arms_idx, trash, start_configs)
        if paths_to_trash is None:
            logging.info('Path to trash not found')
            return None
        last_configs = split_arms_conf(paths_to_trash[-1], len(trash))
        paths_to_bins = self.compute_path_to_bin(arms_idx, bin_locations, last_configs)
        if paths_to_bins is None:
            logging.info('Path to bin not found')
            return None
        return paths_to_trash, paths_to_bins

    def compute_path_to_trash(self, arms_idx, trash, start_configs):
        """"
        @param arms_idx: arm indices to find paths for
        @param trash: list of trash configs, same order as in arms_idx
        @param start_configs: list of start configs of the arms, same order as in arms_idx

        @returns a list of paths to appropriate trash, same order as in arms_idx
        if no path found, return None.
        """
        n_arms = len(arms_idx)
        arms_to_above_position_configs = []
        arms_to_actual_goal_configs = []

        for arm_idx, trash_conf in zip(arms_idx, trash):
            trash = self.trash_generator.summon_trash(trash_conf)
            grip_point = trash.get_curr_gripping_points()[0]

            self.trash_generator.remove_trash()

            above_grip_point = grip_point.copy()
            above_grip_point[2] += 0.3

            if self.arms[arm_idx].is_right_arm:
                orientation = p.getQuaternionFromEuler([0, np.pi / 2, -np.pi / 2])

            else:
                orientation = p.getQuaternionFromEuler([0, np.pi / 2, np.pi / 2])

            end_pos = [above_grip_point, orientation]  # This orientation is "from above"
            arms_to_above_position_configs.append(end_pos)

            # Then hopefully find a path that only lowers the arm to pick up the trash
            end_pos2 = [grip_point, orientation]
            arms_to_actual_goal_configs.append(end_pos2)

        # Verify the arms can reach the above position - assume this is good enough and that the arms will be
        #   able to reach the downwards config as well
        if not all(
            self.does_arm_reach(arm_idx, arms_to_above_position_configs[idx][0], arms_to_above_position_configs[idx][1])
            for idx, arm_idx in enumerate(arms_idx)
        ):
            return None

        # First move to base config, and only then look for path to trash
        goal_configs = []
        for arm_idx, position in zip(arms_idx, arms_to_above_position_configs):
            goal_config = self.arms[arm_idx].inverse_kinematics(*position)
            base_config = self.arms[arm_idx].base_config.copy()

            # Prevent overshooting by controlling the rotation of the arm (joint 1) - rotate only as much as needed to pick up the trash
            base_config[0] = goal_config[0]

            goal_configs.append(base_config)

        path_to_base = self.arms_manager.birrt(
            [self.arms[arm_idx] for arm_idx in arms_idx],
            goal_configs=goal_configs,
            start_configs=start_configs, max_attempts=MAX_ATTEMPTS_TO_FIND_PATH,
            collision_distance=0.15
        )

        if path_to_base is not None:
            path_to_base_conf_per_arm = split_arms_conf(path_to_base[-1], n_arms)

        else:
            # Skip path to base if one is not found
            path_to_base = []
            path_to_base_conf_per_arm = start_configs

        path_to_above_position = self.arms_manager.birrt(
            [self.arms[arm_idx] for arm_idx in arms_idx],
            goal_positions=arms_to_above_position_configs,
            start_configs=path_to_base_conf_per_arm,
            max_attempts=MAX_ATTEMPTS_TO_FIND_PATH,
            collision_distance=0.15
        )

        if path_to_above_position is None:
            self.trash_generator.remove_trash()
            return None

        # get list of the arms configs when they reach the "above position"
        above_pos_conf_per_arm = split_arms_conf(path_to_above_position[-1], n_arms)

        path_from_above_pos_to_actual_pos = self.arms_manager.birrt([self.arms[arm_idx] for arm_idx in arms_idx],
                                                                    arms_to_actual_goal_configs,
                                                                    start_configs=above_pos_conf_per_arm,
                                                                    max_attempts=MAX_ATTEMPTS_TO_FIND_PATH)
        if path_from_above_pos_to_actual_pos is None:
            self.trash_generator.remove_trash()
            return None

        # removes all trash
        self.trash_generator.remove_trash()

        return path_to_base + path_to_above_position + path_from_above_pos_to_actual_pos

    def compute_path_to_bin(self, arms_idx, bin_locations, start_configs):
        """"
        @param arms_idx: arm indices to find paths for
        @param bin_locations: list of bin locations, same order as in arms_idx
        @param start_configs: list of start configs of the arms, same order as in arms_idx

        @returns a list of paths to appropriate bins, same order as in arms_idx
        if no path found, return None.

        Assumes that there are at most 2 arms
        """
        # The path:
        # 1. Lowering/raising vertically (to avoid collisions and to be above the trash bin)
        # 2. Horizontal rotation to the direction of the bin
        # 3. Path to the exact bin location by BIRRT

        curr_arms = [self.arms[arm_idx] for arm_idx in arms_idx]

        # For each arm, find the better rotation angle (better = shorter path)
        rotate_degree_lst = [self.find_horizontal_rotation_degree(arm, bin_dst)
                             for arm, bin_dst in zip(curr_arms, bin_locations)]

        # Check if the arms will rotate to the same spot
        # if so, we will lower/raise the arms (vertically) to avoid collision
        if self.is_rotation_to_the_same_point(curr_arms, rotate_degree_lst):
            # --> len(curr_arms) = 2
            # Set different y offset to avoid collision
            vertical_offset_from_bin_lst = [0.75, 1.2]
        else:
            # The arms turn to different places, so they won't collide, we can use normal y offset
            vertical_offset_from_bin_lst = [0.95 for _ in range(len(curr_arms))]

        # Find end poses and above poses
        # End pose - the end poses of the arms (where they will release the trash)
        # Above pose - same as the pose when picking the trash, except that there is an additional offset in the y axis
        above_poses = []
        end_poses = []
        for arm, bin_loc, offset_from_bin in zip(curr_arms, bin_locations, vertical_offset_from_bin_lst):
            bin_loc = list(bin_loc)
            bin_loc[2] += offset_from_bin

            end_poses.append([bin_loc, p.getQuaternionFromEuler([0, np.pi / 2, -np.pi / 2])])

            # The above pose should only be higher vertically
            current_pose = arm.get_end_effector_pose().copy()
            current_pose[0][2] = bin_loc[2]
            above_poses.append(current_pose)

        # ------- Attempt to first pick up the trash higher only vertically -------
        path_to_above = self.arms_manager.birrt([self.arms[arm_idx] for arm_idx in arms_idx], above_poses, start_configs, max_attempts=MAX_ATTEMPTS_TO_FIND_PATH)
        if path_to_above is None:
            return None

        # get list of the arms configs when they reach the "above position"
        above_pos_conf_per_arm = split_arms_conf(path_to_above[-1], len(arms_idx))

        # ------- Block of code for horizontal rotation of the arms -------
        curr_confs_lst = above_pos_conf_per_arm  # list of the current configurations (for each arm)

        rotation_confs = []  # list of the configurations for the rotation (each conf includes all arms)
        iter = 40  # more iterations --> lower rotation speed
        for i in range(iter):
            next_confs_lst = []  # list of the next configurations (for each arm)
            for curr_conf, total_degree in zip(curr_confs_lst, rotate_degree_lst):
                next_conf = curr_conf.copy()
                next_conf[0] += total_degree / iter  # horizontal rotation
                next_confs_lst.append(next_conf)
            next_conf_all_arms_merged = [val for conf in next_confs_lst for val in conf]
            rotation_confs.append(next_conf_all_arms_merged)
            curr_confs_lst = next_confs_lst

        # get list of the arms configs when they reach the "rotated position"
        rotated_conf_per_arm = split_arms_conf(rotation_confs[-1], len(arms_idx))

        return path_to_above + rotation_confs

    def sync_arm_positions(self, real_arms_configs):
        """
        @param real_arms_configs: list of the arms in the real environment

        Sets every arm in the background environment to the configuration of their respective arm in the real environment
        """

        for back_arm, real_arm_config in zip(self.arms, real_arms_configs):
            back_arm.set_arm_joints(real_arm_config)

    def does_arm_reach(self, arm_idx, location, orientation=None, epsilon=0.03):
        """
        @param arm_idx: idx of the arm to check the reach of
        @param location: the location that should be checked if arm reaches
        @param orientation: the orientation of the arm at the desired location
        @param epsilon: threshold of comparison

        Returns True if arm can reach the desired location and orientation, False otherwise
        """

        if orientation is None:
            if self.arms[arm_idx].is_right_arm:
                orientation = p.getQuaternionFromEuler([0, np.pi / 2, -np.pi / 2])

            else:
                orientation = p.getQuaternionFromEuler([0, np.pi / 2, np.pi / 2])

        # Save original state of arm
        original_values = self.arms[arm_idx].get_arm_joint_values()

        # Run IK 10 times to reach better solutions
        for i in range(10):
            self.arms[arm_idx].set_arm_joints(self.arms[arm_idx].inverse_kinematics(location, orientation=orientation))
            effector_position, effector_orientation = self.arms[arm_idx].get_end_effector_pose()

        # Return arm to original state
        self.arms[arm_idx].set_arm_joints(original_values)
        logging.debug(f'does_arm_reach: Arm {arm_idx} - location[0] = {location[0]}, effector_position[0] = {effector_position[0]}')

        # Make sure the gripper's X and Z axes are correct - assume we can tolerate error in the Y axis by waiting
        if (
            abs(effector_position[0] - location[0]) > epsilon or
            abs(effector_position[2] - location[2]) > epsilon
        ):
            logging.info(f'Arm {arm_idx} (right = {self.arms[arm_idx].is_right_arm}) can\'t reach desired location {location}')
            logging.debug(f'effector_position[0] = {effector_position[0]}')
            logging.debug(f'effector_position[2] = {effector_position[2]}')
            return False

        return True

    def find_horizontal_rotation_degree(self, arm, bin_loc):
        """
        Returns the best direction (1 or -1) for the arm to move to the bin, only by moving joint 0
        """
        effector_loc = arm.get_end_effector_pose()[0]
        arm_base_loc = arm.pose[0]

        return utils.calc_angle(effector_loc[:2], arm_base_loc[:2], bin_loc[:2])

    @staticmethod
    def is_rotation_to_the_same_point(arms, rotate_degree_lst):
        """
        @param arms: the arms that will rotate
        @param rotate_degree_lst: the rotation degree for each arm
        @return: True if the arms rotates to the same point, False otherwise
        """
        rotation_to_same_point = False
        if len(arms) == 2 and rotate_degree_lst[0] * rotate_degree_lst[1] < 0:
            # 2 arm task, and arms rotates to opposite directions
            arms_y_loc = [arm.pose[0][1] for arm in arms]

            if arms[0].is_right_arm:
                if (
                    (arms_y_loc[1] > arms_y_loc[0] and rotate_degree_lst[1] > 0) or
                    (arms_y_loc[1] < arms_y_loc[0] and rotate_degree_lst[0] > 0)
                ):
                    rotation_to_same_point = True
            else:
                if (
                    (arms_y_loc[1] < arms_y_loc[0] and rotate_degree_lst[1] > 0) or
                    (arms_y_loc[1] > arms_y_loc[0] and rotate_degree_lst[0] > 0)
                ):
                    rotation_to_same_point = True

        return rotation_to_same_point


class ParallelEnv(object):
    def __init__(self, env_args: EnvironmentArgs, arms_idx: List[int], debug: bool):
        """
        :param env_args: arguments on how to initialize the environment
        :param arms_idx: List of indices of the arms this parallel environment handles
        :param debug: print debug messages flag
        """
        self.env_args = env_args
        self.input_queue = mp.Queue()
        self.output_queue = mp.Queue()
        self.arms_idx = arms_idx

        self.worker = mp.Process(target=worker_runner, daemon=True,
                                 args=(self.env_args, self.input_queue, self.output_queue, self.arms_idx, debug))
        self.worker.start()

    def dispatch(self, task_id, arms_idx: List[int], trash_conf: List[Dict], bin_locations, start_configs,
                 real_arms_configs=None):
        """
        Dispatch a path calculation request to the background process

        :param task_id: ID of the task being calculated
        :param arms_idx: arm indices to find paths for
        :param trash: list of trash configs, same order as in arms_idx
        :param bin_locations: list of bin locations, same order as in arms_idx
        :param start_configs: list of start configs of the arms, same order as in arms_idx
        :param real_arms_configs: list of arms in the real environment, same order as in the background environment
        """
        logging.debug('Master: Sending new task!')
        task = PendingTask(task_id, arms_idx, trash_conf, bin_locations, start_configs,
                           real_arms_configs=real_arms_configs)
        self.input_queue.put(task)

    def poll_dispatched_tasks(self) -> List[PendingTaskResult]:
        """
        Returns a list of completed path calculation requests
        """
        finished_tasks = []
        while not self.output_queue.empty():
            logging.debug('Master: Received new finished task!')
            task = self.output_queue.get()
            finished_tasks.append(task)

        return finished_tasks


class ParallelEnvWorker(object):
    def __init__(self, env_args: EnvironmentArgs, input_queue: mp.Queue, output_queue: mp.Queue, arms_idx: List[int]):
        """
        :param env_args: arguments on how to initialize the environment
        :param input_queue: Queue used to receive tasks
        :param output_queue: Queue used to send completed tasks
        :param arms_idx: List of indices of the arms this parallel environment handles
        """
        self.env = BackgroundEnv(env_args)
        self.input_queue = input_queue
        self.output_queue = output_queue
        self.arms_idx = arms_idx

    def run(self):
        """
        Main event loop of the worker, gets tasks from the input queue, calculate
        them, and then put results in the output queue
        """
        prev_end_configs = {}
        direction = 'right' if self.env.arms[self.arms_idx[0]].is_right_arm else 'left'
        logging.info(f'The pair is {direction}-sided')
        while True:
            if not self.input_queue.empty():
                # Got a task
                logging.info('Worker: Received new task!')
                task: PendingTask = self.input_queue.get()
                start_configs = []
                for i in range(len(task.arms_idx)):
                    if task.arms_idx[i] in prev_end_configs:
                        start_configs.append(prev_end_configs[task.arms_idx[i]])
                    else:
                        start_configs.append(task.start_configs[i])
                path = self.env.compute_motion_plan(task.arms_idx,
                                                    task.trash_conf,
                                                    task.bin_locations,
                                                    start_configs,
                                                    task.real_arms_configs)

                # Send task result back
                task_result = PendingTaskResult(task.task_id, path)
                if path is not None:
                    logging.debug(f'Found path')
                    path_to_bin = path[1]
                    path_to_bin_per_arm = split_arms_conf_lst(path_to_bin, len(task.arms_idx))
                    for i in range(len(task.arms_idx)):
                        prev_end_configs[task.arms_idx[i]] = path_to_bin_per_arm[i][-1]
                else:
                    logging.debug(f'Failed to find path')
                self.output_queue.put(task_result)
                logging.info('Worker: Sending finished task!')


def worker_runner(env_args: EnvironmentArgs, input_queue: mp.Queue, output_queue: mp.Queue, arms_idx: List[int],
                  debug: bool):
    """
    Utility function to run worker process

    :param env_args: arguments on how to initialize the environment
    :param input_queue: Queue used to receive tasks
    :param output_queue: Queue used to send completed tasks
    :param arms_idx: List of indices of the arms this parallel environment handles
    :param debug: print debug messages flag
    """
    import psutil
    process = psutil.Process()
    process.nice(psutil.ABOVE_NORMAL_PRIORITY_CLASS)
    prefix = '_'.join([str(x) for x in arms_idx])
    init_loggers(debug=debug, prefix=prefix)
    worker = ParallelEnvWorker(env_args, input_queue, output_queue, arms_idx)
    worker.run()
