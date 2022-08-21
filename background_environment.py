import logging
import time
import pybullet as p
import numpy as np
from environment import Environment
from multiarm_planner.multiarm_environment import split_arms_conf

MAX_ATTEMPTS_TO_FIND_PATH = 3


class BackgroundEnv(Environment):
    def __init__(self, connection_mode, arms_path, trash_bins_path):
        """"
        :param connection_mode: pybullet simulation connection mode. e.g.: pybullet.GUI, pybullet.DIRECT
        """
        super().__init__(connection_mode, 0, arms_path, trash_bins_path, set_pybullet_utils_p=True)

    def compute_motion_plan(self, arms_idx, trash, bin_locations, start_configs, real_arms=None):
        """"
        @param arms_idx: arm indices to find paths for
        @param trash: list of trash configs, same order as in arms_idx
        @param bin_locations: list of bin locations, same order as in arms_idx
        @param start_configs: list of start configs of the arms, same order as in arms_idx
        @param real_arms: list of arms in the real environment, same order as in the background environment

        @returns:
        if path is found, returns paths to trash, paths to bin
        paths to trash - a list of paths to appropriate trash, same order as in arms_idx
        paths to bins - a list of paths to appropriate bins, same order as in arms_idx

        if no path found, return None
        """
        if real_arms is not None:
            self.sync_arm_positions(real_arms)

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

            # TODO: Having the trash here hurts the probability of finding a path while having no real merit
            #   since the entire path will take place without the trash being in this exact location.
            #   Should remove the summoning altogether and just calculate the gripping points.
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
            goal_positiosn=arms_to_above_position_configs,
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
        TODO - Right now the function assumes that we run this with only one arm
        """

        above_poses = []
        end_poses = []

        for arm_idx, bin_loc in zip(arms_idx, bin_locations):
            bin_loc = list(bin_loc)
            bin_loc[2] += 0.85

            if self.arms[arm_idx].is_right_arm:
                orientation = p.getQuaternionFromEuler([0, np.pi / 2, np.pi / 2])
                rotation_value = 0

            else:
                orientation = p.getQuaternionFromEuler([0, np.pi / 2, -np.pi / 2])
                rotation_value = np.pi

            end_poses.append([bin_loc, orientation])

            # The above pose should only be higher vertically
            current_pose = self.arms[arm_idx].get_end_effector_pose()
            current_pose[0][2] = bin_loc[2]
            above_poses.append(current_pose)

        # Attempt to first pick up the trash higher only vertically
        path_to_above = self.arms_manager.birrt([self.arms[arm_idx] for arm_idx in arms_idx], above_poses, start_configs, max_attempts=MAX_ATTEMPTS_TO_FIND_PATH)
        if path_to_above is None:
            return None

        # get list of the arms configs when they reach the "above position"
        above_pos_conf_per_arm = split_arms_conf(path_to_above[-1], len(arms_idx))

        # Block of code for rotation arms
        rotate_arm_1 = above_pos_conf_per_arm[0].copy()
        rotate_arm_2 = above_pos_conf_per_arm[1].copy() if len(arms_idx) == 2 else [0] # Not used if called only with one arm

        rotation_confs = []
        for i, j in zip(np.linspace(rotate_arm_1[0], rotation_value, num=100, endpoint=True), np.linspace(rotate_arm_2[0], rotation_value, num=100, endpoint=True)):
            rotation1 = rotate_arm_1.copy()
            rotation1[0] = i

            if len(arms_idx) == 2:
                rotation2 = rotate_arm_2.copy()
                rotation2[0] = j
                rotation_confs.append(rotation1 + rotation2)

            else:
                rotation_confs.append(rotation1)

        # get list of the arms configs when they reach the "rotated position"
        rotated_conf_per_arm = split_arms_conf(rotation_confs[-1], len(arms_idx))

        # TODO: Old code of moving to bin
        # path_to_bin = self.arms_manager.birrt([self.arms[arm_idx] for arm_idx in arms_idx], end_poses, start_configs=above_pos_conf_per_arm,
        #                                max_attempts=MAX_ATTEMPTS_TO_FIND_PATH, collision_distance=0.15)

        path_to_bin = self.arms_manager.birrt([self.arms[arm_idx] for arm_idx in arms_idx], end_poses,
                                              start_configs=rotated_conf_per_arm,
                                              max_attempts=MAX_ATTEMPTS_TO_FIND_PATH)

        if path_to_bin is None:
            return None

        return path_to_above + rotation_confs + path_to_bin

    def sync_arm_positions(self, real_arms):
        """
        @param real_arms: list of the arms in the real environment

        Sets every arm in the background environment to the configuration of their respective arm in the real environment
        """

        for back_arm, real_arm in zip(self.arms, real_arms):
            back_arm.set_arm_joints(real_arm.get_arm_joint_values())

    def does_arm_reach(self, arm_idx, location, orientation=None, epsilon=0.03):
        """
        @param arm_idx: idx of the arm to check the reach of
        @param location: the location that should be checked if arm reaches
        @param orientation: the orientation of the arm at the desired location
        @param epsilon: threshold of comparison

        Returns True if arm can reach the desired location and orientation, False otherwise
        """

        if self.arms[arm_idx].is_right_arm:
            orientation = p.getQuaternionFromEuler([0, np.pi / 2, -np.pi / 2])

        else:
            orientation = p.getQuaternionFromEuler([0, np.pi / 2, np.pi / 2])

        # Save original state of arm
        original_values = self.arms[arm_idx].get_arm_joint_values()

        # Run IK 10 times to reach better solutions - TODO: not sure if this causes considerable slowdown, can lower from 10 iterations
        for i in range(10):
            self.arms[arm_idx].set_arm_joints(self.arms[arm_idx].inverse_kinematics(location, orientation=orientation))
            effector_position, effector_orientation = self.arms[arm_idx].get_end_effector_pose()

        # Return arm to original state
        self.arms[arm_idx].set_arm_joints(original_values)

        # Make sure the gripper's X and Z axes are correct - assume we can tolerate error in the Y axis by waiting
        if (
            abs(effector_position[0] - location[0]) > epsilon or
            abs(effector_position[2] - location[2]) > epsilon
        ):
            logging.info(f'Arm {arm_idx} can\'t reach desired location')
            return False

        return True
