import time
import pybullet as p
import numpy as np
from environment import Environment


class BackgroundEnv(Environment):
    def __init__(self, connection_mode=p.DIRECT):
        """"
        :param connection_mode: pybullet simulation connection mode. e.g.: pybullet.GUI, pybullet.DIRECT
        """
        super().__init__(connection_mode, conveyor_speed=0, set_pybullet_utils_p=True)

    def compute_motion_plan(self, arms_idx, trash, bin_locations, start_configs):
        """"
        @param arms_idx: arm indices to find paths for
        @param trash: a tuple: (TrashConfig, trash location), same order as in arms_idx
        @param bin_locations: list of bin locations, same order as in arms_idx
        @param start_configs: list of start configs of the arms, same order as in arms_idx

        @returns: paths to trash, paths to bin
        paths to trash - a list of paths to appropriate trash, same order as in arms_idx
        paths to bins - a list of paths to appropriate bins, same order as in arms_idx

        TODO - Right now the function assumes that we run this with only one arm
        """
        paths_to_trash = self.compute_path_to_trash(arms_idx, trash, start_configs)
        last_configs = [paths_to_trash[-1]]  # TODO - change this when we will work with multi arms
        paths_to_bins = self.compute_path_to_bin(arms_idx, bin_locations, last_configs)

        return paths_to_trash, paths_to_bins

    def compute_path_to_trash(self, arms_idx, trash, start_configs):
        """"
        @param arms_idx: arm indices to find paths for
        @param trash: a tuple: (TrashConfig, trash location), same order as in arms_idx
        @param start_configs: list of start configs of the arms, same order as in arms_idx

        @returns a list of paths to appropriate trash, same order as in arms_idx
        TODO - Right now the function assumes that we run this with only one arm
        """
        arms_to_above_position_configs = {}
        arms_to_actual_goal_configs = {}

        for arm_idx, trash_info in zip(arms_idx, trash):
            # TODO: Fix this entire block of code because it is super specific
            trash = self.trash_generator.summon_trash(trash_config=trash_info[0], forced_location=trash_info[1])
            trash_pos, orientation = self.p_simulation.getBasePositionAndOrientation(trash.get_id())
            trash_pos = list(trash_pos)

            # First position above the trash (higher chance of not colliding)
            trash_pos[2] += 0.7
            trash_pos[1] += 0.025
            trash_pos[0] += 0.1225

            end_pos = [trash_pos, p.getQuaternionFromEuler([0, np.pi / 2, np.pi / 2])]  # This orientation is "from above", TODO- make this dynamic?
            arms_to_above_position_configs[self.arms[arm_idx]] = end_pos

            # Then hopefully find a path that only lowers the arm to pick up the trash
            next_pos = trash_pos.copy()
            next_pos[2] -= 0.5875
            end_pos2 = [next_pos, p.getQuaternionFromEuler([0, np.pi / 2, np.pi / 2])]
            arms_to_actual_goal_configs[self.arms[arm_idx]] = end_pos2

        path = self.arms_manager.birrt(arms_to_above_position_configs.keys(), arms_to_above_position_configs.values(), start_configs)
        path.extend(self.arms_manager.birrt(arms_to_actual_goal_configs.keys(), arms_to_actual_goal_configs.values(), [path[-1]]))

        # removes all trash
        self.trash_generator.remove_trash()

        return path

    def compute_path_to_bin(self, arms_idx, bin_locations, start_configs):
        """"
        @param arms_idx: arm indices to find paths for
        @param bin_locations: list of bin locations, same order as in arms_idx
        @param start_configs: list of start configs of the arms, same order as in arms_idx

        @returns a list of paths to appropriate bins, same order as in arms_idx
        TODO - Right now the function assumes that we run this with only one arm
        """
        end_poses = []
        for bin_loc in bin_locations:
            bin_loc = list(bin_loc)
            bin_loc[2] += 0.85
            end_poses.append([bin_loc, p.getQuaternionFromEuler([0, np.pi / 2, np.pi / 2])])
        return self.arms_manager.birrt([self.arms[arm_idx] for arm_idx in arms_idx], end_poses, start_configs)