import time
import pybullet as p
import numpy as np
from environment import Environment

MAX_ATTEMPS_TO_FIND_PATH = 200

class BackgroundEnv(Environment):
    def __init__(self, connection_mode=p.DIRECT):
        """"
        :param connection_mode: pybullet simulation connection mode. e.g.: pybullet.GUI, pybullet.DIRECT
        """
        super().__init__(connection_mode, conveyor_speed=0, set_pybullet_utils_p=True)

    def compute_motion_plan(self, arms_idx, trash, bin_locations, start_configs):
        """"
        @param arms_idx: arm indices to find paths for
        @param trash: list of trash configs, same order as in arms_idx
        @param bin_locations: list of bin locations, same order as in arms_idx
        @param start_configs: list of start configs of the arms, same order as in arms_idx

        @returns:
        if path is found, returns paths to trash, paths to bin
        paths to trash - a list of paths to appropriate trash, same order as in arms_idx
        paths to bins - a list of paths to appropriate bins, same order as in arms_idx

        if no path found, return None

        TODO - Right now the function assumes that we run this with only one arm
        """
        paths_to_trash = self.compute_path_to_trash(arms_idx, trash, start_configs)
        if paths_to_trash is None:
            return None
        last_configs = [paths_to_trash[-1]]  # TODO - change this when we will work with multi arms
        paths_to_bins = self.compute_path_to_bin(arms_idx, bin_locations, last_configs)
        if paths_to_bins is None:
            return None
        return paths_to_trash, paths_to_bins

    def compute_path_to_trash(self, arms_idx, trash, start_configs):
        """"
        @param arms_idx: arm indices to find paths for
        @param trash: list of trash configs, same order as in arms_idx
        @param start_configs: list of start configs of the arms, same order as in arms_idx

        @returns a list of paths to appropriate trash, same order as in arms_idx
        if no path found, return None.
        TODO - Right now the function assumes that we run this with only one arm
        """
        arms_to_above_position_configs = {}
        arms_to_actual_goal_configs = {}

        for arm_idx, trash_conf in zip(arms_idx, trash):
            # TODO: Fix this entire block of code because it is super specific
            trash = self.trash_generator.summon_trash(trash_conf)
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

        path_to_above_position = self.arms_manager.birrt(arms_to_above_position_configs.keys(), arms_to_above_position_configs.values(),
                                       start_configs=start_configs, max_attempts=MAX_ATTEMPS_TO_FIND_PATH)
        if path_to_above_position is None:
            self.trash_generator.remove_trash()
            return None
        path_from_above_pos_to_actual_pos = self.arms_manager.birrt(arms_to_actual_goal_configs.keys(),
                                                                    arms_to_actual_goal_configs.values(),
                                                                    start_configs=[path_to_above_position[-1]],
                                                                    max_attempts=MAX_ATTEMPS_TO_FIND_PATH)
        if path_from_above_pos_to_actual_pos is None:
            self.trash_generator.remove_trash()
            return None

        # removes all trash
        self.trash_generator.remove_trash()

        return path_to_above_position + path_from_above_pos_to_actual_pos

    def compute_path_to_bin(self, arms_idx, bin_locations, start_configs):
        """"
        @param arms_idx: arm indices to find paths for
        @param bin_locations: list of bin locations, same order as in arms_idx
        @param start_configs: list of start configs of the arms, same order as in arms_idx

        @returns a list of paths to appropriate bins, same order as in arms_idx
        if no path found, return None.
        TODO - Right now the function assumes that we run this with only one arm
        """
        end_poses = []
        for bin_loc in bin_locations:
            bin_loc = list(bin_loc)
            bin_loc[2] += 0.85
            end_poses.append([bin_loc, p.getQuaternionFromEuler([0, np.pi / 2, np.pi / 2])])
        return self.arms_manager.birrt([self.arms[arm_idx] for arm_idx in arms_idx], end_poses, start_configs,
                                       max_attempts=MAX_ATTEMPS_TO_FIND_PATH)