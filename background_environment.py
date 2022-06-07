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

    def compute_motion_plan(self, arm_index_to_trash: dict, start_configs=None):
        """"
        @param arm_index_to_trash: dictionary that maps from arm index (from self.arms)
        to a trash: (TrashConfig, trash location)

        @param start_configs: list of start configs of the arms (same order as in arm_index_to_trash TODO- this is ugly)

        @returns a dictionary that maps from arms indices to paths, each arm gets the pass that will lead it to
        its trash target
        """

        arms_to_above_position_configs = {}
        arms_to_actual_goal_configs = {}

        for arm_idx, trash_info in arm_index_to_trash.items():
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

    def path_to_bin(self, arms_idx, bin_locations, start_configs):
        """"
        @param arms_idx: arm indices to find paths for
        @param bin_locations: list of bin locations (same order as in arm indices)
        @param start_configs: list of start configs of the arms (same order as in arm_index_to_trash TODO- this is ugly)

        @returns a dictionary that maps from arms indices to paths, each arm gets the path that will lead it to
        the appropriate bin
        """

        bin_location = list(bin_locations)
        bin_location[2] += 0.85
        end_pos = [bin_location, p.getQuaternionFromEuler([0, np.pi / 2, np.pi / 2])]
        return self.arms_manager.birrt([self.arms[arm_idx] for arm_idx in arms_idx], [end_pos], start_configs)
