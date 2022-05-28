import time
import pybullet as p
import numpy as np
from environment import Environment


class BackgroundEnv(Environment):
    def __init__(self, connection_mode=p.DIRECT):
        """"
        :param connection_mode: pybullet simulation connection mode. e.g.: pybullet.GUI, pybullet.DIRECT
        """
        super().__init__(connection_mode, conveyor_speed=0)
        # TODO - change the p global parameter in pybullet_utils ? we can't do this because real simulation also uses pybullet_utils...

    def compute_motion_plan(self, arm_index_to_trash: dict, start_configs=None):
        """"
        @param arm_index_to_trash: dictionary that maps from arm index (from self.arms)
        to a trash: (TrashConfig, trash location)

        @returns a dictionary that maps from arms indices to paths, each arm gets the pass that will lead it to
        its trash target

        :param start_configs: list of start configs of the arms (same order as in arm_index_to_trash) TODO- this is ugly
        """
        # TODO - trash to bin case

        # set the starting configs of the arms
        # TODO - add option to give the starting configs
        for ur5 in self.arms:
            if start_configs is None:
                ur5.reset()
            else:
                pass
                #TODO

        # compute paths # TODO
        arms_to_goal_configs = {}
        for arm_idx, trash_info in arm_index_to_trash.items():
            trash = self.trash_generator.summon_trash(trash_config=trash_info[0], forced_location=trash_info[1])
            trash_pos, orientation = self.p_simulation.getBasePositionAndOrientation(trash.get_id())
            trash_pos = list(trash_pos)
            trash_pos[2] += 0.2  # TODO - make this dynamic?
            end_pos = [trash_pos, p.getQuaternionFromEuler([0, np.pi / 2, 0])]  # This orientation is "from above", TODO- make this dynamic?
            arms_to_goal_configs[self.arms[arm_idx]] = end_pos
        time.sleep(2)
        path = self.arms_manager.birrt(arms_to_goal_configs.keys(), arms_to_goal_configs.values(), start_configs)
        time.sleep(2)
        # removes all trash
        self.trash_generator.remove_trash()

        return path
