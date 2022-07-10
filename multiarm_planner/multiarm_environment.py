from itertools import chain
from time import sleep
from .obstacles import Obstacle
from .ur5_group import UR5Group

from .rrt.rrt_connect import birrt
from .rrt.pybullet_utils import configure_pybullet, draw_line, remove_all_markers

from .robot_ur5_env import MultiRobotUR5Env
from .mrdrrt.mrdrrt_planner import MRdRRTPlanner

def split_arms_conf_lst(arms_conf_lst, n_arm):
    """
    @param arms_conf_lst: list of configuration for all arms together (as obtained from birrt method in MultiarmEnvironment)
    @param n_arm: number of arms used to create this @param arms_conf_lst
    Returns list of configuration lists, for each arm separately
    """
    arm_conf_len = 6
    return [[conf[arm_conf_len * i:arm_conf_len * (i + 1)] for conf in arms_conf_lst] for i in range(n_arm)]

def split_arms_conf(arms_conf, n_arm):
    """
    @param arms_conf: 1 configuration of all arms together (as obtained from birrt method in MultiarmEnvironment)
    @param n_arm: number of arms used to create this @param arms_conf_lst
    Returns list of configuration, for each arm
    """
    arm_conf_len = 6
    return [arms_conf[arm_conf_len * i:arm_conf_len * (i + 1)] for i in range(n_arm)]

class MultiarmEnvironment:
    def __init__(self, p_env, ur5_arms, gui=True, visualize=False):
        """
        @param p_env: pybullet simulation physics client
        """
        print("[MultiarmEnv] Setting up multiarm environment")
        # set up simulator
        # configure_pybullet(rendering=gui, debug=False, yaw=0, pitch=0, dist=1.0, target=(0, 0, 0.3))

        self.p_env = p_env

        self.gui = gui
        self.visualize = visualize
        self.obstacles = None

        self.ur5_group = UR5Group(ur5_arms)

    def setup_run(self, ur5_poses, start_conf, target_eff_poses=None, obstacles=None, specific_ur5s=None):
        if self.gui:
            remove_all_markers()
            for pose, target in zip(target_eff_poses, self.targets):
                target.set_pose(pose)

        self.ur5_group.setup(ur5_poses, start_conf, specific_ur5s=specific_ur5s)

        if obstacles is not None:
            del self.obstacles
            self.obstacles = Obstacle.load_obstacles(obstacles)

    def birrt_from_task(self, task):
        print("[MultiarmEnv] Running BiRRT for task {0}".format(task.id))
        return self._birrt(start_configs=task.start_config,
                          goal_configs=task.goal_config,
                          ur5_poses=task.base_poses,
                          target_eff_poses=task.target_eff_poses,
                          obstacles=task.obstacles)

    def _birrt(self, ur5_arms, start_configs, goal_configs,
              ur5_poses, target_eff_poses=None, obstacles=None, resolutions=0.1, timeout=100000):
        self.setup_run(ur5_poses, start_configs, target_eff_poses, obstacles, specific_ur5s=ur5_arms)

        extend_fn = self.ur5_group.get_extend_fn(resolutions)
        collision_fn = self.ur5_group.get_collision_fn()
        start_conf = list(chain.from_iterable(start_configs))
        goal_conf = list(chain.from_iterable(goal_configs))
        
        path = birrt(start_conf=start_conf,
                     goal_conf=goal_conf,
                     distance=self.ur5_group.distance_fn,
                     sample=self.ur5_group.sample_fn,
                     extend=extend_fn,
                     collision=collision_fn,
                     iterations=10000,
                     smooth=5,
                     visualize=self.visualize,
                     fk=self.ur5_group.forward_kinematics,
                     group=True,
                     greedy=True,
                     timeout=timeout)

        if path is None:
            return None
        if self.gui:
            self.demo_path(ur5_poses, start_configs, path)
        return path

    def mrdrrt_from_task(self, task, cache_roadmaps=True, num_prm_nodes=50):
        print("[MultiarmEnv] Running MrDRRT for task {0}".format(task.id))
        return self._mrdrrt(start_configs=task.start_config,
                           goal_configs=task.goal_config,
                           ur5_poses=task.base_poses,
                           target_eff_poses=task.target_eff_poses,
                           obstacles=task.obstacles,
                           task_path=task.task_path,
                           cache_roadmaps=cache_roadmaps,
                           num_prm_nodes=num_prm_nodes)
                           
    def _mrdrrt(self, ur5_arms, start_configs, goal_configs,
              ur5_poses, target_eff_poses=None, obstacles=None,
              resolutions=0.1, task_path=None, cache_roadmaps=False, num_prm_nodes=50):
        start_configs = tuple(tuple(conf) for conf in start_configs)
        goal_configs = tuple(tuple(conf) for conf in goal_configs)
        self.setup_run(ur5_poses, start_configs, target_eff_poses, obstacles, specific_ur5s=ur5_arms)
        env = MultiRobotUR5Env(self.ur5_group, resolutions, self.obstacles)
        mrdrrt = MRdRRTPlanner(env, visualize=self.visualize)

        mrdrrt.get_implicit_graph(start_configs=start_configs, goal_configs=goal_configs, ur5_poses=ur5_poses,
                                    cache_roadmaps=cache_roadmaps, task_path=task_path, n_nodes=num_prm_nodes)

        self.ur5_group.setup(ur5_poses, start_configs)
        path = mrdrrt.find_path(start_configs, goal_configs)
        if path is None:
            return None

        if self.gui:
            self.demo_path(ur5_poses, start_configs, path)
        return path

    def get_configs_for_rrt(self, ur5_arms, goal_positions):
        start_configs = [ur5.get_arm_joint_values() for ur5 in ur5_arms]
        goal_configs = [ur5.inverse_kinematics(*goal_position) for ur5, goal_position in zip(ur5_arms, goal_positions)]
        ur5_poses = [ur5.get_pose() for ur5 in ur5_arms]

        return start_configs, goal_configs, ur5_poses

    def birrt(self, ur5_arms, goal_positions, start_configs=None, max_attempts=1):
        """"
        Returns a list of configurations for the arms to get to the goal_positions, or None if it couldn't find a path.
        @param max_attempts: number of attempts to find a path
        """
        current_configs, goal_configs, current_poses = self.get_configs_for_rrt(ur5_arms, goal_positions)
        start_configs = current_configs if start_configs is None else start_configs

        path = None
        attempt_count = 1
        while path is None and attempt_count <= max_attempts:
            path = self._birrt(ur5_arms, start_configs, goal_configs, current_poses)
            attempt_count += 1
        return path

    def mrdrrt(self, ur5_arms, goal_positions, start_configs=None):
        current_configs, goal_configs, current_poses = self.get_configs_for_rrt(ur5_arms, goal_positions)
        start_configs = current_configs if start_configs is None else start_configs

        return self._mrdrrt(ur5_arms, start_configs, goal_configs, current_poses)

    def demo_path(self, ur5_poses, start_configs, path_conf):
        self.ur5_group.setup(ur5_poses, start_configs)
        input("Press enter to play demo!")
        edges = []
        for i in range(len(path_conf)):
            if i != len(path_conf) - 1:
                for pose1, pose2 in zip(self.ur5_group.forward_kinematics(path_conf[i]),
                                        self.ur5_group.forward_kinematics(path_conf[i + 1])):
                    draw_line(pose1[0], pose2[0], rgb_color=[1, 0, 0], width=6)
                    edges.append((pose1[0], pose2[0]))
        for i, q in enumerate(path_conf):
            self.ur5_group.set_joint_positions(q)
            sleep(0.05)
        input("Done playing demo")


