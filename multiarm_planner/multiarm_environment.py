from itertools import chain
from time import sleep

from .ur5_group import UR5Group

from .rrt.rrt import rrt
from .rrt.rrt_connect import birrt
from .rrt.pybullet_utils import configure_pybullet, draw_line

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
    def __init__(self, p_env, ur5_arms, visualize=False):
        """
        @param p_env: pybullet simulation physics client
        """
        print("[MultiarmEnv] Setting up multiarm environment")
        # set up simulator
        # configure_pybullet(rendering=gui, debug=False, yaw=0, pitch=0, dist=1.0, target=(0, 0, 0.3))

        self.p_env = p_env

        self.visualize = visualize
        self.obstacles = None

        self.ur5_group = UR5Group(ur5_arms)

    def setup_run(self, start_conf, specific_ur5s=None):
        self.ur5_group.setup(start_conf, specific_ur5s=specific_ur5s)

    def _birrt(self, ur5_arms, start_configs, goal_configs, resolutions=0.1, timeout=10, rrt_only=False, collision_distance=None):
        self.setup_run(start_configs, specific_ur5s=ur5_arms)

        extend_fn = self.ur5_group.get_extend_fn(resolutions)
        collision_fn = self.ur5_group.get_collision_fn(collision_distance=collision_distance)
        start_conf = list(chain.from_iterable(start_configs))
        goal_conf = list(chain.from_iterable(goal_configs))

        if rrt_only:
            goal_test = lambda x: x == goal_conf

            path, num_iterations, time = rrt(start=start_conf,
                                             goal_sample=goal_conf,
                                             distance=self.ur5_group.distance_fn,
                                             sample=self.ur5_group.sample_fn,
                                             extend=extend_fn,
                                             collision=collision_fn,
                                             goal_test=goal_test,
                                             iterations=10000,
                                             goal_probability=0.2,
                                             # This is important to tune to better match dRRT behaviour
                                             greedy=False,  # Also this
                                             visualize=self.visualize,
                                             fk=self.ur5_group.forward_kinematics,
                                             group=True,
                                             timeout=timeout)

        else:
            path, num_iterations, time = birrt(start_conf=start_conf,
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

        return path, num_iterations, time

    def _mrdrrt(
        self,
        ur5_arms,
        start_configs,
        goal_configs,
        ur5_poses,
        resolutions=0.1,
        task_path=None,
        cache_roadmaps=True,
        num_prm_nodes=50,
        goal_biasing=0.2,
        timeout=300
    ):
        start_configs = tuple(tuple(conf) for conf in start_configs)
        goal_configs = tuple(tuple(conf) for conf in goal_configs)
        self.setup_run(start_configs, specific_ur5s=ur5_arms)
        env = MultiRobotUR5Env(self.ur5_group, resolutions, self.obstacles)
        mrdrrt = MRdRRTPlanner(env, visualize=self.visualize)

        mrdrrt.get_implicit_graph(start_configs=start_configs, goal_configs=goal_configs, ur5_poses=ur5_poses,
                                    cache_roadmaps=cache_roadmaps, task_path=task_path, n_nodes=num_prm_nodes)

        self.ur5_group.setup(start_configs)
        path, num_iterations, time = mrdrrt.find_path(start_configs, goal_configs, goal_biasing=goal_biasing, timeout=timeout)

        return path, num_iterations, time

    def get_configs_for_rrt(self, ur5_arms, goal_positions=None, start_configs=None, goal_configs=None):
        """
        @param ur5_arms: relevant ur5 arms
        @param goal_positions: the goal positions of the ur5 arms' end effectors - mutually exclusive with goal_configs
        @param start_configs: the starting configurations of the ur5 arms in case they shouldn't be calculated
        @param goal_configs: the goal configurations of the ur5 arms in case they shouldn't be calculated

        Returns the starting configurations of the ur5 arms, the goal configurations inferred from goal positions and
            the current poses of the ur5 arms
        """
        start_configs = [ur5.get_arm_joint_values() for ur5 in ur5_arms] if start_configs is None else start_configs
        goal_configs = [
            ur5.inverse_kinematics(*goal_position)
            for ur5, goal_position in zip(ur5_arms, goal_positions)
        ] if goal_positions is not None else goal_configs

        ur5_poses = [ur5.get_pose() for ur5 in ur5_arms]

        return start_configs, goal_configs, ur5_poses

    def birrt(self, ur5_arms, goal_positions=None, start_configs=None, goal_configs=None, max_attempts=1, collision_distance=None):
        """
        @param ur5_arms: the ur5 arms to find a path for
        @param goal_positions: the end locations of the ur5 effectors to find a path to - mutually exclusive with goal_configs
        @param start_configs: the starting configurations of the arms - uses the current arm configurations if not provided
        @param goal_configs: the end configurations of the ur5s to find a path to - mutually exclusive with goal_positions
        @param max_attempts: number of attempts to find a path
        @param collision_distance: distance from tip joint to consider as collision

        Returns a list of configurations for the arms to get to the goal_positions or goal_configs, or None
            if it couldn't find a path, using BiRRT algorithm.
        """

        start_configs = [ur5.get_arm_joint_values() for ur5 in ur5_arms] if start_configs is None else start_configs

        # Setup run first because IK looks for a configuration that is close to the current one
        self.setup_run(start_configs, specific_ur5s=ur5_arms)

        start_configs, goal_configs, current_poses = self.get_configs_for_rrt(
            ur5_arms,
            goal_positions=goal_positions,
            start_configs=start_configs,
            goal_configs=goal_configs
        )

        path = None
        attempt_count = 1
        while path is None and attempt_count <= max_attempts:
            path = self._birrt(ur5_arms, start_configs, goal_configs, collision_distance=collision_distance)[0]
            attempt_count += 1

        return path

    def mrdrrt(self, ur5_arms, goal_positions, start_configs=None):
        current_configs, goal_configs, current_poses = self.get_configs_for_rrt(ur5_arms, goal_positions)
        start_configs = current_configs if start_configs is None else start_configs

        return self._mrdrrt(ur5_arms, start_configs, goal_configs, current_poses)[0]
