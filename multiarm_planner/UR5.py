import pybullet as p
import numpy as np
import quaternion
import random

from task import TaskState, Task
from .rrt.pybullet_utils import (
    get_self_link_pairs,
    violates_limits
)
from .rrt.pybullet_utils import (
    get_difference_fn,
    get_distance_fn,
    get_sample_fn,
    get_extend_fn,
    control_joints,
    set_joint_positions,
    get_joint_positions,
    get_link_pose,
    inverse_kinematics,
    forward_kinematics
)
from math import pi
from threading import Thread
from time import sleep
import enum
# from trash_generator import TaskState


class ArmState(enum.Enum):
    IDLE = 0
    MOVING_TO_TRASH = 1
    MOVING_TO_BIN = 2
    PICKING_TRASH = 3
    RELEASING_TRASH = 4


class HemisphereWorkspace:
    def __init__(self, radius, origin):
        self.radius = radius
        self.origin = np.array(origin)
        random.seed()

    def point_in_workspace(self,
                           i=None,
                           j=None,
                           k=None):
        if i is None or j is None or k is None:
            i = random.uniform(0, 1)
            j = random.uniform(0, 1)
            k = random.uniform(0, 1)
        j = j ** 0.5
        output = np.array([
            self.radius * j * np.cos(i * np.pi * 2) * np.cos(k * np.pi / 2),
            self.radius * j * np.sin(i * np.pi * 2) * np.cos(k * np.pi / 2),
            self.radius * j * np.sin(k * np.pi / 2),
        ])
        output = output + self.origin
        assert np.linalg.norm(np.array(output) - self.origin) < self.radius
        return output


NORMAL = 0
TOUCHED = 1


class Robotiq2F85:
    TICKS_TO_CHANGE_GRIP = 250

    def __init__(self, p_simulation, ur5, color, replace_textures=True):
        """
        @param p_simulation: pybullet simulation physics client
        """
        self.p_simulation = p_simulation
        self.ur5 = ur5
        pose = ur5.get_end_effector_pose()
        self.body_id = self.p_simulation.loadURDF(
            'assets/gripper/robotiq_2f_85.urdf',
            pose[0],
            pose[1])
        self.color = color
        self.replace_textures = replace_textures
        self.tool_joint_idx = 7
        self.tool_offset = [0, 0, 0]
        self.tool_constraint = self.p_simulation.createConstraint(
            ur5.body_id,
            self.tool_joint_idx,
            self.body_id,
            0,
            jointType=p.JOINT_FIXED,
            jointAxis=[0, 0, 0],
            parentFramePosition=[0, 0, 0],
            childFramePosition=self.tool_offset,
            childFrameOrientation=p.getQuaternionFromEuler([0, -np.pi / 2, 0])
        )

        self.setup()

    def setup(self):
        # Set friction coefficients for gripper fingers
        for i in range(self.p_simulation.getNumJoints(self.body_id)):
            self.p_simulation.changeDynamics(self.body_id,
                             i,
                             lateralFriction=1.0,
                             spinningFriction=1.0,
                             rollingFriction=0.0001,
                             frictionAnchor=True)
            # ,contactStiffness=0.0,contactDamping=0.0)
            if self.replace_textures:
                self.p_simulation.changeVisualShape(
                    self.body_id,
                    i,
                    textureUniqueId=-1,
                    rgbaColor=(0, 0, 0, 0.5))

        self.p_simulation.changeVisualShape(
            self.body_id,
            0,
            textureUniqueId=-1,
            rgbaColor=(
                self.color[0],
                self.color[1],
                self.color[2], 0.5))

        self._mode = NORMAL
        self.normal()
        self.joints = [self.p_simulation.getJointInfo(
            self.body_id, i) for i in range(self.p_simulation.getNumJoints(self.body_id))]
        self.joints = [
            joint_info[0]
            for joint_info in self.joints
            if joint_info[2] == p.JOINT_REVOLUTE]
        self.p_simulation.setJointMotorControlArray(
            self.body_id,
            self.joints,
            p.POSITION_CONTROL,
            targetPositions=[
                0.0] * len(self.joints),
            positionGains=[1.0]
            * len(self.joints))
        # self.open()
        self.constraints_thread = Thread(
            target=self.step_daemon_fn)
        self.constraints_thread.daemon = True
        self.constraints_thread.start()

    def open(self):
        self.p_simulation.setJointMotorControl2(
            self.body_id,
            1,
            p.VELOCITY_CONTROL,
            targetVelocity=-5,
            force=10000)
        self.step_simulation(400)

    def close(self):
        self.p_simulation.setJointMotorControl2(
            self.body_id,
            1,
            p.VELOCITY_CONTROL,
            targetVelocity=5,
            force=10000)
        self.step_simulation(400)

    def normal(self):
        if self._mode != NORMAL:
            self.p_simulation.changeVisualShape(
                self.body_id,
                0,
                textureUniqueId=-1,
                rgbaColor=(
                    self.color[0],
                    self.color[1],
                    self.color[2], 0.5))
            self._mode = NORMAL

    def touched(self):
        if self._mode != TOUCHED:
            self.p_simulation.changeVisualShape(
                self.body_id,
                0,
                textureUniqueId=-1,
                rgbaColor=(0.4, 1.0, 0.4, 0.8))
            self._mode = TOUCHED

    def step(self):
        gripper_joint_positions = self.p_simulation.getJointState(self.body_id, 1)[0]
        self.p_simulation.setJointMotorControlArray(
            self.body_id,
            [6, 3, 8, 5, 10],
            p.POSITION_CONTROL,
            [gripper_joint_positions,
                -gripper_joint_positions,
                -gripper_joint_positions,
                gripper_joint_positions,
                gripper_joint_positions],
            positionGains=np.ones(5))

    def step_simulation(self, num_steps):
        for i in range(int(num_steps)):
            self.p_simulation.stepSimulation()
            if self.body_id is not None:
                # Constraints
                gripper_joint_positions = np.array([self.p_simulation.getJointState(self.body_id, i)[
                                                        0] for i in range(self.p_simulation.getNumJoints(self.body_id))])
                self.p_simulation.setJointMotorControlArray(
                    self.body_id, [6, 3, 8, 5, 10], p.POSITION_CONTROL,
                    [
                        gripper_joint_positions[1], -gripper_joint_positions[1],
                        -gripper_joint_positions[1], gripper_joint_positions[1],
                        gripper_joint_positions[1]
                    ],
                    positionGains=np.ones(5)
                )
            sleep(1e-3)

    def step_daemon_fn(self):
        while True:
            self.step()
            sleep(0.01)

    def transform_orientation(self, orientation):
        A = np.quaternion(*orientation)
        B = np.quaternion(*p.getQuaternionFromEuler([0, np.pi / 2, 0]))
        C = B * A
        return quaternion.as_float_array(C)

    def set_pose(self, pose):
        self.p_simulation.resetBasePositionAndOrientation(
            self.body_id,
            pose[0],
            self.transform_orientation(pose[1]))

    def update_eef_pose(self):
        self.set_pose(self.ur5.get_end_effector_pose())


class Robotiq2F85Target(Robotiq2F85):
    def __init__(self, pose, color):
        self.color = color
        self.replace_textures = True
        self.body_id = self.p_simulation.loadURDF(
            'assets/gripper/robotiq_2f_85_no_colliders.urdf',
            pose[0],
            pose[1],
            useFixedBase=1)
        self.set_pose(pose)
        self.setup()


class InvalidArmState(Exception):
    pass


class UR5:
    joint_epsilon = 0.01
    joints_count = 6
    next_available_color = 0
    workspace_radius = 0.85
    colors = [
        [0.4, 0, 0],
        [0, 0, 0.4],
        [0, 0.4, 0.4],
        [0.4, 0, 0.4],
        [0.4, 0.4, 0],
        [0, 0, 0],
    ]
    LINK_COUNT = 10

    GROUPS = {
        'arm': ["shoulder_pan_joint",
                "shoulder_lift_joint",
                "elbow_joint",
                "wrist_1_joint",
                "wrist_2_joint",
                "wrist_3_joint"],
        'gripper': None
    }

    GROUP_INDEX = {
        'arm': [1, 2, 3, 4, 5, 6],
        'gripper': None
    }

    INDEX_NAME_MAP = {
        0: 'world_joint',
        1: 'shoulder_pan_joint',
        2: 'shoulder_lift_joint',
        3: 'elbow_joint',
        4: 'wrist_1_joint',
        5: 'wrist_2_joint',
        6: 'wrist_3_joint',
        7: 'ee_fixed_joint',
        8: 'wrist_3_link-tool0_fixed_joint',
        9: 'base_link-base_fixed_joint'
    }

    LOWER_LIMITS = [-2 * pi, -2 * pi, -pi, -2 * pi, -2 * pi, -2 * pi]
    UPPER_LIMITS = [2 * pi, 2 * pi, pi, 2 * pi, 2 * pi, 2 * pi]
    MAX_VELOCITY = [3.15, 3.15, 3.15, 3.2, 3.2, 3.2]
    MAX_FORCE = [150.0, 150.0, 150.0, 28.0, 28.0, 28.0]

    HOME = [0, 0, 0, 0, 0, 0]
    UP = [0, -1.5707, 0, -1.5707, 0, 0]
    RESET = [0, -1, 1, 0.5, 1, 0]
    EEF_LINK_INDEX = 7

    UR5_MOVE_SPEED = 0.05

    def __init__(self,
                 p_simulation,
                 pose,
                 home_config=None,
                 velocity=1.0,
                 enabled=True,
                 acceleration=2.0,
                 training=False):
        """
        @param p_simulation: pybullet simulation physics client
        """
        self.p_simulation = p_simulation
        self.velocity = velocity
        self.acceleration = acceleration
        self.pose = pose
        self.target_joint_values = None
        self.enabled = enabled
        self.workspace = HemisphereWorkspace(
            radius=UR5.workspace_radius,
            origin=self.pose[0])
        self.subtarget_joint_actions = False
        self.color = UR5.colors[UR5.next_available_color]
        UR5.next_available_color = (UR5.next_available_color + 1)\
            % len(UR5.colors)
        if training:
            self.body_id = self.p_simulation.loadURDF('assets/ur5/ur5_training.urdf',
                                      self.pose[0],
                                      self.pose[1],
                                      flags=p.URDF_USE_SELF_COLLISION)
            self.end_effector = None
            self.p_simulation.changeVisualShape(
                self.body_id,
                UR5.EEF_LINK_INDEX,
                textureUniqueId=-1,
                rgbaColor=(
                    self.color[0],
                    self.color[1],
                    self.color[2], 0.5))
        else:
            self.body_id = self.p_simulation.loadURDF('assets/ur5/ur5.urdf',
                                      self.pose[0],
                                      self.pose[1],
                                      flags=p.URDF_USE_SELF_COLLISION)
            self.end_effector = Robotiq2F85(p_simulation=self.p_simulation, ur5=self,
                                            color=self.color)
        # Get revolute joint indices of robot (skip fixed joints)
        robot_joint_info = [self.p_simulation.getJointInfo(self.body_id, i)
                            for i in range(self.p_simulation.getNumJoints(self.body_id))]
        self._robot_joint_indices = [
            x[0] for x in robot_joint_info if x[2] == p.JOINT_REVOLUTE]

        self._robot_joint_lower_limits = [
            x[8] for x in robot_joint_info if x[2] == p.JOINT_REVOLUTE]
        self._robot_joint_upper_limits = [
            x[9] for x in robot_joint_info if x[2] == p.JOINT_REVOLUTE]

        self.home_config = [-np.pi, -np.pi / 2,
                            np.pi / 2, -np.pi / 2,
                            -np.pi / 2, 0] if home_config is None\
            else home_config

        # for motion planning
        self.arm_difference_fn = get_difference_fn(
            self.body_id, self.GROUP_INDEX['arm'])
        self.arm_distance_fn = get_distance_fn(
            self.body_id, self.GROUP_INDEX['arm'])
        self.arm_sample_fn = get_sample_fn(
            self.body_id, self.GROUP_INDEX['arm'])
        self.arm_extend_fn = get_extend_fn(
            self.body_id, self.GROUP_INDEX['arm'])
        self.link_pairs = get_self_link_pairs(
            self.body_id,
            self.GROUP_INDEX['arm'])
        self.closest_points_to_others = []
        self.closest_points_to_self = []
        self.max_distance_from_others = 0.5
        self.curr_task = None  # the current executed task
        self.paths = None  # list of paths of the current executed task
        self.state = ArmState.IDLE
        self.current_tick = 0
        self.start_tick = 0
        self.first_config = False

    def update_closest_points(self, obstacles_ids=None):
        # if type(obstacles_ids) is list:
        #     others_id = obstacles_ids
        if obstacles_ids:
            # Add the plane to the list
            others_id = [0] + obstacles_ids
        else:
            others_id = [self.p_simulation.getBodyUniqueId(i)
                        for i in range(self.p_simulation.getNumBodies())
                        if self.p_simulation.getBodyUniqueId(i) != self.body_id]
                    
        self.closest_points_to_others = [
            sorted(list(self.p_simulation.getClosestPoints(
                bodyA=self.body_id, bodyB=other_id,
                distance=self.max_distance_from_others)),
                key=lambda contact_points: contact_points[8])
            if other_id != 0 else []
            for other_id in others_id]
        self.closest_points_to_self = [
            self.p_simulation.getClosestPoints(
                bodyA=self.body_id, bodyB=self.body_id,
                distance=0,
                linkIndexA=link1, linkIndexB=link2)
            for link1, link2 in self.link_pairs]

    def set_config_and_check_collision(self, q):
        self.set_arm_joints(q)
        return self.check_collision()

    def check_collision(self, collision_distance=0.0, obstacles_ids=None):
        self.update_closest_points(obstacles_ids=obstacles_ids)
        # Collisions with others
        for i, closest_points_to_other in enumerate(
                self.closest_points_to_others):
            if i == 0:
                # Treat plane specially
                for point in self.p_simulation.getClosestPoints(
                        bodyA=self.body_id, bodyB=0,
                        distance=0.0):
                    if point[8] < collision_distance:
                        self.prev_collided_with = point
                    return True
            else:
                # Check whether closest point's distance is
                # less than collision distance
                for point in closest_points_to_other:
                    if point[8] < collision_distance:
                        self.prev_collided_with = point
                        return True
        # Self Collision
        for closest_points_to_self_link in self.closest_points_to_self:
            for point in closest_points_to_self_link:
                if len(point) > 0:
                    self.prev_collided_with = point
                    return True
        self.prev_collided_with = None
        return False

    def calc_next_subtarget_joints(self):
        current_joints = self.get_arm_joint_values()
        if type(self.target_joint_values) != np.ndarray:
            self.target_joint_values = np.array(self.target_joint_values)
        subtarget_joint_values = self.target_joint_values - current_joints
        dt = 1.0 / 240.0
        dj = dt * self.velocity
        max_j = max(abs(subtarget_joint_values))
        if max_j < dj:
            return subtarget_joint_values
        subtarget_joint_values = subtarget_joint_values * dj / max_j
        return subtarget_joint_values + current_joints

    def disable(self, idx=0):
        self.enabled = False
        # self.set_pose([
        #     [idx, 20, 0],
        #     [0.0, 0.0, 0.0, 1.0]])
        # self.reset()
        # self.step()

    def enable(self):
        self.enabled = True

    def step(self):
        if self.end_effector is not None:
            self.end_effector.step()
        if self.subtarget_joint_actions:
            control_joints(
                self.body_id,
                self.GROUP_INDEX['arm'],
                self.calc_next_subtarget_joints(),
                velocity=self.velocity,
                acceleration=self.acceleration)

    def start_task(self, task: Task, print_ticks_stat=False):
        print('Arm starting task!')
        self.curr_task = task
        self.paths = [self.curr_task.path_to_trash, self.curr_task.path_to_bin]
        self.state = ArmState.MOVING_TO_TRASH
        self.first_config = True
        # For ticks statistics
        if print_ticks_stat:
            self.ticks_for_curr_conf_move = 0
            self.ticks_stat = {
                f'{ArmState.MOVING_TO_TRASH.name} #conf': len(self.paths[0]),
                f'{ArmState.MOVING_TO_BIN.name} #conf': len(self.paths[1]),
                f'{ArmState.MOVING_TO_TRASH.name} #ticks per conf': [],
                f'{ArmState.MOVING_TO_BIN.name} #ticks per conf': []
            }
            self.print_ticks_stat = print_ticks_stat

    def end_task(self):
        self.curr_task.state = TaskState.DONE

        # reset current task fields
        self.curr_task = None
        self.paths = None

        # For ticks statistics
        if self.print_ticks_stat:
            print(f'Total ticks: {self.current_tick}')
            for k, v in self.ticks_stat.items():
                print(f'{k}: {v}')

    def ur5_step(self):
        if self.state == ArmState.IDLE:
            self.current_tick = 0
            return

        path_completed = False
        current_joint_state = [self.p_simulation.getJointState(self.body_id, i)[0] for i in self._robot_joint_indices]
        self.current_tick += 1

        if self.state == ArmState.MOVING_TO_TRASH or self.state == ArmState.MOVING_TO_BIN:
            if len(self.paths) == 0:
                raise InvalidArmState(f'No path found for state: {self.state}')

            current_path = self.paths[0]

            if self.print_ticks_stat:
                self.ticks_for_curr_conf_move += 1

            if self.first_config or all([np.abs(current_joint_state[i] - current_path[0][i]) < 1e-2 for i in range(len(self._robot_joint_indices))]):
                if not self.first_config:
                    # Reached target configuration
                    if self.print_ticks_stat:
                        self.ticks_stat[f'{self.state.name} #ticks per conf'].append(self.ticks_for_curr_conf_move)
                        self.ticks_for_curr_conf_move = 0
                    current_path.pop(0)

                self.first_config = False
                if len(current_path) > 0:
                    # Move to next configuration
                    next_target_config = current_path[0]
                    self.p_simulation.setJointMotorControlArray(
                        self.body_id, self._robot_joint_indices,
                        self.p_simulation.POSITION_CONTROL, next_target_config,
                        positionGains=type(self).UR5_MOVE_SPEED * np.ones(len(self._robot_joint_indices))
                    )

                else:
                    path_completed = True
                    self.paths.pop(0)

        if path_completed:
            self.start_tick = self.current_tick

            if self.state == ArmState.MOVING_TO_TRASH:
                # Finished moving to trash - now pick it up
                self.state = ArmState.PICKING_TRASH
                self.close_gripper()

            elif self.state == ArmState.MOVING_TO_BIN:
                # Finished moving to bin - drop trash in bin
                self.state = ArmState.RELEASING_TRASH
                self.open_gripper()

        if self.state == ArmState.PICKING_TRASH:
            # Check if gripper picked trash
            if self.current_tick - self.start_tick > type(self.end_effector).TICKS_TO_CHANGE_GRIP:
                self.state = ArmState.MOVING_TO_BIN

        if self.state == ArmState.RELEASING_TRASH:
            if self.current_tick - self.start_tick > type(self.end_effector).TICKS_TO_CHANGE_GRIP:
                self.state = ArmState.IDLE
                self.stop_gripper()
                self.end_task()

    def close_gripper(self):
        if self.end_effector is not None:
            self.p_simulation.setJointMotorControl2(
                self.end_effector.body_id,
                1,
                self.p_simulation.VELOCITY_CONTROL,
                targetVelocity=5,
                force=10000
            )

    def open_gripper(self):
        if self.end_effector is not None:
            self.p_simulation.setJointMotorControl2(
                self.end_effector.body_id,
                1,
                self.p_simulation.VELOCITY_CONTROL,
                targetVelocity=-5,
                force=10000
            )

    def stop_gripper(self):
        if self.end_effector is not None:
            self.p_simulation.setJointMotorControl2(
                self.end_effector.body_id,
                1,
                self.p_simulation.VELOCITY_CONTROL,
                targetVelocity=0,
                force=10000
            )

    def get_pose(self):
        return self.p_simulation.getBasePositionAndOrientation(self.body_id)

    def set_pose(self, pose):
        self.pose = pose
        self.p_simulation.resetBasePositionAndOrientation(
            self.body_id,
            self.pose[0],
            self.pose[1])
        self.workspace.origin = self.pose[0]
        if self.end_effector is not None:
            self.end_effector.update_eef_pose()

    def global_to_ur5_frame(self, position, rotation=None):
        self_pos, self_rot = self.p_simulation.getBasePositionAndOrientation(self.body_id)
        invert_self_pos, invert_self_rot = self.p_simulation.invertTransform(
            self_pos, self_rot)
        ur5_frame_pos, ur5_frame_rot = self.p_simulation.multiplyTransforms(
            invert_self_pos, invert_self_rot,
            position, invert_self_rot if rotation is None else rotation
        )
        return ur5_frame_pos, ur5_frame_rot

    def on_touch_target(self):
        if self.end_effector is not None:
            self.end_effector.touched()

    def on_untouch_target(self):
        if self.end_effector is not None:
            self.end_effector.normal()

    def get_link_global_positions(self):
        linkstates = [self.p_simulation.getLinkState(
            self.body_id, link_id, computeForwardKinematics=True)
            for link_id in range(UR5.LINK_COUNT)]
        link_world_positions = [
            world_pos for
            world_pos, world_rot, _, _, _, _, in linkstates
        ]
        return link_world_positions

    def get_arm_joint_values(self):
        return np.array(get_joint_positions(
            self.body_id,
            self.GROUP_INDEX['arm']))

    def reset(self):
        self.set_arm_joints(self.home_config)

    def get_end_effector_pose(self, link=None):
        link = link if link is not None else self.EEF_LINK_INDEX
        return get_link_pose(self.body_id, link)

    def violates_limits(self):
        return violates_limits(
            self.body_id,
            self.GROUP_INDEX['arm'],
            self.get_arm_joint_values())

    def set_target_end_eff_pos(self, pos, orientation=None):
        self.set_arm_joints(
            self.inverse_kinematics(position=pos, orientation=orientation))

    def inverse_kinematics(self, position, orientation=None):
        return inverse_kinematics(
            self.body_id,
            self.EEF_LINK_INDEX,
            position,
            orientation)

    def forward_kinematics(self, joint_values):
        return forward_kinematics(
            self.body_id,
            UR5.GROUP_INDEX['arm'],
            joint_values,
            self.EEF_LINK_INDEX)

    def control_arm_joints(self, joint_values, velocity=None):
        velocity = self.velocity if velocity is None else velocity
        self.target_joint_values = joint_values
        if not self.subtarget_joint_actions:
            control_joints(
                self.body_id,
                self.GROUP_INDEX['arm'],
                self.target_joint_values,
                velocity=velocity,
                acceleration=self.acceleration)

    def control_arm_joints_delta(self, delta_joint_values, velocity=None):
        self.control_arm_joints(
            self.get_arm_joint_values() + delta_joint_values,
            velocity=velocity)

    def control_arm_joints_norm(self, normalized_joint_values, velocity=None):
        self.control_arm_joints(
            self.unnormalize_joint_values(normalized_joint_values),
            velocity=velocity)

    # NOTE: normalization is between -1 and 1
    @staticmethod
    def normalize_joint_values(joint_values):
        return (joint_values - np.array(UR5.LOWER_LIMITS)) /\
            (np.array(UR5.UPPER_LIMITS) - np.array(UR5.LOWER_LIMITS)) * 2 - 1

    @staticmethod
    def unnormalize_joint_values(normalized_joint_values):
        return (0.5 * normalized_joint_values + 0.5) *\
            (np.array(UR5.UPPER_LIMITS) - np.array(UR5.LOWER_LIMITS)) +\
            np.array(UR5.LOWER_LIMITS)

    def at_target_joints(self):
        actual_joint_state = self.get_arm_joint_values()
        return all(
            [np.abs(actual_joint_state[joint_id] -
                    self.target_joint_values[joint_id])
                < UR5.joint_epsilon
                for joint_id in range(len(actual_joint_state))])

    def set_arm_joints(self, joint_values):
        set_joint_positions(
            self.body_id,
            self.GROUP_INDEX['arm'],
            joint_values)
        self.control_arm_joints(joint_values=joint_values)
        if self.end_effector is not None:
            self.end_effector.update_eef_pose()
