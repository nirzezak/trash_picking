import logging

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
    forward_kinematics,
    JointState
)
from math import pi
from threading import Thread
from time import sleep
import enum


class ArmState(enum.Enum):
    IDLE = 0
    MOVING_TO_TRASH = 1
    MOVING_TO_BIN = 2
    WAITING_FOR_TRASH = 3
    PICKING_TRASH = 4
    RELEASING_TRASH = 5


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
    TICKS_TO_CHANGE_GRIP = 75

    def __init__(self, p_simulation, ur5, color, replace_textures=True):
        """
        @param p_simulation: pybullet simulation physics client
        """
        self.p_simulation = p_simulation
        self.ur5 = ur5

        # Some QoL improvements to how the gripper spawns initially
        pose_loc, pose_orient = ur5.get_end_effector_pose()
        pose_loc[1] -= 0.05
        pose_orient = p.getQuaternionFromEuler([0, -np.pi / 2, -np.pi /2])
        self.body_id = self.p_simulation.loadURDF(
            'assets/gripper/robotiq_2f_85.urdf',
            pose_loc,
            pose_orient
        )

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
                    rgbaColor=(self.color[0], self.color[1], self.color[2], 1)
                )

        self.p_simulation.changeVisualShape(
            self.body_id,
            0,
            rgbaColor=(
                self.color[0],
                self.color[1],
                self.color[2], 1
            )
        )

        self._mode = NORMAL
        self.normal()
        self.joints = [self.p_simulation.getJointInfo(
            self.body_id, i) for i in range(self.p_simulation.getNumJoints(self.body_id))]

        self.joints = [
            joint_info[0]
            for joint_info in self.joints
            if joint_info[2] == p.JOINT_REVOLUTE]

        self.gripper_lower_limit = self.p_simulation.getJointInfo(self.body_id, self.joints[0])[8]
        self.gripper_upper_limit = self.p_simulation.getJointInfo(self.body_id, self.joints[0])[9]

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


class InvalidArmState(Exception):
    pass


class UR5:
    joint_epsilon = 1e-2
    configuration_unchanged_epsilon = 1e-3
    max_unchanged_count = 30
    max_ticks_in_conf = 100

    next_available_color = 0
    colors = [
        [1, 0, 0],
        [0, 1, 0],
        [0, 0, 1],
        [1, 1, 1],
        [1, 0, 1],
        [0, 1, 1],
        [1, 0, 1],
        [1, 1, 0],
        [0, 0, 0],
    ]
    workspace_radius = 0.85

    GROUP_INDEX = {
        'arm': [1, 2, 3, 4, 5, 6]
    }

    LOWER_LIMITS = [-20 * pi, -20 * pi, -10 * pi, -20 * pi, -20 * pi, -20 * pi]
    UPPER_LIMITS = [20 * pi, 20 * pi, 10 * pi, 20 * pi, 20 * pi, 20 * pi]

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
        # # # Our stuff # # #
        self.p_simulation = p_simulation
        self._is_right_arm = None
        self.base_config = [0, 0, 0, 0, 0, 0]

        # Arm state machine
        self.curr_task = None  # the current executed task
        self.paths = None  # list of paths of the current executed task
        self.state = ArmState.IDLE
        self.current_tick = 0
        self.start_tick = 0
        self.first_config = False

        # Detect getting stuck
        self.prev_joint_state = None
        self.no_change_joint_state_count = 0
        self.ticks_in_conf = 0

        # Debug print: calculate how many ticks each conf took
        self.print_ticks_stat = False
        self.ticks_for_curr_conf_move = 0
        self.ticks_stat = None

        # # # Arm Pybullet init # # #
        self.velocity = velocity
        self.acceleration = acceleration
        self.pose = pose
        self.target_joint_values = None
        self.enabled = enabled
        self.workspace = HemisphereWorkspace(
            radius=UR5.workspace_radius,
            origin=self.pose[0])
        self.color = UR5.colors[UR5.next_available_color]
        UR5.next_available_color = (UR5.next_available_color + 1) % len(UR5.colors)

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

        # # # For motion planning # # #
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
        self.prev_collided_with = None

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

        # For ticks statistics
        if self.print_ticks_stat:
            logging.debug(f'Total ticks: {self.current_tick}')
            logging.debug(f'Estimated ticks: {self.curr_task.len_in_ticks}')
            for k, v in self.ticks_stat.items():
                logging.debug(f'{k}: {v}')

        # reset current task fields
        self.curr_task = None
        self.paths = None

    def ur5_step(self):
        self.current_tick += 1
        if self.state == ArmState.IDLE:
            self._state_machine_idle()
        elif self.state == ArmState.MOVING_TO_TRASH or self.state == ArmState.MOVING_TO_BIN:
            self._state_machine_moving()
        elif self.state == ArmState.WAITING_FOR_TRASH:
            self._state_machine_wait_for_trash()
        elif self.state == ArmState.PICKING_TRASH:
            self._state_machine_picking_trash()
        elif self.state == ArmState.RELEASING_TRASH:
            self._state_machine_releasing_trash()

    def _state_machine_idle(self):
        self.current_tick = 0

    def _state_machine_moving(self):
        if len(self.paths) == 0:
            raise InvalidArmState(f'No path found for state: {self.state}')

        current_path = self.paths[0]
        current_joint_state = [self.p_simulation.getJointState(self.body_id, i)[0] for i in self._robot_joint_indices]

        self._calc_prev_conf_diff(current_joint_state)

        if self.print_ticks_stat:
            self.ticks_for_curr_conf_move += 1
        self.ticks_in_conf += 1

        if self.first_config or \
                self._are_configurations_close(current_joint_state, current_path[0], self.joint_epsilon) or \
                self.no_change_joint_state_count == self.max_unchanged_count or \
                self.ticks_in_conf == self.max_ticks_in_conf:
            if not self.first_config:
                # Reached target configuration
                if self.print_ticks_stat:
                    self.ticks_stat[f'{self.state.name} #ticks per conf'].append(self.ticks_for_curr_conf_move)
                    self.ticks_for_curr_conf_move = 0
                self.ticks_in_conf = 0
                current_path.pop(0)
            else:
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
                self.paths.pop(0)
                self.start_tick = self.current_tick

                if self.state == ArmState.MOVING_TO_TRASH:
                    # Finished moving to trash - now wait for trash to reach optimal position
                    self.state = ArmState.WAITING_FOR_TRASH

                elif self.state == ArmState.MOVING_TO_BIN:
                    # Finished moving to bin - drop trash in bin
                    self.state = ArmState.RELEASING_TRASH
                    self.force_open_gripper()

    def _state_machine_wait_for_trash(self):
        distance = self.get_end_effector_pose()[0][1] - self.curr_task.trash.get_curr_position()[1]
        if distance <= 1e-3:
            # Trash reached optimal position for picking
            self.start_tick = self.current_tick
            self.curr_task.trash.reset_friction()
            self.close_gripper()
            self.state = ArmState.PICKING_TRASH

    def _state_machine_picking_trash(self):
        # Check if gripper picked trash
        self.no_change_joint_state_count = 0
        self.ticks_in_conf = 0
        if self.current_tick - self.start_tick > type(self.end_effector).TICKS_TO_CHANGE_GRIP:
            self.state = ArmState.MOVING_TO_BIN

    def _state_machine_releasing_trash(self):
        if self.current_tick - self.start_tick > type(self.end_effector).TICKS_TO_CHANGE_GRIP:
            self.no_change_joint_state_count = 0
            self.ticks_in_conf = 0
            self.prev_joint_state = None

            # Bring gripper back to resting position after forcing it to over-open
            self.open_gripper()

            self.state = ArmState.IDLE
            self.end_task()

    def _calc_prev_conf_diff(self, current_joint_state):
        if self.prev_joint_state is not None:
            if self._are_configurations_close(current_joint_state, self.prev_joint_state,
                                              self.configuration_unchanged_epsilon):
                self.no_change_joint_state_count += 1
            else:
                self.no_change_joint_state_count = 0

        self.prev_joint_state = current_joint_state

    def _are_configurations_close(self, conf1, conf2, epsilon):
        diff_in_states = [
            np.abs(conf1[i] - conf2[i]) for i in range(len(self._robot_joint_indices))
        ]
        closeness = [x < epsilon for x in diff_in_states]
        return all(closeness)

    def update_closest_points(self, obstacles_ids=None):
        # if type(obstacles_ids) is list:
        #     others_id = obstacles_ids
        if obstacles_ids:
            # Add the plane to the list
            others_id = [0] + obstacles_ids
        else:
            others_id = [
                self.p_simulation.getBodyUniqueId(i)
                for i in range(self.p_simulation.getNumBodies())
                if (
                    self.p_simulation.getBodyUniqueId(i) != self.body_id and
                    (self.end_effector is not None and self.p_simulation.getBodyUniqueId(i) != self.end_effector.body_id)
                )
            ]
                    
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
                    # Keep an extra safety distance from the tip joint to illustrate gripper behavior
                    if point[8] < 0 or (point[3] == 6 and point[8] < collision_distance):
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

    def disable(self, idx=0):
        self.enabled = False

    def enable(self):
        self.enabled = True

    def step(self):
        if self.end_effector is not None:
            self.end_effector.step()

    def close_gripper(self):
        if self.end_effector is not None:
            self.p_simulation.setJointMotorControl2(
                self.end_effector.body_id,
                self.end_effector.joints[0],
                self.p_simulation.POSITION_CONTROL,
                targetPosition=self.end_effector.gripper_upper_limit,
                force=5,
                maxVelocity=5
            )

    def open_gripper(self):
        if self.end_effector is not None:
            self.p_simulation.setJointMotorControl2(
                self.end_effector.body_id,
                self.end_effector.joints[0],
                self.p_simulation.POSITION_CONTROL,
                targetPosition=self.end_effector.gripper_lower_limit,
                force=10000,
                maxVelocity=-5
            )

    def force_open_gripper(self):
        if self.end_effector is not None:
            self.p_simulation.setJointMotorControl2(
                self.end_effector.body_id,
                self.end_effector.joints[0],
                self.p_simulation.VELOCITY_CONTROL,
                targetVelocity=-5,
                force=5
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

    def get_arm_joint_values(self):
        return np.array([JointState(*self.p_simulation.getJointState(self.body_id, joint)).jointPosition for joint in self.GROUP_INDEX['arm']])

    def get_end_effector_pose(self, link=None):
        link = link if link is not None else self.EEF_LINK_INDEX
        return get_link_pose(self.body_id, link, p_simulation=self.p_simulation)

    def inverse_kinematics(self, position, orientation=None):
        return inverse_kinematics(
            self.body_id,
            self.EEF_LINK_INDEX,
            position,
            orientation,
            max_num_iterations=50,
            p_simulation=self.p_simulation
        )

    def forward_kinematics(self, joint_values):
        return forward_kinematics(
            self.body_id,
            UR5.GROUP_INDEX['arm'],
            joint_values,
            self.EEF_LINK_INDEX)

    def control_arm_joints(self, joint_values, velocity=None):
        velocity = self.velocity if velocity is None else velocity
        self.target_joint_values = joint_values
        control_joints(
            self.body_id,
            self.GROUP_INDEX['arm'],
            self.target_joint_values,
            velocity=velocity,
            acceleration=self.acceleration)

    # NOTE: normalization is between -1 and 1
    @staticmethod
    def normalize_joint_values(joint_values):
        return (joint_values - np.array(UR5.LOWER_LIMITS)) /\
            (np.array(UR5.UPPER_LIMITS) - np.array(UR5.LOWER_LIMITS)) * 2 - 1

    def set_arm_joints(self, joint_values):
        set_joint_positions(
            self.body_id,
            self.GROUP_INDEX['arm'],
            joint_values)
        self.control_arm_joints(joint_values=joint_values)
        if self.end_effector is not None:
            self.end_effector.update_eef_pose()

    def set_base_config(self, base_config):
        self.set_arm_joints(base_config)
        self.base_config = base_config

    @property
    def is_right_arm(self):
        """
        Returns True if the UR5 arm is on the right side of the conveyor, False otherwise
        """
        if self._is_right_arm is not None:
            return self._is_right_arm

        # The conveyor is centered around 0 on the X-axis, therefore, arms with positive X are on the right side
        self._is_right_arm = self.get_pose()[0][0] > 0

        return self._is_right_arm

    def get_gripped_ids(self, grip_threshold=3e-4):
        """
        @param grip_threshold: the minimum distance from gripper to consider as "gripped"

        Returns set of IDs that the gripper is supposedly gripping
        """
        if self.end_effector is None:
            return None

        contact_points = self.p_simulation.getContactPoints(bodyA=self.end_effector.body_id)
        contact_ids = set()

        for contact_point in contact_points:
            contact_id = contact_point[2]
            contact_distance = contact_point[8]

            if contact_distance < grip_threshold:
                contact_ids.add(contact_id)

        return contact_ids
