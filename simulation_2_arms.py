import os
import time
import pybullet as p
import pybullet_data
from multiarm_planner import UR5, ur5_group, multiarm_environment
import numpy as np

URDF_FILES_PATH = "models"

BINS_LOCATIONS = [[1.5, 0.0, 0.1], [-1.5, 0.0, 0.1]]

UR5_LOCATIONS = [([1, 0, 0.3], [0, 0, 0, 1]), ([-1, 0, 0.3], [0, 0, 0, 1])]


# Open GUI and set pybullet_data in the path
p.connect(p.GUI)
# p.resetDebugVisualizerCamera(3, 90, -30, [0.0, -0.0, -0.0])
p.setTimeStep(1 / 240.)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
print(pybullet_data.getDataPath())

# UR5_LOCATIONS = [([0, 0, 0.8], p.getQuaternionFromEuler([np.pi, 0, 0])), ([-1, 0, 1], [0, 0, 0, 1])]

# Load plane contained in pybullet_data
plane_id = p.loadURDF(os.path.join(pybullet_data.getDataPath(), "plane.urdf"))

bins_ids = [p.loadURDF(os.path.join(URDF_FILES_PATH, "bin.urdf"), bin_loc, flags=p.URDF_USE_INERTIA_FROM_FILE,
                   useFixedBase=True) for bin_loc in BINS_LOCATIONS]

ur5_arms = [UR5.UR5(ur5_loc) for ur5_loc in UR5_LOCATIONS]
for ur5 in ur5_arms:
    ur5.reset()

# trash_id = p.loadURDF(os.path.join(URDF_FILES_PATH, 'YcbMustardBottle', "model.urdf"), [0., 0., 0.],
#                       flags=p.URDF_USE_INERTIA_FROM_FILE)

p.setGravity(0, 0, -9.8)
trash_id = p.loadURDF('models/ur5/meshes/objects/cuboid.urdf', [-0.4, -0.4, 0.3], globalScaling=0.75)
table_id = p.loadURDF("table_square\\table_square.urdf", [-0.4, -0.4, -0.2], globalScaling=0.75)

# p.changeDynamics(trash_id, -1, mass=0.5)
ur5_group = ur5_group.UR5Group(ur5s=ur5_arms)

multiarm_env = multiarm_environment.MultiarmEnvironment(gui=False, visualize=False, ur5_group=ur5_group)

trash_pos, orientation = p.getBasePositionAndOrientation(trash_id)
trash_pos = list(trash_pos)
trash_pos[2] += 0.2
end_pos = [trash_pos, p.getQuaternionFromEuler([0, np.pi / 2, 0])]  # This orientation means to take the trash "from above"
effector_pose = None #[[-0.026179734617471695, 0.1522190272808075, 0.21971933543682098],  [0.00350586767308414, 0.7089446187019348, 0.007206596899777651, 0.7052186727523804]]

time.sleep(2)
path = None
path = multiarm_env.birrt2([ur5_arms[0].inverse_kinematics(*end_pos), ur5_arms[1].inverse_kinematics(*end_pos)], effector_pose)
# path = multiarm_env.mrdrrt2([ur5_arms[0].inverse_kinematics(*end_pos)], effector_pose)
# path = multiarm_env.birrt2([ur5_arms[0].inverse_kinematics(*end_pos)], [(trash_pos, p.getQuaternionFromEuler([0, np.pi / 2, 0]))])
# path = multiarm_env.birrt2([ur5.get_arm_joint_values() for ur5 in ur5_arms], ([0, 0, 0, 1], [0, 0, 0, 1]))
if path is None:
    print('No path')

else:
    print('Found Path')
    trash_pos[2] -= 0.12
    end_pos = [trash_pos, p.getQuaternionFromEuler([0, np.pi / 2, 0])]
    path3 = multiarm_env.birrt2([ur5_arms[0].inverse_kinematics(*end_pos), ur5_arms[1].inverse_kinematics(*end_pos)], effector_pose)

    end_pos = [ur5_arms[0].home_config, p.getQuaternionFromEuler([0, np.pi / 2, 0])]
    path2 = multiarm_env.birrt2([ur5_arms[0].home_config, ur5_arms[1].home_config], effector_pose)
    if path2 is None:
        print('No path')

    else:
        ur5_arms[0].reset()
        ur5_arms[1].reset()
        print("*********")
        print(path)
        for i, q in enumerate(path):
            multiarm_env.ur5_group.set_joint_positions(q)
            # time.sleep(0.2)
            # arm_step_conf_len = int(len(q)/2)
            # ur5_arms[0].move_joints(q[:arm_step_conf_len], speed=0.05)
            # ur5_arms[1].move_joints(q[arm_step_conf_len:], speed=0.05)
            time.sleep(0.01)

        for i, q in enumerate(path3):
            multiarm_env.ur5_group.set_joint_positions(q)
            # time.sleep(0.2)
            # arm_step_conf_len = int(len(q)/2)
            # ur5_arms[0].move_joints(q[:arm_step_conf_len], speed=0.05)
            # ur5_arms[1].move_joints(q[arm_step_conf_len:], speed=0.05)
            time.sleep(0.01)

        ur5_arms[0].close_gripper()
        ur5_arms[1].close_gripper()
        # ur5_arms[0].set_arm_joints(last_config)
        for i, q in enumerate(path2):
            multiarm_env.ur5_group.set_joint_positions(q)
            # time.sleep(0.1)
            # arm_step_conf_len = int(len(q)/2)
            # ur5_arms[0].move_joints(q[:arm_step_conf_len], speed=0.05)
            # ur5_arms[1].move_joints(q[arm_step_conf_len:], speed=0.05)
            time.sleep(0.01)

        ur5_arms[0].open_gripper()
        ur5_arms[1].open_gripper()



# ur5_arms[0].end_effector.close()
# p.setJointMotorControl2(
#             ur5_arms[0].end_effector.body_id,
#             1,
#             p.VELOCITY_CONTROL,
#             targetVelocity=0,
#             force=0)

# trash_pos, orientation = p.getBasePositionAndOrientation(trash_id)
# trash_pos = list(trash_pos)
# trash_pos[2] += 0.2
# trash_pos[0] -= 0.2
# end_pos = [trash_pos, p.getQuaternionFromEuler([0, np.pi / 2, 0])]
# ur5_arms[0].set_target_end_eff_pos(*end_pos)
# print('first')
# for i in range(10):
#     p.stepSimulation()
#     time.sleep(0.01)

# print('second')
# ur5_arms[0].end_effector.open()
# for i in range(100):
#     p.stepSimulation()
#     time.sleep(0.05)

# time.sleep(1)
# trash_pos[2] -= 0.1
# end_pos = [trash_pos, p.getQuaternionFromEuler([0, np.pi / 2, 0])]
# ur5_arms[0].set_target_end_eff_pos(*end_pos)
# for i in range(1):
#     p.stepSimulation()
#     time.sleep(0.01)
#
# time.sleep(1)
# print('third')
# ur5_arms[0].end_effector.close()
# for i in range(400):
#     p.stepSimulation()
#     time.sleep(0.05)

if __name__ == '__main__':
    # old = p.getBasePositionAndOrientation(ur5_arms[0].id)
    # old2 = p.getBasePositionAndOrientation(ur5_arms[1].id)
    # print(ur5_arms[0].get_arm_joint_values())
    # print(ur5_arms[0].get_end_effector_pose())

    while True:
        # p.stepSimulation()

        # trash_pos, orientation = p.getBasePositionAndOrientation(trash_id)
        # trash_pos = list(trash_pos)
        # trash_pos[2] += 0.2
        # end_pos = [trash_pos, p.getQuaternionFromEuler([0, np.pi / 2, 0])]
        # ur5_arms[0].set_target_end_eff_pos(*end_pos)


        # gripper_orientation = p.getQuaternionFromEuler([np.pi, 0, p.getEulerFromQuaternion(orientation)[2]])




        # print(ur5_arms[0].get_arm_joint_values())

        # print("\n\n\n")
        #
        # if old != new or old2 != new2:
        #     print('changed')

        time.sleep(1./240)