import os
import time
import pybullet as p
import pybullet_data
from multiarm_planner import UR5, multiarm_environment
from multiarm_planner.ur5_group import UR5Group
import numpy as np
from pybullet_utils import bullet_client as bc

p_gui = bc.BulletClient(connection_mode=p.GUI)
p_background = bc.BulletClient(connection_mode=p.DIRECT)
ps = [p_gui, p_background]


URDF_FILES_PATH = "models"

DISTANCE = 100


BINS_LOCATIONS = [[[1.5, 0.0, 0.1], [-1.5, 0.0, 0.1]], [[1.5 + DISTANCE, 0.0 + DISTANCE, 0.1 + DISTANCE], [-1.5 + DISTANCE, 0.0 + DISTANCE, 0.1 + DISTANCE]]]

UR5_LOCATIONS = [[([0, 0, 0.3], [0, 0, 0, 1])], [([0 + DISTANCE, 0 + DISTANCE, 0.3 + DISTANCE], [0, 0, 0, 1])]]


# Open GUI and set pybullet_data in the path
#p.connect(p.GUI)
# p.resetDebugVisualizerCamera(3, 90, -30, [0.0, -0.0, -0.0])

# 0 - gui 1 - background
plane_id = []
bins_ids = []
ur5_arms = []
trash_id = []
table_id = []

i = -1
for p_ in ps:
    i += 1

    p_.setTimeStep(1 / 240.)
    p_.setAdditionalSearchPath(pybullet_data.getDataPath())

    # Load plane contained in pybullet_data
    plane_id.append(p_.loadURDF(os.path.join(pybullet_data.getDataPath(), "plane.urdf")))

    bins_ids.append([p_.loadURDF(os.path.join(URDF_FILES_PATH, "bin.urdf"), bin_loc, flags=p_.URDF_USE_INERTIA_FROM_FILE,
                       useFixedBase=True) for bin_loc in BINS_LOCATIONS[i]])

    ur5_arms.append([UR5.UR5(ur5_loc) for ur5_loc in UR5_LOCATIONS[i]])

    p_.setGravity(0, 0, -9.8)
    trash_id.append(p_.loadURDF('cube_small.urdf', [-0.4 + DISTANCE*i, -0.4+ DISTANCE*i, 0.3+ DISTANCE*i], globalScaling=0.75))
    table_id.append(p_.loadURDF("table_square\\table_square.urdf", [-0.4+ DISTANCE*i, -0.4+ DISTANCE*i, -0.2+ DISTANCE*i], globalScaling=0.75))



ur5_arms[0][0].reset()
ur5_arms[1][0].reset()

ur5_group = []
multiarm_env = []

for i in range(len(ps)):
    ur5_group.append(UR5Group(ur5s=ur5_arms[i]))
    multiarm_env.append(multiarm_environment.MultiarmEnvironment(gui=False, visualize=False, ur5_group=ur5_group[i]))


# p_gui.changeDynamics(trash_id, -1, mass=0.5)


trash_pos, orientation = p_background.getBasePositionAndOrientation(trash_id[1])
trash_pos = list(trash_pos)
trash_pos[2] += 0.2
end_pos = [trash_pos, p_background.getQuaternionFromEuler([0, np.pi / 2, 0])]  # This orientation means to take the trash "from above"
effector_pose = None #[[-0.026179734617471695, 0.1522190272808075, 0.21971933543682098],  [0.00350586767308414, 0.7089446187019348, 0.007206596899777651, 0.7052186727523804]]

time.sleep(2)
path = None
path = multiarm_env[1].birrt2([ur5_arms[1][0].inverse_kinematics(*end_pos)], effector_pose)
# path = multiarm_env.mrdrrt2([ur5_arms[0].inverse_kinematics(*end_pos)], effector_pose)
# path = multiarm_env.birrt2([ur5_arms[0].inverse_kinematics(*end_pos)], [(trash_pos, p.getQuaternionFromEuler([0, np.pi / 2, 0]))])
# path = multiarm_env.birrt2([ur5.get_arm_joint_values() for ur5 in ur5_arms], ([0, 0, 0, 1], [0, 0, 0, 1]))
if path is None:
    print('No path')

else:
    print('Found Path')
    trash_pos[2] -= 0.12
    end_pos = [trash_pos, p_background.getQuaternionFromEuler([0, np.pi / 2, 0])]
    path3 = multiarm_env[1].birrt2([ur5_arms[1][0].inverse_kinematics(*end_pos)], effector_pose)

    end_pos = [ur5_arms[1][0].home_config, p_background.getQuaternionFromEuler([0, np.pi / 2, 0])]
    path2 = multiarm_env[1].birrt2([end_pos[0]], effector_pose)
    if path2 is None:
        print('No path')

    else:
        ur5_arms[0][0].reset()
        for i, q in enumerate(path):
            # multiarm_env.ur5_group.set_joint_positions(q)
            # time.sleep(0.2)
            ur5_arms[0][0].move_joints(q, speed=0.05)
            time.sleep(0.01)

        for i, q in enumerate(path3):
            # multiarm_env.ur5_group.set_joint_positions(q)
            # time.sleep(0.2)
            ur5_arms[0][0].move_joints(q, speed=0.05)
            time.sleep(0.01)

        ur5_arms[0][0].close_gripper()
        # ur5_arms[0].set_arm_joints(last_config)
        for i, q in enumerate(path2):
            # multiarm_env.ur5_group.set_joint_positions(q)
            # time.sleep(0.1)
            ur5_arms[0][0].move_joints(q, speed=0.05)
            time.sleep(0.01)

        ur5_arms[0][0].open_gripper()



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
    print("**********")
    print(p_gui.getNumBodies())
    print(p_background.getNumBodies())  # TODO - weird results?
    print("**********")

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