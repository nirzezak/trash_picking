import background_environment
import real_environment
import pybullet as p
from trash_configs import TrashConfig

if __name__ == '__main__':
    # env_gui = real_environment.RealEnv(p.DIRECT)
    env_background = background_environment.BackgroundEnv(p.GUI)  # TODO this should be in RealEnv init
    env_background.compute_motion_plan({3: (TrashConfig.MUSTARD, [-0.2, 0, 1]), 2: (TrashConfig.MUSTARD, [0, 0.5, 1])})
    #for _ in range(10000):
    #    env_gui.step()
