import real_environment
import pybullet as p

if __name__ == '__main__':
    env_gui = real_environment.RealEnv(p.GUI)
    while True:
       env_gui.step()
