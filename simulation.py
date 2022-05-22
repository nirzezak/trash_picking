from environment import Environment
import pybullet as p


if __name__ == '__main__':
    env_gui = Environment(p.GUI)
    env_background = Environment(p.DIRECT)
    for _ in range(10000):
        env_gui.step()
        env_background.step()
