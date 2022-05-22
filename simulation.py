from environment import Environment
import pybullet as p

BACKGROUND_ZERO_POINT = [4, 0, 0]

if __name__ == '__main__':
    env_gui = Environment(p.GUI)
    env_background = Environment(p.DIRECT, BACKGROUND_ZERO_POINT)
    for _ in range(10000):
        env_gui.step()
        env_background.step()
