from background_environment import BackgroundEnv
from real_environment import RealEnv

if __name__ == '__main__':
    env_gui = RealEnv()
    env_background = BackgroundEnv()
    for _ in range(10000):
        env_gui.step()
