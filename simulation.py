from environment import Environment
import pybullet as p

if __name__ == '__main__':
    env = Environment(p.GUI)
    for _ in range(10000):
        env.step()
