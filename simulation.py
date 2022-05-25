from environment import Environment

if __name__ == '__main__':
    env = Environment()
    for _ in range(10000):
        env.step()
