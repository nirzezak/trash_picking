import argparse

import real_environment
import pybullet as p

from environment import EnvironmentArgs
from loggers import init_loggers


def get_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description='Waste Sorting With Multiple Arms Simulation')
    parser.add_argument('--back', action='store_true',
                        help='display the background environment in the GUI instead')
    parser.add_argument('--debug', action='store_true', help='print debug messages')
    parser.add_argument('--arms', default='configs/arms_locations.json', help='JSON file that contain arms positions')
    parser.add_argument('--bins', default='configs/trash_bins_locations.json',
                        help='JSON file that contain bins positions')

    return parser.parse_args()


if __name__ == '__main__':
    args = get_args()
    init_loggers(args.debug)
    connection_mode = p.DIRECT if args.back else p.GUI
    env_args = EnvironmentArgs(connection_mode, args.arms, args.bins)
    env_gui = real_environment.RealEnv(env_args)
    while True:
        env_gui.step()
