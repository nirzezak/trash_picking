import argparse

import real_environment
import pybullet as p

from environment import EnvironmentArgs
from loggers import init_loggers


def get_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description='Trash Picking Simulation')
    parser.add_argument('--back', action='store_true', help='Display the background environment in the GUI instead')
    parser.add_argument('--debug', action='store_true', help='Print debug messages')
    parser.add_argument('--arms', default='configs/arms_locations.json', help='JSON file that contain arms positions')
    parser.add_argument('--bins', default='configs/trash_bins_locations.json',
                        help='JSON file that contain bins positions')

    return parser.parse_args()


def run(debug, back, arms_path, bins_path, summon_component=None, task_manager_component=None):
    init_loggers(debug)
    connection_mode = p.DIRECT if back else p.GUI
    env_args = EnvironmentArgs(connection_mode, arms_path, bins_path)
    env_gui = real_environment.RealEnv(env_args, summon_component, task_manager_component)
    while True:
        env_gui.step()


if __name__ == '__main__':
    args = get_args()
    run(args.debug, args.back, args.arms, args.bins)
