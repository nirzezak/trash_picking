import argparse
import logging
import sys

import real_environment
import pybullet as p


def get_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description='Trash Picking Simulation')
    parser.add_argument('--back', action='store_true', help='Display the background environment in the GUI instead')
    parser.add_argument('--debug', action='store_true', help='Print debug messages')
    parser.add_argument('--arms', default='configs/arms_locations.json', help='JSON file that contain arms positions')
    parser.add_argument('--bins', default='configs/trash_bins_locations.json',
                        help='JSON file that contain bins positions')

    return parser.parse_args()


def init_loggers(debug: bool):
    level = logging.DEBUG if debug else logging.INFO
    log_format = '%(process)d:%(levelname)s: %(message)s'
    logging.basicConfig(filename='log.txt', filemode='w', level=level, format=log_format)
    root = logging.getLogger()

    handler = logging.StreamHandler(sys.stderr)
    handler.setLevel(level)
    formatter = logging.Formatter(log_format)
    handler.setFormatter(formatter)
    root.addHandler(handler)


if __name__ == '__main__':
    args = get_args()
    init_loggers(args.debug)
    connection_mode = p.DIRECT if args.back else p.GUI
    env_gui = real_environment.RealEnv(connection_mode, args.arms, args.bins)
    while True:
        env_gui.step()
