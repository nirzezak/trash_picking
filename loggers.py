import logging
import sys


def init_loggers(debug: bool, prefix='main'):
    level = logging.DEBUG if debug else logging.INFO
    log_format = '%(levelname)s: %(message)s'
    logging.basicConfig(filename=f'logs/{prefix}_log.txt', filemode='w', level=level, format=log_format)
    root = logging.getLogger()

    handler = logging.StreamHandler(sys.stderr)
    handler.setLevel(level)
    formatter = logging.Formatter(log_format)
    handler.setFormatter(formatter)
    root.addHandler(handler)
