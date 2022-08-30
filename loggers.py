import logging
import sys


def init_loggers(debug: bool, prefix='main'):
    """
    Initialize logging for the process.
    If you need to use it for several processes, log each one to a different file
    (or use a suitable python package that will do that...)

    :param debug: whether to show logging messages from DEBUG or from INFO+
    :param prefix: prefix of logging file, that would be created in 'logs/{prefix}_log.txt'
    """
    level = logging.DEBUG if debug else logging.INFO
    log_format = '%(levelname)s: %(message)s'
    logging.basicConfig(filename=f'logs/{prefix}_log.txt', filemode='w', level=level, format=log_format)
    root = logging.getLogger()

    handler = logging.StreamHandler(sys.stderr)
    handler.setLevel(level)
    formatter = logging.Formatter(log_format)
    handler.setFormatter(formatter)
    root.addHandler(handler)
