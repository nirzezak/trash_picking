import time


def timefunc(f):
    """Simple timer function to identify slow spots in algorithm.
    Just import function and put decorator @timefunc on top of definition of any
    function that you want to time.
    """
    def f_timer(*args, **kwargs):
        start = time.time()
        result = f(*args, **kwargs)
        end = time.time()
        print(f.__name__, 'took {:.2f} seconds'.format(end - start))
        return result, (end - start)
    return f_timer
