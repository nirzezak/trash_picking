_curr = 0


def tick():
    """
    Make the world clock tick
    """
    global _curr
    _curr += 1


def now():
    """
    Get current time
    """
    global _curr
    return _curr
