_curr = 0


def tick():
    global _curr
    _curr += 1


def now():
    global _curr
    return _curr
