def add_element_wise(l1, l2):
    """
    :param l1, l2: lists
    :returns l1 + l2 element-wise
    """
    return [sum(x) for x in zip(l1, l2)]
