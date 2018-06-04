def can_avoid_vertically(start, obstacle, height_limit=100):
    """

    Args:
        start (int): start is the height of the drone
        obstacle (list): obstacle is the lowest and highest height
        height_limit (int): speed_height is the ratio of speed the drone
            is moving to the height it can feasibly travel (height/distance)

    Returns:
        bool

    Examples:
        can_avoid_vertically(start=400, obstacle=[300, 670], height_limit=60)
    """
    return True if (height_limit > start - obstacle[0]) or (height_limit > start + obstacle[1]) else False


def dodge_vertically():
    """"""
    pass
