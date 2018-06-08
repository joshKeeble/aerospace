from math import sqrt
import math
def can_avoid_vertically(start, obstacle, drone_gps, obstacle_gps, waypoint_next_gps, height_limit=100):
    """

    Args:
        start (int): start is the height of the drone
        obstacle (list): obstacle is the lowest and highest height
        height_limit (int): speed_height is the ratio of speed the drone
            is moving to the height it can feasibly travel (height/distance)

        drone/obstacle/waypoint_next _gps (list) are given in feet list x,y

    Returns:
        bool

    Examples:
        can_avoid_vertically(start=400, obstacle=[300, 670], height_limit=100)
    """
    #convert gps coords to feet, find out the offset of the drone from the obstacle then run check based off of new height

    #compare gps of waypoint and drone to find the slope l
    dist_here_to_wp = math.fabs((waypoint_next_gps[1] - drone_gps[1])) / math.fabs((waypoint_next_gps[0] - drone_gps[0]))
    eqA = drone_gps[1] - waypoint_next_gps[1]
    eqB = waypoint_next_gps[0] - drone_gps[0]
    eqC = (drone_gps[0] * waypoint_next_gps[1]) - (waypoint_next_gps[0] * drone_gps[1])
    d = (math.fabs((eqA*drone_gps[0])+(eqB*drone_gps[1])+eqC))/sqrt((eqA*eqA)+(eqB*eqB))



    radius = obstacle[1] - obstacle[0]
    z = sqrt((radius * radius)-(d*d))
    #midPoint = radius
    #initA = (start - radius) * (start - radius)
    #initC  = (radius*radius)
    #spliceRadius = sqrt(initC-initA)

    return True if (height_limit > start - z) or (height_limit > start + z) else False #changed obstacle[0] and [1]


def dodge_vertically():
    """"""
    pass
