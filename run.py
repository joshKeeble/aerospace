# import faggot
# import gps_height_reading
from moving import RRT, LineCollisionCheck, PathSmoothing
from vertical import can_avoid_vertically, dodge_vertically

field_length = 3000
height_limit = 150


def process_obstacles(obstacles, current_height):
    return_obstacle_list = list()
    moving = obstacles.get("moving_obstacles", None)
    for ob in moving:
        return_obstacle_list.append((
            ob.get("latitude"),
            ob.get("longitude"),
            ob.get("sphere_radius")  # ob["altitude_msl"] need to calculate radius at height
        ))
    stationary = obstacles.get("stationary_obstacles", None)
    for ob in stationary:
        if current_height < ob.get("cylinder_height"):
            return_obstacle_list.append((
                ob.get("latitude"),
                ob.get("longitude"),
                ob.get("cylinder_radius")
            ))

    return return_obstacle_list


def fly_to_next_waypoint(goal):
    pass


def main():
    while True:
        location = dict(
            lat=200.1111,
            lon=200.1111,
            height=400
        )  # get gps altitude

        goal = dict(
            lat=400,
            lon=400,
            height=400
        )  # get actual goal way-point

        obstacles = dict(
            moving_obstacles=[
                {
                    "altitude_msl": 189.56748784643966,
                    "latitude": 38.141826869853645,
                    "longitude": -76.43199876559223,
                    "sphere_radius": 150.0
                },
                {
                    "altitude_msl": 250.0,
                    "latitude": 38.14923628783763,
                    "longitude": -76.43238529543882,
                    "sphere_radius": 150.0
                }
            ],
            stationary_obstacles=[
                {
                    "cylinder_height": 750.0,
                    "cylinder_radius": 300.0,
                    "latitude": 38.140578,
                    "longitude": -76.428997
                },
                {
                    "cylinder_height": 400.0,
                    "cylinder_radius": 100.0,
                    "latitude": 38.149156,
                    "longitude": -76.430622
                }
            ]
        )  # get real data with function

        obstacle_list = process_obstacles(obstacles=obstacles, current_height=location.get("height"))
        if LineCollisionCheck(
                first=[location.get("lat"), location.get("lon")],
                second=[goal.get("lat"), goal.get("lon")],
                obstacleList=obstacle_list):
            fly_to_next_waypoint(goal=goal)
            print("No Line Collision, Flying Straight")
            continue

        if can_avoid_vertically(start=location.get("height"), obstacle=[100, 200], height_limit=height_limit):
            goal = dodge_vertically()
            fly_to_next_waypoint(goal=goal)
            print("Avoiding Vertically")
            continue

        path = RRT(
            start=[location.get("lat"), location.get("lon")],
            goal=[goal.get("lat"), goal.get("lon")],
            expandDis=int(field_length * 0.03),
            obstacleList=obstacle_list,
            randX=[100, 200],  # Add logic to determine area
            randY=[100, 200]  # Add logic to determine area
        ).Planning(animation=False)
        path = PathSmoothing(path=path, maxIter=30, obstacleList=obstacle_list)
        goal = path[2]  # flexible
        fly_to_next_waypoint(goal=goal)
        print("Avoiding Horizontally")


if __name__ == "__main__":
    main()
