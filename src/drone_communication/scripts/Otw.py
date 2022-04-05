with open( "/home/oze/mission_region0_route0_variant0.waypoint", mode="r", encoding="utf-8") as Trajectory_Mission_Planner:
    next(Trajectory_Mission_Planner)
    for Point in Trajectory_Mission_Planner:
        Point = Point.split()
        try:
            print(Point)
        except:
            print("Problem")