#!/usr/bin/env python3

from drone_api import Drone_api

points = [[0, 0, 10],
          [-10, -10, 10], [10, -10, 10],
          [10, -5, 10], [-10, -5, 10],
          [-10, 0, 10], [10, 0, 10],
          [10, 5, 10], [-10, 5, 10],
          [-10, 10, 10], [10, 10, 10],
          [0, 0, 10]]

drone = Drone_api()
drone.start()
for i in range(len(points)):
    if drone.is_shutdown():
        break
    print(f'POINT {points[i]}')
    drone.set_local_pose(*points[i], 0)
    while not drone.point_is_reached() and not drone.is_shutdown():
        drone.sleep(0.2)
    drone.sleep(2)
