from ApfPlanner import *
import time


VARIANT = 5

STEP_SIZE = 0.4
KP, ETA, KB, KC, ESCAPE_SIGN = [4.08, 172.15, 0.77, 1.68, -0.47]
start = (1, 10)
goal = (30, 20)
range_x = (0, 35)
range_y = (0, 35)


obstacle_manager = ObstacleManager()
if VARIANT == 91:
    obstacle_manager.add_circle('rob1', 8, 7-2-2, 3, 5)
    obstacle_manager.add_circle('rob2', 22, 30, 4, 2)
elif VARIANT == 1:
    pass
elif VARIANT == 2:
    obstacle_manager.add_circle('rob1', 15, 18, 3, 4)
elif VARIANT == 3:
    #obstacle_manager.add_circle('rob1', 15.5, 15, 3, 4)
    obstacle_manager.add_circle('rob1', 15, 18, 3, 4)
elif VARIANT == 4:
    obstacle_manager.add_circle('rob1', 15.5, 15, 3, 4)
elif VARIANT == 5:
    obstacle_manager.add_rectangle("rob1", 15, 12, 8, 8, 45, 3)
    obstacle_manager.add_circle('rob2', 25, 20, 3, 4)
    obstacle_manager.add_circle('rob3', 19, 21, 1, 4)
elif VARIANT == 6:
    obstacle_manager.add_circle('rob1', 34, 20, 3, 4)
elif VARIANT == 7:
    start = (1, 10)
    goal = (30, 10)
    obstacle_manager.add_rectangle('rob1', 14, 10, 3, 7)
elif VARIANT == 92:
    obstacle_manager.add_circle('rob1', 10, 17, 2, 2+2)
    obstacle_manager.add_circle('rob2', 14, 17, 2, 2+2)
    obstacle_manager.add_circle('rob3', 18, 17, 2, 2+2)
    obstacle_manager.add_circle('rob4', 18, 13, 2, 2+2)
    obstacle_manager.add_circle('rob5', 18, 9, 2, 2+2)
    obstacle_manager.add_circle('rob6', 14, 9, 2, 2+2)
    obstacle_manager.add_circle('rob7', 10, 9, 2, 2+2)
    #obstacle_manager.add_rectangle('rob8', 12, 25, 4, 4)
elif VARIANT == 93:
    obstacle_manager.add_circle('rob1', 8, 7 - 2 - 2, 3, 5)
    obstacle_manager.add_circle('rob2', 22, 30, 4, 2)
    obstacle_manager.add_rectangle('rob3', 20, 20, 4, 8, 60, 2 + 1)
    obstacle_manager.add_rectangle('rob4', 10, 13, 9, 4, 90, 4.5)
elif VARIANT == 94:
    obstacle_manager.add_circle('rob1', 10, 17, 2, 2 + 2)
    obstacle_manager.add_circle('rob2', 14, 17, 2, 2 + 2)
    obstacle_manager.add_circle('rob3', 18, 17, 2, 2 + 2)
    obstacle_manager.add_circle('rob4', 18, 13, 2, 2 + 2)
    obstacle_manager.add_circle('rob5', 18, 9, 2, 2 + 2)
    obstacle_manager.add_circle('rob6', 14, 9, 2, 2 + 2)
    obstacle_manager.add_circle('rob7', 10, 9, 2, 2 + 2)
    obstacle_manager.add_rectangle('rob8', 5, 25, 10, 7)

obstacle_manager.add_rectangle('dole', 35/2, 0, 35, 0.1, 0.5)
obstacle_manager.add_rectangle('gore', 35/2, 35, 35, 0.1, 0.5)
obstacle_manager.add_rectangle('levo', 0, 35/2, 35, 0.1, 90, 0.5)
obstacle_manager.add_rectangle('desno', 35, 35/2, 35, 0.1, 90, 0.5)



figure, axes = plt.subplots()
plt.axis([range_x[0], range_x[1], range_y[0], range_y[1]])
plt.plot(*goal, 'rx')

obstacles_circ, obstacles_rect = obstacle_manager.get_obstacles()

planner = ApfPlanner(start, goal, obstacles_circ, obstacles_rect, STEP_SIZE, KP, ETA, KB, KC, ESCAPE_SIGN)

planner.plot_obstacles(obstacles_circ, obstacles_rect, axes)

startTime = time.time()

path, total_dis, total_cnt = planner.run_planner()

stopTime = time.time()
print('solved in: ' + str(stopTime - startTime) + ' seconds')


seg_path = planner.get_path_every_nth_point(path)
planner.plot_path(seg_path)


print('Distance = ' + str(total_dis))

plt.show()
