import random
import time
from ApfRrtStar import *

VARIANT = 92

range_x = (0, 35)
range_y = (0, 35)

start = (1, 10)
goal = (30, 20)


if __name__ == '__main__':
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

    plot_obstacles(obstacles_circ, obstacles_rect, axes)

    startTime = time.time()
    planner = ApfRrtStar(start, goal, obstacles_circ, obstacles_rect, 4, 0.3, 1000, 30, range_x, range_y, 2)
    planner.run_planner()
    stopTime = time.time()
    print('solved in: ' + str(stopTime - startTime) + ' seconds')

    path = planner.construct_path()

    path_x = [i[0] for i in path]
    path_y = [i[1] for i in path]

    nodes = planner.get_all_nodes()
    nodes_x = [i.x for i in nodes]
    nodes_y = [i.y for i in nodes]

    plt.plot(nodes_x, nodes_y, 'go', markersize=3)

    for node in nodes:
        if node.parent:
            x = [node.x, node.parent.x]
            y = [node.y, node.parent.y]
            plt.plot(x, y, 'g-', linewidth=0.5)

    plt.plot(path_x, path_y, 'b-')
    # plot thick start node
    plt.plot(start[0], start[1], 'ko', markersize=6)

    plt.show()
