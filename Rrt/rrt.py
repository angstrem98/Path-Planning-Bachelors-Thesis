from math import sin, cos, atan2, hypot
import random
import time
from Obstacles import *
from matplotlib import pyplot as plt

VARIANT = 92

range_x = (0, 35)
range_y = (0, 35)

start = (1, 10)
goal = (30, 20)

class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None


class Rrt:
    def __init__(self, start, goal, obstacles_circ, obstacles_rect, step_size, iter_max, range_x, range_y):
        self.start = Node(*start)
        self.goal = Node(*goal)
        self.obstacles_circ = obstacles_circ.copy()
        self.obstacles_rect = obstacles_rect.copy()
        self.step_size = step_size
        self.iter_max = iter_max
        self.range_x = range_x
        self.range_y = range_y
        self.node_list = [self.start]
        self.goal_sample_rate = 0.2

    def get_random_node(self):
        if random.random() > self.goal_sample_rate:
            x = random.uniform(self.range_x[0], self.range_x[1])
            y = random.uniform(self.range_y[0], self.range_y[1])
            return Node(x, y)

        return self.goal

    def get_nearest_neighbor_index(self, n):
        distances = [hypot(n.x - i.x, n.y - i.y) for i in self.node_list]

        return distances.index(min(distances))

    def get_distance_and_angle(self, node_start, node_end):
        dx = node_end.x - node_start.x
        dy = node_end.y - node_start.y

        return hypot(dx, dy), atan2(dy, dx)

    def steer(self, node_start, node_end):
        dist, angle = self.get_distance_and_angle(node_start, node_end)
        dist = min(dist, self.step_size)

        node_new = Node(node_start.x + dist * cos(angle), node_start.y + dist * sin(angle))
        node_new.parent = node_start

        return node_new

    def check_collision(self, node_start, node_end):
        """
        :param node_start:
        :param node_end:
        :return: True if there is NO collision
        """
        a = (node_start.x, node_start.y)
        b = (node_end.x, node_end.y)

        intersects = False
        for obs in self.obstacles_circ:
            if is_line_segment_intersecting_circle(a, b, obs):
                intersects = True
                break
        if not intersects:
            for obs in self.obstacles_rect:
                if is_line_segment_intersecting_rectangle(a, b, obs):
                    intersects = True
                    break

        return not intersects

    def get_nearest_neighbor_index(self, n):
        distances = [hypot(n.x - i.x, n.y - i.y) for i in self.node_list]

        return distances.index(min(distances))

    def construct_path(self):
        path = []
        goal_nearest_neighbor = self.node_list[self.get_nearest_neighbor_index(self.goal)]
        self.goal.parent = goal_nearest_neighbor

        node_curr = self.goal

        while node_curr.parent is not None:
            path.append((node_curr.x, node_curr.y))
            node_curr = node_curr.parent
        path.append((node_curr.x, node_curr.y))

        path = path[::-1]

        return path

    def run_planner(self):
        for i in range(self.iter_max):
            node_rnd = self.get_random_node()
            nearest_neighbor_index = self.get_nearest_neighbor_index(node_rnd)
            node_near = self.node_list[nearest_neighbor_index]
            node_new = self.steer(node_near, node_rnd)

            if self.check_collision(node_near, node_new):
                self.node_list.append(node_new)
                dist, _ = self.get_distance_and_angle(node_new, self.goal)

                '''if dist <= self.step_size:
                    self.goal.parent = node_new
                    return self.construct_path()'''

        return None

    def get_all_nodes(self):
        return self.node_list

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
    for i in obstacles_circ:
        c = plt.Circle((i.x, i.y), i.r)
        axes.add_artist(c)

    for i in obstacles_rect:
        _, _, dl, _ = i.get_points()
        r = plt.Rectangle(dl, i.w, i.h, i.fi)
        axes.add_artist(r)

    startTime = time.time()
    planner = Rrt(start, goal, obstacles_circ, obstacles_rect, 4, 2000, range_x, range_y)
    path = planner.run_planner()
    path = planner.construct_path()
    stopTime = time.time()
    print('solved in: ' + str(stopTime - startTime) + ' seconds')


    path_x = [i[0] for i in path]
    path_y = [i[1] for i in path]

    plt.plot(path_x, path_y, 'b-')


    nodes = planner.get_all_nodes()
    nodes_x = [i.x for i in nodes]
    nodes_y = [i.y for i in nodes]

    plt.plot(nodes_x, nodes_y, 'go', markersize=2)

    for node in nodes:
        if node.parent:
            x = [node.x, node.parent.x]
            y = [node.y, node.parent.y]
            plt.plot(x, y, 'g-', linewidth=0.5)

    # plot thick start node
    plt.plot(start[0], start[1], 'ko', markersize=6)

    plt.show()