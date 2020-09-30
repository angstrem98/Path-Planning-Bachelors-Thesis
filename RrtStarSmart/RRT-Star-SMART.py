from math import sin, cos, atan2, hypot, log
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


class RrtStar_SMART:
    def __init__(self, start, goal, obstacles_circ, obstacles_rect, step_size, iter_max, radius, range_x, range_y):
        self.start = Node(*start)
        self.goal = Node(*goal)
        self.obstacles_circ = obstacles_circ
        self.obstacles_rect = obstacles_rect
        self.step_size = step_size
        self.iter_max = iter_max
        self.radius = radius
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

    def get_near_neighbor_indices(self, node):
        n = len(self.node_list)
        r = min(self.step_size, self.radius * sqrt(log(n) / n))
        distances = [hypot(node.x - i.x, node.y - i.y) for i in self.node_list]
        indices = [idx for idx in range(len(distances)) if distances[idx] <= r and
                   self.check_collision(self.node_list[idx], node)]

        return indices

    def get_node_cost(self, node):
        cost = 0
        node_curr = node

        while node_curr.parent is not None:
            cost += hypot(node_curr.parent.x - node_curr.x, node_curr.parent.y - node_curr.y)
            node_curr = node_curr.parent

        return cost

    def get_new_node_cost(self, node_start, node_end):
        dist, _ = self.get_distance_and_angle(node_start, node_end)
        cost = self.get_node_cost(node_start) + dist

        return cost

    def choose_parent(self, node_new, neighbor_indices):
        costs = [self.get_new_node_cost(self.node_list[i], node_new) for i in neighbor_indices]
        min_costs_index = costs.index(min(costs))
        new_parent_index = neighbor_indices[min_costs_index]
        node_new.parent = self.node_list[new_parent_index]

    def rewire(self, node_new, neighbor_indices):
        for i in neighbor_indices:
            node_neighbor = self.node_list[i]

            if self.get_node_cost(node_neighbor) > self.get_new_node_cost(node_new, node_neighbor):
                node_neighbor.parent = node_new

    def select_goal_best_parent(self):
        r = self.step_size * 2
        distances = [hypot(self.goal.x - i.x, self.goal.y - i.y) for i in self.node_list]
        indices = [idx for idx in range(len(distances)) if distances[idx] <= r and
                   self.check_collision(self.node_list[idx], self.goal)]

        if indices:
            costs = [distances[i] + self.get_node_cost(self.node_list[i]) for i in indices]
            index = indices[costs.index(min(costs))]
            self.goal.parent = self.node_list[index]
        else:
            nearest_ind = self.get_nearest_neighbor_index(self.goal)
            self.goal.parent = self.node_list[nearest_ind]

    def run_shortcutting_optimizer(self):
        node_a = self.goal
        node_b = node_a.parent.parent
        while node_a.parent is not self.start:
            while node_b is not self.start:
                if self.check_collision(node_a, node_b):
                    node_a.parent = node_b
                    node_b = node_a.parent.parent
                else:
                    node_b = node_b.parent

            node_a = node_a.parent
            node_b = node_a.parent.parent


    def construct_path(self):
        path = []
        node_curr = self.goal

        while node_curr.parent is not None:
            path.append((node_curr.x, node_curr.y))
            node_curr = node_curr.parent
        path.append((node_curr.x, node_curr.y))

        path = path[::-1]

        return path

    def intelligent_sampling(self, beacons):
        radius = 5

        for beacon in beacons:
            x_min = beacon.x - radius
            x_max = beacon.x + radius
            y_min = beacon.y - radius
            y_max = beacon.y + radius

            x_rnd = random.uniform(x_min, x_max)
            y_rnd = random.uniform(y_min, y_max)
            node_rnd = Node(x_rnd, y_rnd)
            nearest_neighbor_index = self.get_nearest_neighbor_index(node_rnd)
            node_near = self.node_list[nearest_neighbor_index]
            node_new = self.steer(node_near, node_rnd)

            if self.check_collision(node_near, node_new):
                neighbor_indices = self.get_near_neighbor_indices(node_new)
                self.node_list.append(node_new)
                if neighbor_indices:
                    self.choose_parent(node_new, neighbor_indices)
                    self.rewire(node_new, neighbor_indices)

    def get_beacons(self):
        beacon = self.goal
        beacons = []
        while beacon is not None:
            beacons.append(beacon)
            beacon = beacon.parent
        return beacons

    def run_planner(self):
        initial_path_found = False
        path_cost = float('inf')
        beacons = []
        cnt = 0
        for i in range(self.iter_max):
            node_rnd = self.get_random_node()
            nearest_neighbor_index = self.get_nearest_neighbor_index(node_rnd)
            node_near = self.node_list[nearest_neighbor_index]
            node_new = self.steer(node_near, node_rnd)

            if self.check_collision(node_near, node_new):
                neighbor_indices = self.get_near_neighbor_indices(node_new)
                self.node_list.append(node_new)
                if neighbor_indices:
                    self.choose_parent(node_new, neighbor_indices)
                    self.rewire(node_new, neighbor_indices)

                if not initial_path_found:
                    dist, _ = self.get_distance_and_angle(node_new, self.goal)
                    if dist <= self.step_size:
                        initial_path_found = True
                        '''self.select_goal_best_parent()
                        #self.run_shortcutting_optimizer()
                        path_cost = self.get_node_cost(self.goal)
                        beacons = self.get_beacons()
                        #return None'''
                else:
                    self.select_goal_best_parent()
                    path_cost = self.get_node_cost(self.goal)
                    beacons = self.get_beacons()
                    break


        for i in range(200):
            self.intelligent_sampling(beacons)
            self.run_shortcutting_optimizer()
            new_cost = self.get_node_cost(self.goal)
            if new_cost < path_cost:
                path_cost = new_cost
                beacons = self.get_beacons()
        self.select_goal_best_parent()
        self.run_shortcutting_optimizer()
        path_cost = self.get_node_cost(self.goal)

        return path_cost

    def get_all_nodes(self):
        return self.node_list


def plot_obstacles(obstacles_circ, obstacles_rect, axes):
    for i in obstacles_circ:
        c = plt.Circle((i.x, i.y), i.r)
        axes.add_artist(c)
        #plt.text(i.x, i.y, i.name, fontsize=12)

    for i in obstacles_rect:
        _, _, dl, _ = i.get_points()
        r = plt.Rectangle(dl, i.w, i.h, i.fi)
        axes.add_artist(r)
        #plt.text(i.x, i.y, i.name, fontsize=12)


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
    planner = RrtStar_SMART(start, goal, obstacles_circ, obstacles_rect, 4, 2000, 30, range_x, range_y)
    path_cost = planner.run_planner()
    stopTime = time.time()

    path = planner.construct_path()

    print('solved in: ' + str(stopTime - startTime) + ' seconds')
    print('cost: ' + str(path_cost))

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
