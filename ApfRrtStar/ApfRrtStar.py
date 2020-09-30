from math import sin, cos, atan2, hypot, log
import random
import time
from Obstacles import *
from matplotlib import pyplot as plt


class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None


class ApfRrtStar:
    def __init__(self, start, goal, obstacles_circ, obstacles_rect, step_size, apf_step, iter_max, radius, range_x, range_y, KP):
        self.start = Node(*start)
        self.goal = Node(*goal)
        self.obstacles_circ = obstacles_circ
        self.obstacles_rect = obstacles_rect
        self.step_size = step_size
        self.apf_step = apf_step
        self.iter_max = iter_max
        self.radius = radius
        self.range_x = range_x
        self.range_y = range_y
        self.KP = KP
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

    def construct_path(self):
        path = []
        node_curr = self.goal

        while node_curr.parent is not None:
            path.append((node_curr.x, node_curr.y))
            node_curr = node_curr.parent
        path.append((node_curr.x, node_curr.y))

        path = path[::-1]

        return path

    def calc_attractive_force(self, x, y):
        dx = self.goal.x - x
        dy = self.goal.y - y
        force_angle = atan2(dy, dx)
        force_value = self.KP * hypot(dx, dy)
        force_x = force_value * cos(force_angle)
        force_y = force_value * sin(force_angle)
        return force_x, force_y

    def get_closest_obstacle_distance(self, x, y):
        obstacles = self.obstacles_circ + self.obstacles_rect
        distances = [i.get_distance_and_angle_from_point(x, y)[0] for i in obstacles]
        mindist = min(distances)

        return mindist

    def get_apf_guided_node(self, node_rnd):
        prnd_x = node_rnd.x
        prnd_y = node_rnd.y

        k = 5
        d_min = 0.01

        for i in range(k):
            fa_x, fa_y = self.calc_attractive_force(prnd_x, prnd_y)
            force_angle = atan2(fa_y, fa_x)

            d = self.get_closest_obstacle_distance(prnd_x, prnd_y)
            if d < d_min:
                return Node(prnd_x, prnd_y)
            else:
                prnd_x = prnd_x + self.apf_step * cos(force_angle)
                prnd_y = prnd_y + self.apf_step * sin(force_angle)

        return Node(prnd_x, prnd_y)

    def run_planner(self):
        for i in range(self.iter_max):
            node_rnd = self.get_random_node()
            node_prnd = self.get_apf_guided_node(node_rnd)
            nearest_neighbor_index = self.get_nearest_neighbor_index(node_prnd)
            node_near = self.node_list[nearest_neighbor_index]
            node_new = self.steer(node_near, node_prnd)

            if self.check_collision(node_near, node_new):
                neighbor_indices = self.get_near_neighbor_indices(node_new)
                self.node_list.append(node_new)
                if neighbor_indices:
                    self.choose_parent(node_new, neighbor_indices)
                    self.rewire(node_new, neighbor_indices)

                '''dist, _ = self.get_distance_and_angle(node_new, self.goal)

                if dist <= self.step_size:
                    self.goal.parent = node_new
                    return self.construct_path()'''
        self.select_goal_best_parent()

        return None

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
