from math import atan2, sin, cos, hypot, sqrt
from matplotlib import pyplot as plt
import random
from Obstacles import *


class ApfPlanner:
    def __init__(self, start, goal, obstacles_circ, obstacles_rect, step_size, KP, ETA, KB, KC, ESCAPE_SIGN):
        self.start = start
        self.goal = goal
        self.obstacles_circ = obstacles_circ
        self.obstacles_rect = obstacles_rect
        self.step_size = step_size
        self.KP = KP
        self.ETA = ETA
        self.KB = KB
        self.KC = KC
        self.ESCAPE_SIGN = ESCAPE_SIGN
        self.KD = 100
        self.path = []
        self.obs_dist_penalty = 0
        self.success = True

    def calc_attractive_force(self, x, y):
        dx = self.goal[0] - x
        dy = self.goal[1] - y
        force_angle = atan2(dy, dx)
        force_value = self.KP * hypot(dx, dy)
        force_x = force_value * cos(force_angle)
        force_y = force_value * sin(force_angle)
        return force_x, force_y

    def calc_repulsive_force(self, x, y):
        force_x = 0
        force_y = 0

        dist_to_goal = hypot(self.goal[0] - x, self.goal[1] - y)
        N = 1

        obstacles = self.obstacles_circ + self.obstacles_rect

        for obstacle in obstacles:
            rho, force_angle = obstacle.get_distance_and_angle_from_point(x, y)
            force_angle += 3.1415

            if rho > obstacle.rho_min:
                continue

            if rho == 0:
                rho = 0.001

            force_value = self.ETA * ((1 / rho) - (1 / obstacle.rho_min)) / (rho * rho)
            #force_value *= hypot(self.goal[0] - x, self.goal[1] - y) ** 4   #TODO : namesti stepen

            fv1 = force_value * (dist_to_goal ** N)
            fv2 = 0.5 * N * self.ETA * ((1 / rho) - (1 / obstacle.rho_min)) * (dist_to_goal ** (N - 1))

            #force_value = fv1 - fv2

            force_x += force_value * cos(force_angle)
            force_y += force_value * sin(force_angle)

        return force_x, force_y

    def is_point_occupied(self, x, y):
        for i in self.obstacles_circ:
            d = hypot(i.x - x, i.y - y)
            if d < i.r:
                return True

        for i in self.obstacles_rect:
            d = hypot(i.x - x, i.y - y)
            p = i.get_closest_point(x, y)
            if d < hypot(p[0] - i.x, p[1] - i.y):
                return True

        return False

    def run_planner(self):
        x_curr = self.start[0]
        y_curr = self.start[1]
        self.path.append((x_curr, y_curr))
        dist = hypot(self.goal[0] - x_curr, self.goal[1] - y_curr)

        cnt = 0

        while dist >= 1:
            '''plt.plot(x_curr, y_curr, '.r')
            if cnt % 2 ==0:
                plt.pause(0.01)'''
            fa_x, fa_y = self.calc_attractive_force(x_curr, y_curr)
            fr_x, fr_y = self.calc_repulsive_force(x_curr, y_curr)

            force_x = fa_x + fr_x
            force_y = fa_y + fr_y


            force_angle = atan2(force_y, force_x)

            next_x = self.step_size * cos(force_angle)
            next_y = self.step_size * sin(force_angle)

            if not self.is_point_occupied(x_curr + next_x, y_curr + next_y):
                x_curr += next_x
                y_curr += next_y

            self.path.append((x_curr, y_curr))
            dist = hypot(self.goal[0] - x_curr, self.goal[1] - y_curr)

            cnt += 1

            if cnt > 1400:
                self.success = False
                break
                #return None, 1400, 1400

        self.path.append((self.goal[0], self.goal[1]))

        seg_path = self.get_path_every_nth_point(self.path)
        total_dist = 0
        for i in range(len(seg_path) - 1):
            total_dist += hypot(seg_path[i+1][0] - seg_path[i][0], seg_path[i+1][1] - seg_path[i][1])

        self.path.append(self.goal)
        obstacles = self.obstacles_circ + self.obstacles_rect
        for point in seg_path:
            distances = [(i.get_distance_and_angle_from_point(*point)[0], i.rho_min) for i in obstacles]
            distances = [i[0] for i in distances]
            if distances:
                m = min(distances)
                self.obs_dist_penalty += 1 / m




        return self.path, total_dist, cnt

    @staticmethod
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

    @staticmethod
    def plot_path(path):
        plt.plot(*zip(*path), 'bo-', markersize=2)

    def get_path_every_nth_point(self, path):
        p = [point for idx, point in enumerate(path) if idx % 4 == 0]
        if self.success:
            p.append((self.goal[0], self.goal[1]))

        return p

    def post_process_path(self, path):
        new_path = path[:]
        for n in range(200):
            length = len(new_path)
            idx_a = random.randint(0, length - 1)

            radius = 6.5

            idx_b = random.randint(idx_a, length - 1)

            if radius - hypot(new_path[idx_b][0] - new_path[idx_a][0], new_path[idx_b][1] - new_path[idx_a][1]) < 0:
                continue

            if idx_a == idx_b:
                continue

            a = new_path[idx_a]
            b = new_path[idx_b]

            intersects = False
            for obs in self.obstacles_circ:
                if is_line_segment_intersecting_circle(a, b, obs):
                    intersects = True
                    break
            for obs in self.obstacles_rect:
                if is_line_segment_intersecting_rectangle(a, b, obs):
                    intersects = True
                    break

            if intersects is False:
                for j in range(idx_a + 1, idx_b):
                    new_path[j] = 'a'
                new_path = [x for x in new_path if x != 'a']

        return new_path




