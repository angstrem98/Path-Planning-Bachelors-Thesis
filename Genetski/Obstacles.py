from math import hypot, sin, cos, sqrt, atan2

# **** Obstacle classes **** #


class Circle:
    def __init__(self, name, x, y, r, rho_min, lifetime):
        self.name = name
        self.x = x
        self.y = y
        self.r = r
        self.rho_min = rho_min
        self.lifetime = lifetime

    def get_distance_and_angle_from_point(self, x0, y0):
        return hypot(self.x - x0, self.y - y0) - self.r, atan2(self.y - y0, self.x - x0)

    def set_position(self, x, y):
        self.x = x
        self.y = y


class Rectangle:
    def __init__(self, name, x, y, w, h, fi, rho_min, lifetime):    # fi in degrees
        self.name = name
        self.x = x
        self.y = y
        self.w = w
        self.h = h
        self.fi = fi
        self.rho_min = rho_min
        self.lifetime = lifetime

    def set_position(self, x, y):
        self.x = x
        self.y = y

    def get_points(self):
        """
        :return: (x, y) tuples for up left, up right, down left, down right rectangle points in that order
        """
        x = self.x
        y = self.y
        fi = self.fi
        w2 = self.w / 2
        h2 = self.h / 2

        ulx, uly = rotate_point(-w2, h2, fi)
        urx, ury = rotate_point(w2, h2, fi)
        dlx, dly = rotate_point(-w2, -h2, fi)
        drx, dry = rotate_point(w2, -h2, fi)

        ul = (ulx + x, uly + y)  # UP LEFT
        ur = (urx + x, ury + y)  # UP RIGHT
        dl = (dlx + x, dly + y)  # DOWN LEFT
        dr = (drx + x, dry + y)  # DOWN RIGHT

        return ul, ur, dl, dr

    def get_distance_and_angle_from_point(self, x0, y0):
        ul, ur, dl, dr = self.get_points()

        a = point_to_line_segment_distance(x0, y0, ul, ur)
        b = point_to_line_segment_distance(x0, y0, ul, dl)
        c = point_to_line_segment_distance(x0, y0, dl, dr)
        d = point_to_line_segment_distance(x0, y0, dr, ur)

        m = min(a, b, c, d, key=lambda x: x[0])

        angle = atan2(m[1][1] - y0, m[1][0] - x0)

        return m[0], angle

    def get_closest_point(self, x0, y0):
        ul, ur, dl, dr = self.get_points()

        a = point_to_line_segment_distance(x0, y0, ul, ur)
        b = point_to_line_segment_distance(x0, y0, ul, dl)
        c = point_to_line_segment_distance(x0, y0, dl, dr)
        d = point_to_line_segment_distance(x0, y0, dr, ur)

        m = min(a, b, c, d, key=lambda x: x[0])

        return m[1]


class ObstacleManager:
    def __init__(self):
        self.obstacles_circ = []
        self.obstacles_rect = []

    def add_circle(self, name, x, y, r, rho_min=2, lifetime=-1):
        c = Circle(name, x, y, r, rho_min, lifetime)
        self.obstacles_circ.append(c)

    def add_rectangle(self, name, x, y, w, h, fi=0, rho_min=2, lifetime=-1):
        r = Rectangle(name, x, y, w, h, fi, rho_min, lifetime)
        self.obstacles_rect.append(r)

    def get_obstacle_by_name(self, name):
        obs = next((i for i in self.obstacles_circ if i.name == name), None)
        if obs is None:
            obs = next((i for i in self.obstacles_rect if i.name == name), None)

        return obs

    def get_obstacles(self):
        return self.obstacles_circ, self.obstacles_rect

    def remove_obstacle(self, name):
        obs = self.get_obstacle_by_name(name)
        if type(obs) == Circle:
            self.obstacles_circ.remove(obs)
        else:
            self.obstacles_rect.remove(obs)

    def set_position(self, name, x, y):
        obs = self.get_obstacle_by_name(name)
        obs.set_position(x, y)

    def update_all(self):
        for obs in self.obstacles_circ:
            if obs.lifetime > 0:
                obs.lifetime -= 1
                if obs.lifetime == 0:
                    self.obstacles_circ.remove(obs)

        for obs in self.obstacles_rect:
            if obs.lifetime > 0:
                obs.lifetime -= 1
                if obs.lifetime == 0:
                    self.obstacles_rect.remove(obs)


def point_to_line_segment_distance(x0, y0, line_p1, line_p2):
    """
    :param x0: point x coord
    :param y0: point y coord
    :param line_p1: line segment point 1 coord tuple
    :param line_p2: line segment point 2 coord tuple
    :return: Distance from point to line segment, (x, y) tuple of closest point on line segment
    """
    x1 = line_p1[0]
    y1 = line_p1[1]
    x2 = line_p2[0]
    y2 = line_p2[1]

    px = x2 - x1
    py = y2 - y1

    under = px * px + py * py

    if under == 0:
        return hypot(x1 - x0, y1 - y0), (x1, y1)

    u = ((x0 - x1) * px + (y0 - y1) * py) / under

    if u > 1:
        u = 1
    elif u < 0:
        u = 0

    x = x1 + u * px
    y = y1 + u * py

    dx = x - x0
    dy = y - y0

    # return sqrt((dx * dx + dy * dy)), (x, y)
    return hypot(dx, dy), (x, y)


def rotate_point(x0, y0, theta):
    """
    :param x0:  Point x coord
    :param y0:  Point y coord
    :param theta: Rotation angle in degrees
    :return: Rotated point coords
    """
    theta = theta * 3.1415 / 180    # convert to radians
    return x0 * cos(theta) - y0 * sin(theta), x0 * sin(theta) + y0 * cos(theta)


def is_line_segment_intersecting_circle(line_p1, line_p2, circle):
    """
    :param line_p1: first line segment point
    :param line_p2: second line segment point
    :param circle: circle to check collision with
    :return: bool
    """
    dist, _ = point_to_line_segment_distance(circle.x, circle.y, line_p1, line_p2)
    rho = dist - circle.r

    if rho < 0:
        return True
    else:
        return False


def line_segment_intersection(a, b, c, d):
    """
    :param a: segment AB point A
    :param b: segment AB point B
    :param c: segment CD point C
    :param d: segment CD point D
    :return: bool, tuple    Bool is true if there is intersection, tuple are coordinates of intersection
    """
    num1 = (a[1] - c[1]) * (d[0] - c[0]) - (a[0] - c[0]) * (d[1] - c[1])
    num2 = (a[1] - c[1]) * (b[0] - a[0]) - (a[0] - c[0]) * (b[1] - a[1])
    den = (b[0] - a[0]) * (d[1] - c[1]) - (b[1] - a[1]) * (d[0] - c[0])

    if den == 0 and num1 == 0:
        return True, (a[0] + 0.5 * (b[0] - a[0]), a[1] + 0.5 * (b[1] - a[1]))
    elif den == 0 and num1 != 0:
        return False, ('a', 'a')
    else:
        r = num1 / den
        s = num2 / den

        if r < 0 or r > 1 or s < 0 or s > 1:
            return False, ('a', 'a')
        else:
            return True, (a[0] + r * (b[0] - a[0]), a[1] + r * (b[1] - a[1]))


def is_line_segment_intersecting_rectangle(line_p1, line_p2, rect):
    """
    :param line_p1: first line segment point
    :param line_p2: second line segment point
    :param rect: rectangle to check collision with
    :return: bool
    """
    ul, ur, dl, dr = rect.get_points()
    edges = ((ul, ur), (ul, dl), (dr, dl), (dr, ur))
    intersects = False
    for edge in edges:
        f, _ = line_segment_intersection(line_p1, line_p2, edge[0], edge[1])
        if f:
            intersects = True
            break
    return intersects
