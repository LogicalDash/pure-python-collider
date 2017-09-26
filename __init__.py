from math import floor, ceil, pi, cos, sin, tan, atan2, fabs
from array import array


def ellipse_tan_dot(rx, ry, px, py, theta):
    return ((rx ** 2 - ry ** 2) * cos(theta) * sin(theta) -
            px * rx * sin(theta) + py * ry * cos(theta))


def ellipse_tan_dot_derivative(rx, ry, px, py, theta):
    return ((rx ** 2 - ry ** 2) * (cos(theta) ** 2 - sin(theta) ** 2) -
            px * rx * cos(theta) - py * ry * sin(theta))


class Collide2DPoly(object):
    ''' Collide2DPoly checks whether a point is within a polygon defined by a
    list of corner points.

    Based on http://alienryderflex.com/polygon/

    For example, a simple triangle::

        >>> collider = Collide2DPoly([10., 10., 20., 30., 30., 10.],
        ... cache=True)
        >>> (0.0, 0.0) in collider
        False
        >>> (20.0, 20.0) in collider
        True

    The constructor takes a list of x,y points in the form of [x1,y1,x2,y2...]
    as the points argument. These points define the corners of the
    polygon. The boundary is linearly interpolated between each set of points.
    The x, and y values must be floating points.
    The cache argument, if True, will calculate membership for all the points
    so when collide_point is called it'll just be a table lookup.
    '''
    __slots__ = ['points', 'count', 'constant', 'multiple',
                 'min_x', 'max_x', 'min_y', 'max_y',
                 'width', 'space']
    def __init__(self, points, cache=False, **kwargs):
        length = len(points)
        if length % 2:
            raise IndexError()
        if length < 4:
            self.points = None
            return
        count = length // 2
        self.count = count
        self.points = points = array('d', points)
        self.constant = constant = array('d', [0.0] * count)
        self.multiple = multiple = array('d', [0.0] * count)

        self.min_x = min(points[0::2])
        self.max_x = max(points[0::2])
        self.min_y = min(points[1::2])
        self.max_y = max(points[1::2])
        min_x = floor(self.min_x)
        min_y = floor(self.min_y)
        j = count - 1
        if cache:
            for i in range(count):
                points[2 * i] -= min_x
                points[2 * i + 1] -= min_y

        for i in range(count):
            i_x = i * 2
            i_y = i_x + 1
            j_x = j * 2
            j_y = j_x + 1
            if points[j_y] == points[i_y]:
                constant[i] = points[i_x]
                multiple[i] = 0.
            else:
                constant[i] = (points[i_x] - points[i_y] * points[j_x] /
                                (points[j_y] - points[i_y]) +
                                points[i_y] * points[i_x] /
                                (points[j_y] - points[i_y]))
                multiple[i] = ((points[j_x] - points[i_x]) /
                                (points[j_y] - points[i_y]))
            j = i
        if cache:
            width = int(ceil(self.max_x) - min_x + 1.)
            self.width = width
            height = int(ceil(self.max_y) - min_y + 1.)
            self.space = []
            for y in range(height):
                for x in range(width):
                    j = count - 1
                    odd = 0
                    for i in range(count):
                        i_y = i * 2 + 1
                        j_y = j * 2 + 1
                        if (points[i_y] < y and points[j_y] >= y or
                                        points[j_y] < y and points[i_y] >= y):
                            odd ^= y * multiple[i] + constant[i] < x
                        j = i
                    self.space[y * width + x] = odd

    def collide_point(self, x, y):
        points = self.points
        if not points or not (self.min_x <= x <= self.max_x and
                                              self.min_y <= y <= self.max_y):
            return False
        if hasattr(self, 'space'):
            y -= floor(self.min_y)
            x -= floor(self.min_x)
            return self.space[int(y) * self.width + int(x)]

        j = self.count - 1
        odd = 0
        for i in range(self.count):
            i_y = i * 2 + 1
            j_y = j * 2 + 1
            if (points[i_y] < y and points[j_y] >= y or
                            points[j_y] < y and points[i_y] >= y):
                odd ^= y * self.multiple[i] + self.constant[i] < x
            j = i
        return odd

    def __contains__(self, point):
        return self.collide_point(*point)

    def __iter__(self):
        for x in range(int(floor(self.min_x)), int(ceil(self.max_x)) + 1):
            for y in range(int(floor(self.min_y)), int(ceil(self.max_y)) + 1):
                if self.collide_point(x, y):
                    yield x, y

    def get_inside_points(self):
        """Return a list of all the integral points that are within the polygon."""
        return list(self)

    def bounding_box(self):
        """Return the bounding box containing the polygon as 4 points
        (x1, y1, x2, y2), where x1, y1 is the lower left and x2, y2
        is the upper right point of the rectangle.

        """
        return (int(floor(self.min_x)), int(floor(self.min_y)),
                int(ceil(self.max_x)), int(ceil(self.max_y)))

    def get_area(self):
        return len(self.get_inside_points())

    def get_centroid(self):
        if not self.points:
            return 0, 0
        x = y = 0
        for i in range(self.count):
            x += self.points[2 * i]
            y += self.points[2 * i + 1]
        x /= float(self.count)
        y /= float(self.count)
        if self.space:
            return x + self.min_x, y + self.min_y
        return x, y


def convert_to_poly(points, segments):
    T = list(points)
    poly = []

    for x in range(segments):
        l = x / float(segments)
        # http://en.wikipedia.org/wiki/De_Casteljau%27s_algorithm
        # as the list is in the form of (x1, y1, x2, y2...) iteration is
        # done on each item and the current item (xn or yn) in the list is
        # replaced with a calculation of "xn + x(n+1) - xn" x(n+1) is
        # placed at n+2. Each iteration makes the list one item shorter
        for i in range(1, len(T)):
            for j in range(len(T) - 2 * i):
                T[j] += (T[j+2] - T[j]) * l
        # we got the coordinates of the point in T[0] and T[1]
        poly.append(T[0])
        poly.append(T[1])
    # add one last point to join the curve to the end
    poly.append(T[-2])
    poly.append(T[-1])
    return poly


class CollideBezier(object):
    """Tests whether a point falls inside a Bezier curve,
     itself described by a list of points.

    """
    __slots__ = ['line_collider']

    def __init__(self, points, cache=False, segments=180, **kwargs):
        length = len(points)
        if length % 2:
            raise IndexError("Odd number of points provided")
        if length < 6:
            self.line_collider = None
            return
        if segments <= 1:
            raise ValueError("Invalid segments value, must be >= 2")

        self.line_collider = Collide2DPoly(
            convert_to_poly(points, segments), cache=cache
        )

    @staticmethod
    def convert_to_poly(points, segments=180):
        return convert_to_poly(points, segments)

    def collide_point(self, x, y):
        if self.line_collider is None:
            return False
        return self.line_collider.collide_point(x, y)

    def __contains__(self, point):
        if self.line_collider is None:
            return False
        return self.line_collider.collide_point(*point)

    def __iter__(self):
        if self.line_collider is None:
            return
        return iter(self.line_collider)

    def get_inside_points(self):
        return list(self)

    def bounding_box(self):
        if self.line_collider is None:
            return 0, 0, 0, 0
        return self.line_collider.bounding_box()

    def get_area(self):
        if self.line_collider is None:
            return 0
        return self.line_collider.get_area()

    def get_centroid(self):
        if self.line_collider is None:
            return 0, 0
        return self.line_collider.get_centroid()


class CollideEllipse(object):
    """CollideEllipse checks whether a point is within an ellipse or circle
    aligned with the Cartesian plain, as defined by a center point and a major
    and minor radius. That is, the major and minor axes are along the x and y
    axes.

    :Parameters:
        `x`: float
            the x position of the center of the ellipse
        `y`: float
            the y position of the center of the ellipse
        `rx`: float
            the radius of the ellipse along the x direction
        `ry`: float
            the radius of the ellipse along the y direction

    For example::

        >>> collider = CollideEllipse(x=0, y=10, rx=10, ry=10)
        >>> (0, 10) in collider
        True
        >>> (0, 0) in collider
        True
        >>> (10, 10) in collider
        True
        >>> (11, 10) in collider
        False
        >>>
        >>> collider = CollideEllipse(x=0, y=10, rx=20, ry=10)
        >>> (20, 10) in collider
        True
        >>> (21, 10) in collider
        False

    """
    __slots__ = ['x', 'y', 'rx', 'ry', 'angle', '_rx_squared', '_ry_squared', '_circle', '_zero_circle']

    def __init__(self, x, y, rx, ry, angle=0, **kwargs):
        if rx < 0. or ry < 0.:
            raise ValueError('The radii must be zero or positive')

        self.x = x
        self.y = y
        self.rx = rx
        self.ry = ry
        self.angle = -angle / 180. * pi
        self._circle = rx == ry
        self._rx_squared = rx ** 2
        self._ry_squared = ry ** 2
        self._zero_circle = self._rx_squared == 0 or self._ry_squared == 0

    def collide_point(self, x, y):
        if self._zero_circle:
            return False

        x -= self.x
        y -= self.y
        if self._circle:
            return x ** 2 + y ** 2 <= self._rx_squared
        if self.angle:
            x, y = x * cos(self.angle) - y * sin(self.angle), x * sin(self.angle) + y * cos(self.angle)
        return x ** 2 / self._rx_squared + y ** 2 / self._ry_squared <= 1.

    def __contains__(self, point):
        return self.collide_point(*point)

    def __iter__(self):
        x1, y1, x2, y2 = self.bounding_box()

        for x in range(x1, x2 + 1):
            for y in range(y1, y2 + 1):
                if self.collide_point(x, y):
                    yield x, y

    def get_inside_points(self):
        return list(self)

    def bounding_box(self):
        if self._zero_circle:
            return 0, 0, 0, 0

        if not self.angle or self._circle:
            return (int(floor(-self.rx + self.x)), int(floor(-self.ry + self.y)),
                    int(ceil(self.rx + self.x)), int(ceil(self.ry + self.y)))

        # from http://stackoverflow.com/a/88020/778140
        phi = -self.angle
        t = atan2(-self.ry * tan(phi), self.rx)
        x1 = self.x + self.rx * cos(t) * cos(phi) - self.ry * sin(t) * sin(phi)
        t += pi
        x2 = self.x + self.rx * cos(t) * cos(phi) - self.ry * sin(t) * sin(phi)

        t = atan2(self.ry * tan(pi / 2. - phi), self.rx)
        y1 = self.y + self.ry * sin(t) * cos(phi) + self.rx * cos(t) * sin(phi)
        t += pi
        y2 = self.y + self.ry * sin(t) * cos(phi) + self.rx * cos(t) * sin(phi)

        if x2 < x1:
            x1, x2 = x2, x1

        if y2 < y1:
            y1, y2 = y2, y1

        return int(floor(x1)), int(floor(y1)), int(ceil(x2)), int(ceil(y2))

    def estimate_distance(self, x, y, error=1e-5):
        # from http://www.ma.ic.ac.uk/~rn/distance2ellipse.pdf
        if self._zero_circle:
            return 0

        x -= self.x
        y -= self.y
        if self._circle:
            return fabs((x ** 2 + y ** 2) ** .5 - self.rx)

        if self.angle:
            x, y = x * cos(self.angle) - y * sin(self.angle), x * sin(self.angle) + y * cos(self.angle)

        theta = atan2(self.rx * y, self.ry * x)
        while fabs(ellipse_tan_dot(self.rx, self.ry, x, y, theta)) > error:
            theta -= ellipse_tan_dot(
                self.rx, self.ry, x, y, theta
            ) / ellipse_tan_dot_derivative(self.rx, self.ry, x, y, theta)

        px, py = self.rx * cos(theta), self.ry * sin(theta)
        return ((x - px) ** 2 + (y - py) ** 2) ** .5

    def get_area(self):
        return pi * self.rx * self.ry

    def get_centroid(self):
        return self.x, self.y
