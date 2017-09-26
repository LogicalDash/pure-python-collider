from math import floor, ceil
from array import array


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

    This pure Python version was ported from the kivy.garden.collider
    package.
    '''
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
