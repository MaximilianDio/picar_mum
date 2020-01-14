import math


def diff(p1, p2):
    """

    :param p1: poin2D object with first value x and second value y
    :param p2: same as p1
    :return: float - slope f'(x) ~= (y2-y1)/(x2-x1)
    """
    try:
        if isinstance(p1, Point2D) and isinstance(p2, Point2D):
            return (p2.y - p1.y) / (p2.x - p1.x)
        else:
            raise ValueError("input must be 2 point2D objects")
    except ZeroDivisionError:
        raise ValueError("points must have different x-values")


def midpoint(p1, p2):
    """

    :param p1: list of shape [x,y]
    :param p2: same as p1
    :return: point2D - midpoint between both points
    """
    if isinstance(p1, Point2D) and isinstance(p2, Point2D):
        return Point2D([(p1.x + p2.x) / 2, (p1.y + p2.y) / 2])
    else:
        raise ValueError("input must be 2 point2D objects")


def isclose(a, b, rel_tol=1e-09, abs_tol=0.0):
    return abs(a - b) <= max(rel_tol * max(abs(a), abs(b)), abs_tol)


class Point2D(object):

    def __init__(self, point):
        if not isinstance(point[0], list) and not isinstance(point[1], list):
            self.x = float(point[0])
            self.y = float(point[1])
        else:
            raise ValueError("points must be 1D list or tuple of length 2")


class CurvePoint2D(Point2D):
    def __init__(self, point):
        self.slope = None
        self.circle = None
        super(CurvePoint2D, self).__init__(point)


class Circle2D:

    def __init__(self, *args, **kwargs):
        if len(args) == 2:
            radius = args[0]
            center_point = args[1]

            if isinstance(radius, float) and isinstance(center_point, Point2D):
                self.center = radius
                self.center = Point2D([center_point.x, center_point.y])
        elif len(args) == 3:
            p1 = args[0]
            p2 = args[1]
            p3 = args[2]

            if isinstance(p1, Point2D) and isinstance(p2, Point2D) and isinstance(p3, Point2D):
                self.calc_circle(p1, p2, p3)
            else:
                raise ValueError("input must be 3 point2D objects")

    def calc_circle(self, p1, p2, p3):

        """

        :param p1: list of shape [x,y] with first value the variable (e.g. x) and second function value (e.g. y)
        :param p2: same as p1
        :param p3: same as p1
        :return: center of circle and radius
        """

        # calculate midpoints of points
        mp1 = midpoint(p1, p2)
        mp2 = midpoint(p2, p3)

        # calculate orthogonal gradient of midpoint lines
        try:
            grad_L1 = -1 / diff(p1, p2)
            grad_L2 = -1 / diff(p2, p3)

            if isclose(grad_L1, grad_L2):
                self.center = Point2D([float("inf"), float("inf")])
                self.radius = float("inf")
                return

            # no edge case
            x = 1 / (grad_L1 - grad_L2) * (-mp1.y + mp2.y + grad_L1 * mp1.x - grad_L2 * mp2.x)
            y = grad_L1 * (x - mp1.x) + mp1.y
        except ZeroDivisionError:
            # go through edge cases:
            gradp1p2 = diff(p1, p2)
            gradp2p3 = diff(p2, p3)
            if isclose(gradp1p2, 0.0) and isclose(gradp2p3, 0.0):
                self.center = Point2D([float("inf"), float("inf")])
                self.radius = float("inf")
                return
            elif isclose(gradp1p2, 0.0):
                grad_L2 = -1 / gradp2p3
                x = mp1.x
                y = grad_L2 * (x - mp2.x) + mp2.y
            elif isclose(gradp2p3, 0.0):
                grad_L1 = -1 / gradp1p2
                x = mp2.x
                y = grad_L1 * (x - mp1.x) + mp1.y

        # calculate radius
        radius = math.sqrt(math.pow(p1.x - x, 2) + math.pow(p1.y - y, 2))

        self.center = Point2D([x, y])
        self.radius = radius
