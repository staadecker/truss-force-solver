import math

from core import values_are_equal


class PairOfValues:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def __str__(self):
        return f"({str(self.x)}, {str(self.y)})"

    def __repr__(self):
        return self.__str__()

    def __hash__(self):
        return hash((float(self.x), float(self.y)))

    def __eq__(self, other):
        return values_are_equal(self.x, other.x) and values_are_equal(self.y, other.y)


class Point(PairOfValues):
    def __init__(self, x, y):
        super().__init__(x, y)


class Vector(PairOfValues):
    def __init__(self, x, y):
        super().__init__(x, y)

    @classmethod
    def from_direction(cls, magnitude, radians: float):
        """
        :param magnitude of radians from the positive x axis
        :param radians:
        :return: The vector
        """
        return Vector(magnitude * math.cos(radians), magnitude * math.sin(radians))

    @staticmethod
    def from_a_to_b(a: Point, b: Point):
        return Vector(b.x - a.x, b.y - a.y)

    @staticmethod
    def from_point(point: Point):
        return Vector(point.x, point.y)

    def get_unit_vector(self):
        return self / self.get_magnitude()

    def get_magnitude(self):
        return (self.x ** 2 + self.y ** 2) ** 0.5

    def dot_product(self, other):
        return self.x * other.x + self.y * other.y

    def cos_theta(self, other):
        """
        :return: The result of cos theta where theta is the angle between the vectors
        """
        return self.dot_product(other) / (self.get_magnitude() * other.get_magnitude())

    def is_collinear(self, other):
        value = abs(self.cos_theta(other))
        return values_are_equal(1, value)

    def __add__(self, other):
        return Vector(self.x + other.x, self.y + other.y)

    def __mul__(self, other):
        return Vector(self.x * other, self.y * other)

    def __truediv__(self, other):
        return Vector(self.x / other, self.y / other)

    def __sub__(self, other):
        return Vector(self.x - other.x, self.y - other.y)


class Beam:
    """
    A pair of points that represents a beam with the special property that the order of the pair does not matter
    """

    def __init__(self, joint1: Point, joint2: Point):
        """
        :param joint1: Position of one end of the beam
        :param joint2: Position of the other end of the beam
        :param hss_set: String representing the
        """
        self.joint1: Point = joint1
        self.joint2: Point = joint2

    def __eq__(self, other):
        return (self.joint1 == other.joint1 and self.joint2 == other.joint2) or (
                self.joint2 == other.joint1 and self.joint1 == other.joint2)

    def __hash__(self):
        return hash(self.joint1) + hash(self.joint2)  # Addition because commutative

    def __str__(self):
        return f"{repr(self.joint1)}<->{repr(self.joint2)}"

    def __repr__(self):
        return self.__str__()

    def get_direction_vector(self) -> Vector:
        return Vector.from_a_to_b(self.joint1, self.joint2)

    def get_length(self):
        return self.get_direction_vector().get_magnitude()