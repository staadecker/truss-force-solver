import math
import quantities as pq


def values_are_equal(value_1, value_2) -> bool:
    float_1 = float(value_1)
    return float_1 - TOLERANCE <= float(value_2) <= float_1 + TOLERANCE


def display_float(value):
    if values_are_equal(0, value):
        return "0.00" + " " + str(value.dimensionality)
    number_of_digits = NUMBER_OF_SIGNIFICANT_DIGITS - int(math.floor(math.log10(abs(value)))) - 1
    # If decimals are dropped, convert to int
    if number_of_digits <= 0:
        return str(int(round(value, number_of_digits))) + " " + str(value.dimensionality)
    return str(round(value, number_of_digits))


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


Vector.ZERO = Vector(0, 0)

NUMBER_OF_SIGNIFICANT_DIGITS = 3
TOLERANCE = 10 ** -8  # Used when checking if floats are equal
FACTOR_OF_SAFETY_BUCKLING = 3
FACTOR_OF_SAFETY_YIELDING = 2
YIELD_STRESS = 350 * pq.MPa
YOUNG_MODULUS = 200000 * pq.MPa
kN = pq.UnitQuantity("kiloNewton", pq.N * 1000, "kN")
