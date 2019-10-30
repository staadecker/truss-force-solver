import math
from typing import Dict, Tuple, Set, Optional
import tkinter

"""Limitations in bridge:
- One joint cannot have to beams going in the same direction.
- All external forces are aligned horizontally
- Uniformly distributed load
"""

NUMBER_OF_SIGNIFICANT_DIGITS = 3
TOLERANCE = 10 ** -8  # Used when checking if values are equal
FACTOR_OF_SAFETY_BUCKLING = 3
FACTOR_OF_SAFETY_YIELDING = 2
YIELD_STRESS = 350  # MPa


def values_are_equal(value_1: float, value_2: float) -> bool:
    return value_1 - TOLERANCE <= value_2 <= value_1 + TOLERANCE


def display_value(value: float) -> str:
    """Returns a formatted string for the value according to slide-rule precision."""
    if values_are_equal(0, value):
        return "0.00"
    number_of_digits = NUMBER_OF_SIGNIFICANT_DIGITS - int(math.floor(math.log10(abs(value)))) - 1

    # If decimals are dropped, convert to int
    if number_of_digits <= 0:
        return str(int(round(value, 0)))

    return str(round(value, number_of_digits))


class PairOfValues:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def __str__(self):
        return f"({display_value(self.x)}, {display_value(self.y)})"

    def __repr__(self):
        return self.__str__()

    def __hash__(self):
        return hash((self.x, self.y))

    def __eq__(self, other):
        return values_are_equal(self.x, other.x) and values_are_equal(self.y, other.y)


class Point(PairOfValues):
    def __init__(self, x, y):
        super().__init__(x, y)


class Vector(PairOfValues):
    def __init__(self, x: float, y: float):
        super().__init__(x, y)

    @classmethod
    def from_direction(cls, magnitude: float, radians: float):
        """
        :param magnitude: Number of radians from the positive x axis
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
        return math.sqrt(self.x ** 2 + self.y ** 2)

    def dot_product(self, other):
        return self.x * other.x + self.y * other.y

    def cos_theta(self, other):
        """
        :return: The result of cos theta where theta is the angle between the vectors
        """
        return self.dot_product(other) / (self.get_magnitude() * other.get_magnitude())

    def is_collinear(self, other):
        return values_are_equal(1, abs(self.cos_theta(other)))

    def __add__(self, other):
        return Vector(self.x + other.x, self.y + other.y)

    def __mul__(self, other):
        return Vector(other * self.x, other * self.y)

    def __truediv__(self, other):
        return Vector(self.x / other, self.y / other)

    def __sub__(self, other):
        return Vector(self.x - other.x, self.y - other.y)


Vector.ZERO = Vector(0, 0)


class Beam:
    """
    A pair of points that represents a beam with the special property that the order of the pair does not matter
    """

    def __init__(self, joint1: Point, joint2: Point):
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

    def get_length(self) -> float:
        return self.get_direction_vector().get_magnitude()


class BridgeData:
    def __init__(self, beams: Set[Beam], support_joints: Set[Point], area_load, width):
        """
        :param beams: A dictionary where each joint is a key and each joint points to a set of joints that is connected to the starting joint
        :param support_joints: A set where with the joints where there's an upward support
        """
        self.beams: Set[Beam] = beams
        self.support_joints: Set[Point] = support_joints
        self.area_load: float = area_load
        self.width: float = width


class BridgeFactory:
    @staticmethod
    def get_beams_for_equilateral_bridge(number_of_panels: int, span: float):
        beams = set()
        side_length = span / number_of_panels
        height = side_length * math.sqrt(3) / 2

        previous_top_corner = None

        for i in range(number_of_panels):
            # Corners
            left_corner = Point(i * side_length, 0)
            top_corner = Point((i + 0.5) * side_length, height)
            right_corner = Point((i + 1) * side_length, 0)

            beams.add(Beam(left_corner, right_corner))
            beams.add(Beam(left_corner, top_corner))
            beams.add(Beam(right_corner, top_corner))

            if previous_top_corner is not None:
                beams.add(Beam(previous_top_corner, top_corner))

            previous_top_corner = top_corner

        return beams

    @staticmethod
    def get_beams_for_right_angle_bridge(number_of_panels: int, span: float, angle_ratio: float):
        """
        :param number_of_panels: The number of times the sequence repeats
        :param span: The span of the bridge
        :param angle_ratio: The height divided by the width of one panel
        """
        beams = set()

        chord_length = span / number_of_panels
        height = chord_length * angle_ratio

        if number_of_panels % 2 == 1:
            raise Exception("Uneven number of panels. Can't build beams.")

        for i in range(number_of_panels):
            top_left = Point(i * chord_length, height)
            top_right = Point((i + 1) * chord_length, height)
            bottom_left = Point(i * chord_length, 0)
            bottom_right = Point((i + 1) * chord_length, 0)

            beams.add(Beam(bottom_left, bottom_right))

            if i < number_of_panels / 2:
                beams.add(Beam(bottom_left, top_right))
            else:
                beams.add(Beam(top_left, bottom_right))

            if i == 0:
                beams.add(Beam(bottom_right, top_right))
            elif i == number_of_panels - 1:
                pass
            else:
                beams.add(Beam(top_left, top_right))
                beams.add(Beam(bottom_right, top_right))

        return beams

    @staticmethod
    def add_supports_to_bridge(bridge: BridgeData, supports: Set[Point]):
        for support in supports:
            bridge.support_joints.add(support)


class BridgeCalculator:
    def __init__(self, bridge: BridgeData):
        self.data = bridge
        self.load_per_unit_length = bridge.area_load * bridge.width / 2  # Divide by two since trusses are on both sides
        self.enclosing_rectangle: Optional[Tuple] = None  # x_min, y_min, x_max, y_max
        self.member_forces: Dict[Beam, Optional[float]] = {}
        self.joints = {}
        self.external_forces = {}
        self.max_compression = None
        self.max_tension = None
        self.minimum_area = None
        self.minimum_i = None

    def build_joint_map(self):
        # Build up joints
        for beam in self.data.beams:
            self.member_forces[beam] = None

            if beam.joint1 not in self.joints:
                self.joints[beam.joint1] = set()

            if beam.joint2 not in self.joints:
                self.joints[beam.joint2] = set()

            self.joints[beam.joint1].add(beam.joint2)
            self.joints[beam.joint2].add(beam.joint1)

    def calculate_external_forces(self):
        external_forces_y_pos = None

        for support in self.data.support_joints:
            external_forces_y_pos = support.y

        load_bearing_joints_x_pos = []
        for joint in self.joints:
            if joint.y == external_forces_y_pos:
                load_bearing_joints_x_pos.append(joint.x)

        load_bearing_joints_x_pos = sorted(load_bearing_joints_x_pos)
        supports_x_pos = sorted([support.x for support in self.data.support_joints])

        # Loop from left to right throughout the joints
        for i, x_pos in enumerate(load_bearing_joints_x_pos):
            if i == 0:
                dist_to_neighbouring_joints = (load_bearing_joints_x_pos[i + 1] - x_pos)
            elif i < len(load_bearing_joints_x_pos) - 1:
                dist_to_neighbouring_joints = (load_bearing_joints_x_pos[i + 1] - load_bearing_joints_x_pos[i - 1])
            else:
                dist_to_neighbouring_joints = (x_pos - load_bearing_joints_x_pos[i - 1])

            self.external_forces[
                Point(x_pos, external_forces_y_pos)] = Vector(0,
                                                              -dist_to_neighbouring_joints / 2 * self.load_per_unit_length)

        for i, x_pos in enumerate(supports_x_pos):
            if i == 0:
                dist_to_neighbouring_joints = (supports_x_pos[i + 1] - x_pos)
            elif i < len(supports_x_pos) - 1:
                dist_to_neighbouring_joints = (supports_x_pos[i + 1] - supports_x_pos[i - 1])
            else:
                dist_to_neighbouring_joints = (x_pos - supports_x_pos[i - 1])

            self.external_forces[Point(x_pos, external_forces_y_pos)] += Vector(0,
                                                                                dist_to_neighbouring_joints / 2 * self.load_per_unit_length)

    def get_enclosing_rectangle(self):
        if self.enclosing_rectangle is not None:
            return self.enclosing_rectangle

        (min_x, min_y, max_x, max_y) = (None, None, None, None)
        for joint in self.joints.keys():
            min_x = min(joint.x, min_x) if min_x is not None else joint.x
            min_y = min(joint.y, min_y) if min_y is not None else joint.y

            max_x = max(joint.x, max_x) if max_x is not None else joint.x
            max_y = max(joint.y, max_y) if max_y is not None else joint.y

        self.enclosing_rectangle = (min_x, min_y, max_x, max_y)
        return self.enclosing_rectangle

    def calculate_member_forces(self):
        if len(self.joints) == 0:
            self.build_joint_map()

        if len(self.external_forces) == 0:
            self.calculate_external_forces()

        # Loop through every joint repeatedly.
        # At each joint look for member forces + external forces + joint load
        # In each component, verify if there's only one unknown_opposing_joints. Repeat in other component if solved
        # Repeat for other joints
        passes = 0
        calculated_joints = set()
        while passes < len(self.joints) * 2 and len(calculated_joints) < len(self.joints):
            for joint, opposing_joints in self.joints.items():
                if joint in calculated_joints:
                    continue

                sum_of_forces = Vector(0, 0)
                unknown_opposing_joints = []

                # Get forces in members
                for opposing_joint in opposing_joints:
                    direction_vector = Vector.from_a_to_b(joint, opposing_joint)
                    member_force = self.member_forces[Beam(joint, opposing_joint)]

                    if member_force is None:
                        unknown_opposing_joints.append(opposing_joint)
                    else:
                        sum_of_forces += direction_vector.get_unit_vector() * member_force

                # Get external forces
                if joint in self.external_forces:
                    sum_of_forces += self.external_forces[joint]

                if len(unknown_opposing_joints) > 3:
                    continue

                # If all forces are known make sure sum is zero and you're done
                elif len(unknown_opposing_joints) == 0:
                    if sum_of_forces != Vector(0, 0):
                        print(f"ERROR: Sum of forces at joint, {joint}, is {sum_of_forces} not (0, 0).")
                    calculated_joints.add(joint)

                # If only one force make sure it's opposite to the current sum of forces.
                elif len(unknown_opposing_joints) == 1:
                    unknown_opposing_joint = unknown_opposing_joints[0]
                    direction_vector = Vector.from_a_to_b(joint, unknown_opposing_joint)

                    if sum_of_forces == Vector(0, 0):
                        self.member_forces[Beam(unknown_opposing_joint, joint)] = 0
                    elif sum_of_forces.is_collinear(direction_vector):
                        self.member_forces[Beam(unknown_opposing_joint, joint)] = sum_of_forces.cos_theta(
                            direction_vector) * -1 * sum_of_forces.get_magnitude()
                    else:
                        print(
                            f"Error. Sum of forces ({sum_of_forces}) is not in the same direction as the only unknown beam (dir: {direction_vector}) for joint {joint}.")
                    calculated_joints.add(joint)

                elif len(unknown_opposing_joints) == 2:
                    unknown_joint_1 = unknown_opposing_joints.pop()
                    unknown_joint_2 = unknown_opposing_joints.pop()
                    unknown_force_1 = Vector.from_a_to_b(joint, unknown_joint_1)
                    unknown_force_2 = Vector.from_a_to_b(joint, unknown_joint_2)

                    if unknown_force_1.is_collinear(unknown_force_2):
                        continue

                    (force_1, force_2) = BridgeCalculator.solve_for_two_unknowns(sum_of_forces, unknown_force_1,
                                                                                 unknown_force_2)
                    self.member_forces[Beam(joint, unknown_joint_1)] = force_1
                    self.member_forces[Beam(joint, unknown_joint_2)] = force_2

                    calculated_joints.add(joint)

                elif len(unknown_opposing_joints) == 3:
                    unknown_joint_1 = unknown_opposing_joints.pop()
                    unknown_joint_2 = unknown_opposing_joints.pop()
                    unknown_joint_3 = unknown_opposing_joints.pop()
                    unknown_force_1 = Vector.from_a_to_b(joint, unknown_joint_1)
                    unknown_force_2 = Vector.from_a_to_b(joint, unknown_joint_2)
                    unknown_force_3 = Vector.from_a_to_b(joint, unknown_joint_3)

                    if unknown_force_1.is_collinear(unknown_force_2):
                        (force_1, force_3) = BridgeCalculator.solve_for_two_unknowns(sum_of_forces, unknown_force_1,
                                                                                     unknown_force_3)
                        self.member_forces[Beam(joint, unknown_joint_3)] = force_3
                    elif unknown_force_2.is_collinear(unknown_force_3):
                        (force_1, force_2) = BridgeCalculator.solve_for_two_unknowns(sum_of_forces, unknown_force_1,
                                                                                     unknown_force_2)
                        self.member_forces[Beam(joint, unknown_joint_1)] = force_1
                    elif unknown_force_3.is_collinear(unknown_force_1):
                        (force_1, force_2) = BridgeCalculator.solve_for_two_unknowns(sum_of_forces, unknown_force_1,
                                                                                     unknown_force_2)
                        self.member_forces[Beam(joint, unknown_joint_2)] = force_2

            passes += 1

    def get_highest_member_forces(self):
        self.max_compression = 0
        self.max_tension = 0
        for force in self.member_forces.values():
            self.max_compression = min(force, self.max_compression)
            self.max_tension = max(force, self.max_tension)

        return self.max_compression, self.max_tension

    def get_minimum_area_and_i(self):
        self.minimum_area = FACTOR_OF_SAFETY_YIELDING * max(self.max_tension, abs(self.max_compression)) / YIELD_STRESS

        self.minimum_i = 0
        for beam, force in self.member_forces.items():
            if force < 0:  # Compression
                i = FACTOR_OF_SAFETY_BUCKLING * abs(force) * (beam.get_length() ** 2) / ((math.pi ** 2) * 200000)
                if i > self.minimum_i:
                    self.minimum_i = i

        return self.minimum_area, self.minimum_i

    @staticmethod
    def solve_for_two_unknowns(sum_of_forces, unknown1, unknown2):
        a = sum_of_forces.x
        b = sum_of_forces.y
        c = unknown1.x
        d = unknown1.y
        e = unknown2.x
        f = unknown2.y
        force_1 = (b * e - a * f) / (f * c - d * e) * unknown1.get_magnitude()
        force_2 = (a * d - c * b) / (c * f - d * e) * unknown2.get_magnitude()
        return force_1, force_2


class BridgeGUI:
    MARGIN = 20
    HEIGHT_TARGET = 1000
    WIDTH_TARGET = 1400
    CANVAS_PADDING = 50
    LENGTH_OF_VECTOR = 50
    LENGTH_TO_LABEL = 25
    HEIGHT_OF_LABEL = 10
    WIDTH_OF_LABEL = 12
    INFORMATION_Y_SPACING = 25
    INFORMATION_X_SPACING = 500
    INFORMATION_LOOP_AFTER = 10

    def __init__(self):
        self.top = tkinter.Tk()
        self.scale_factor = None
        self.canvas = None
        self.height = None
        self.information_counter = 0

    def draw_bridge(self, bridge: BridgeCalculator):
        self.create_canvas(bridge)
        self.draw_beams(bridge)
        self.draw_external_forces(bridge)

    def create_canvas(self, bridge: BridgeCalculator):
        (min_x, min_y, max_x, max_y) = bridge.get_enclosing_rectangle()
        vertical_span = max_y - min_y
        horizontal_span = max_x - min_x

        scale_factor_vertical = BridgeGUI.HEIGHT_TARGET / vertical_span
        scale_factor_horizontal = BridgeGUI.WIDTH_TARGET / horizontal_span

        self.scale_factor = min(scale_factor_horizontal, scale_factor_vertical)

        self.height = (vertical_span * self.scale_factor + 2 * BridgeGUI.CANVAS_PADDING) * 2
        width = horizontal_span * self.scale_factor + 2 * BridgeGUI.CANVAS_PADDING

        self.canvas = tkinter.Canvas(self.top, height=self.height, width=width)

    def draw_beams(self, bridge: BridgeCalculator):
        for beam, force in bridge.member_forces.items():
            self.canvas.create_line(self.scale_x(beam.joint1.x), self.scale_y(beam.joint1.y),
                                    self.scale_x(beam.joint2.x), self.scale_y(beam.joint2.y))
            if force is not None:
                label_position = Vector.from_point(self.scale_point(beam.joint2)) + BridgeGUI.flip_vector_on_y(
                    Vector.from_a_to_b(beam.joint2, beam.joint1)) / 2 * self.scale_factor
                self.draw_label(label_position, force)

    def draw_external_forces(self, bridge: BridgeCalculator):
        for joint, force in bridge.external_forces.items():
            self.draw_vector(force, joint)

    def display(self):
        self.canvas.pack()
        self.top.mainloop()

    def draw_vector(self, vector: Vector, starting_point: Point):
        # Flip y since 0 0 is at top
        unscaled_vector_to_draw = BridgeGUI.flip_vector_on_y(vector).get_unit_vector()

        vector_tail = Vector.from_point(self.scale_point(starting_point))
        vector_tip = unscaled_vector_to_draw * BridgeGUI.LENGTH_OF_VECTOR + vector_tail
        label_position = vector_tail + unscaled_vector_to_draw * BridgeGUI.LENGTH_TO_LABEL

        self.draw_line(vector_tail, vector_tip, arrow=True)
        self.draw_label(label_position, vector.get_magnitude())

    def draw_label(self, position: Vector, content: float):
        tkinter.Label(self.canvas, text=display_value(content)).place(x=position.x - BridgeGUI.WIDTH_OF_LABEL,
                                                                      y=position.y - BridgeGUI.HEIGHT_OF_LABEL)

    def draw_line(self, a: PairOfValues, b: PairOfValues, arrow=False):
        self.canvas.create_line(a.x, a.y, b.x, b.y, arrow=tkinter.LAST if arrow else None)

    def add_information(self, information: str):
        x_pos = BridgeGUI.CANVAS_PADDING + BridgeGUI.INFORMATION_Y_SPACING * (
                self.information_counter // BridgeGUI.INFORMATION_LOOP_AFTER)
        y_pos = BridgeGUI.CANVAS_PADDING + (
                self.information_counter % BridgeGUI.INFORMATION_LOOP_AFTER) * BridgeGUI.INFORMATION_Y_SPACING
        tkinter.Label(self.canvas, text=information).place(x=x_pos,
                                                           y=y_pos)
        self.information_counter += 1

    def scale_point(self, point: Point) -> Point:
        return Point(self.scale_x(point.x), self.scale_y(point.y))

    def scale_x(self, value):
        return value * self.scale_factor + BridgeGUI.CANVAS_PADDING

    def scale_y(self, value):
        return self.height - value * self.scale_factor - BridgeGUI.CANVAS_PADDING

    @classmethod
    def flip_vector_on_y(cls, vector):
        return Vector(vector.x, -vector.y)


if __name__ == "__main__":
    AREA_LOAD = 5 + 1 + 0.75
    WIDTH = 3.7
    SPAN = 107.96 / 3
    HEIGHT_WIDTH_RATIO = 4 / 3

    beams = BridgeFactory.get_beams_for_right_angle_bridge(10, SPAN, HEIGHT_WIDTH_RATIO)
    supports = {
        Point(0, 0),
        Point(SPAN, 0)
    }
    myBridge = BridgeData(beams, supports, AREA_LOAD, WIDTH)
    # myBridge = BridgeFactory.build_equilateral_bridge(24, SPAN, LOAD_PER_UNIT_LENGTH)
    # BridgeFactory.add_supports_to_bridge(myBridge, {Point(SPAN/2, 0)})
    # BridgeFactory.add_supports_to_bridge(myBridge, {Point(SPAN / 3, 0), Point(SPAN / 3 * 2, 0)})
    bridge_calculator = BridgeCalculator(myBridge)
    bridge_calculator.build_joint_map()
    bridge_calculator.calculate_member_forces()
    (max_compression, max_tension) = bridge_calculator.get_highest_member_forces()
    (minimum_area, minimum_i) = bridge_calculator.get_minimum_area_and_i()

    minimum_area *= 1000
    minimum_i *= 10 ** 3  # Since meters and kN go to mm and N and then displayed as 10^6

    GUI = BridgeGUI()
    GUI.draw_bridge(bridge_calculator)
    GUI.add_information(f"Area Load: {AREA_LOAD} kN/mm^2")
    GUI.add_information(f"Width: 3.70m")
    GUI.add_information(f"Span: {display_value(SPAN)}m")
    GUI.add_information(f"Height/Width ratio: {display_value(HEIGHT_WIDTH_RATIO)}")
    GUI.add_information(f"Max compression: {display_value(max_compression)}kN")
    GUI.add_information(f"Max tension: {display_value(max_tension)}kN")
    GUI.add_information(f"Minimum area: {display_value(minimum_area)}mm^2")
    GUI.add_information(f"Minimum I: {display_value(minimum_i)} x 10^6 mm^4")

    GUI.display()
