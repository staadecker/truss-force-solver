import math
from typing import Dict, Tuple, Set, Optional
import tkinter

"""Limitations in bridge:
- One joint cannot have to beams going in the same direction.
- All external forces are aligned horizontally
- Uniformly distributed load
"""


def display_value(value):
    """Returns a formatted string for the value according to slide-rule precision."""
    if value == 0:
        return "0.00"
    string_value = str(value)
    for index, char in enumerate(string_value):
        if char in ("0", ".", "-"):
            continue

        target = index + 3
        if char == "1":
            target += 1
        if string_value[:target].find(".") != -1:
            target += 1
        return string_value[:target]


class Vector:
    TOLERANCE = 10 ** -6

    def __init__(self, x: float, y: float):
        self.x = x
        self.y = y

    @staticmethod
    def build_from_direction(magnitude: float, direction: float):
        """
        :param magnitude: Number of radians from the positive x axis
        :param direction:
        :return: The vector
        """
        return Vector(magnitude * math.cos(direction), magnitude * math.sin(direction))

    @staticmethod
    def from_a_to_b(a, b):
        return b - a

    def get_unit_vector(self):
        return self / self.get_magnitude()

    def get_magnitude(self):
        return math.sqrt(self.x ** 2 + self.y ** 2)

    def is_collinear(self, other):
        if other.x == 0 or self.x == 0:
            if other.x == 0 and self.x == 0:
                return True
            return False
        return other.y / other.x - Vector.TOLERANCE <= self.y / self.x <= other.y / other.x + Vector.TOLERANCE

    def sign_difference(self, other):
        """
        :return: 1 or -1 depending on if the vectors point in the same or opposite directions
        """
        return (self.x / other.x) / abs(self.x / other.x)

    def __add__(self, other):
        return Vector(self.x + other.x, self.y + other.y)

    def __mul__(self, other):
        return Vector(other * self.x, other * self.y)

    def __truediv__(self, other):
        return Vector(self.x / other, self.y / other)

    def __hash__(self):
        return hash((self.x, self.y))  # Double brackets for tuple

    def __eq__(self, other):
        return self.x - Vector.TOLERANCE <= other.x <= self.x + Vector.TOLERANCE \
               and self.y - Vector.TOLERANCE <= other.y <= self.y + Vector.TOLERANCE

    def __str__(self):
        return f"({display_value(self.x)}, {display_value(self.y)})"

    def __repr__(self):
        return self.__str__()

    def __sub__(self, other):
        return Vector(self.x - other.x, self.y - other.y)


class Point(Vector):
    def __init__(self, x, y):
        super().__init__(x, y)


class Beam:
    """
    A pair of points that represents a beam with the special property that the order of the pair does not matter
    """

    def __init__(self, joint1: Point, joint2: Point):
        self.joint1 = joint1
        self.joint2 = joint2

    def __eq__(self, other):
        return (self.joint1 == other.joint1 and self.joint2 == other.joint2) \
               or (self.joint2 == other.joint1 and self.joint1 == other.joint2)

    def __hash__(self):
        return hash(self.joint1) + hash(self.joint2)

    def __str__(self):
        return f"{repr(self.joint1)}->{repr(self.joint2)}"

    def __repr__(self):
        return self.__str__()

    def get_direction_vector(self) -> Vector:
        return Vector.from_a_to_b(self.joint1, self.joint2)


class BridgeData:
    def __init__(self, beams: Set[Beam], load_per_unit_length, supports: Set[Point], height_of_deck):
        """
        :param beams: A dictionary where each joint is a key and each joint points to a set of joints that is connected to the starting joint
        :param supports: A set where with the joints where there's an upward support
        """
        self.supports = supports
        self.beams = beams
        self.load_per_unit_length = load_per_unit_length
        self.height_of_deck = height_of_deck


class CalculatedBridge:
    def __init__(self, bridge: BridgeData):
        self.data = bridge
        self.vertical_span: Optional[Tuple[float, float]] = None
        self.horizontal_span: Optional[Tuple[float, float]] = None
        self.member_forces: Dict[Beam, Optional[float]] = {}
        self.joints = {}
        self.external_forces = {}

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
        if self.joints == {}:
            self.build_joint_map()

        inline_joint_x_values = []
        for joint in self.joints:
            if joint.y != self.data.height_of_deck:
                continue
            inline_joint_x_values.append(joint.x)

        inline_joint_x_values = sorted(inline_joint_x_values)

        for index, x_value in enumerate(inline_joint_x_values):
            if index == 0:
                length_covered = (inline_joint_x_values[index + 1] - x_value) / 2
            elif index < len(inline_joint_x_values) - 1:
                length_covered = (inline_joint_x_values[index + 1] - inline_joint_x_values[index - 1]) / 2
            else:
                length_covered = (x_value - inline_joint_x_values[index - 1]) / 2
            self.external_forces[Point(x_value, self.data.height_of_deck)] = Vector(0,
                                                                                    -length_covered * self.data.load_per_unit_length)

        sorted_support_x_values = sorted([support.x for support in self.data.supports])

        for index, x_value in enumerate(sorted_support_x_values):
            if index == 0:
                length_covered = (sorted_support_x_values[index + 1] - x_value) / 2
            elif index < len(sorted_support_x_values) - 1:
                length_covered = (sorted_support_x_values[index + 1] - sorted_support_x_values[index - 1]) / 2
            else:
                length_covered = (x_value - sorted_support_x_values[index - 1]) / 2
            self.external_forces[Point(x_value, self.data.height_of_deck)] += Vector(0,
                                                                                     length_covered * self.data.load_per_unit_length)

    def get_vertical_span(self):
        if self.vertical_span is not None:
            return self.vertical_span

        min_value = None
        max_value = None
        for joint in self.joints.keys():
            min_value = min(joint.y, min_value) if min_value is not None else joint.y
            max_value = max(joint.y, max_value) if max_value is not None else joint.y

        self.vertical_span = (min_value, max_value)
        return self.vertical_span

    def get_horizontal_span(self):
        if self.horizontal_span is not None:
            return self.horizontal_span

        min_value = None
        max_value = None
        for joint in self.joints.keys():
            min_value = min(joint.x, min_value) if min_value is not None else joint.x
            max_value = max(joint.x, max_value) if max_value is not None else joint.x

        self.horizontal_span = (min_value, max_value)
        return self.horizontal_span

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
                        self.member_forces[Beam(unknown_opposing_joint, joint)] = sum_of_forces.sign_difference(
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
                        if sum_of_forces != Vector(0, 0):
                            print(
                                f"Sum of the forces at joint {joint} cannot be zero since 2 unknown beams are collinear.")
                    else:
                        (force_1, force_2) = CalculatedBridge.solve_for_two_unknowns(sum_of_forces, unknown_force_1,
                                                                                     unknown_force_2)
                        self.member_forces[Beam(joint, unknown_joint_1)] = force_1
                        self.member_forces[Beam(joint, unknown_joint_2)] = force_2

                    calculated_joints.add(joint)

                elif len(unknown_opposing_joints) == 3:
                    unknown_joint_1 = unknown_opposing_joints.pop()
                    unknown_joint_2 = unknown_opposing_joints.pop()
                    unknown_joint_3 = unknown_opposing_joints.pop()
                    unknown_force_1 = unknown_joint_1 - joint
                    unknown_force_2 = unknown_joint_2 - joint
                    unknown_force_3 = unknown_joint_3 - joint

                    if unknown_force_1.is_collinear(unknown_force_2):
                        (force_1, force_3) = CalculatedBridge.solve_for_two_unknowns(sum_of_forces, unknown_force_1,
                                                                                     unknown_force_3)
                        self.member_forces[Beam(joint, unknown_joint_3)] = force_3
                    elif unknown_force_2.is_collinear(unknown_force_3):
                        (force_1, force_2) = CalculatedBridge.solve_for_two_unknowns(sum_of_forces, unknown_force_1,
                                                                                     unknown_force_2)
                        self.member_forces[Beam(joint, unknown_joint_1)] = force_1
                    elif unknown_force_3.is_collinear(unknown_force_1):
                        (force_1, force_2) = CalculatedBridge.solve_for_two_unknowns(sum_of_forces, unknown_force_1,
                                                                                     unknown_force_2)
                        self.member_forces[Beam(joint, unknown_joint_2)] = force_2

            passes += 1

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


class BridgeFactory:
    @staticmethod
    def build_equilateral_bridge(number_of_triangles: int, span: float, load_per_unit_length: float):
        beams = set()
        side_length = span / number_of_triangles
        height = side_length * math.sqrt(3) / 2

        previous_top_corner = None

        for i in range(number_of_triangles):
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

        return BridgeData(beams, load_per_unit_length, {Point(0, 0), Point(span, 0)}, 0)

    @staticmethod
    def add_supports_to_bridge(bridge: BridgeData, supports: Set[Point]):
        for support in supports:
            bridge.supports.add(support)


class BridgeGUI:
    MARGIN = 20
    HEIGHT_TARGET = 1000
    HORIZONTAL_TARGET = 1200
    CANVAS_PADDING = 50
    LENGTH_OF_VECTOR = 50
    LENGTH_TO_LABEL = 15

    def __init__(self):
        self.top = tkinter.Tk()
        self.scale_factor = None
        self.canvas = None
        self.height = None

    def draw_bridge(self, bridge: CalculatedBridge):
        self.create_canvas(bridge)
        self.draw_beams(bridge)
        self.draw_external_forces(bridge)

    def draw_beams(self, bridge: CalculatedBridge):
        for beam, force in bridge.member_forces.items():
            self.canvas.create_line(self.scalex(beam.joint1.x), self.scaley(beam.joint1.y),
                                    self.scalex(beam.joint2.x), self.scaley(beam.joint2.y))
            if force is not None:
                label_position = self.scale_point(beam.joint2) + BridgeGUI.flip_vector_on_y(
                    beam.joint1 - beam.joint2) / 2 * self.scale_factor
                self.draw_label(label_position, force)

    def create_canvas(self, bridge):
        vertical_span = bridge.get_vertical_span()
        horizontal_span = bridge.get_horizontal_span()
        vertical_span_distance = vertical_span[1] - vertical_span[0]
        horizontal_span_distance = horizontal_span[1] - horizontal_span[0]
        scale_factor_vertical = BridgeGUI.HEIGHT_TARGET / vertical_span_distance
        scale_factor_horizontal = BridgeGUI.HORIZONTAL_TARGET / horizontal_span_distance

        self.scale_factor = min(scale_factor_horizontal, scale_factor_vertical)
        self.height = vertical_span_distance * self.scale_factor + 2 * BridgeGUI.CANVAS_PADDING

        self.canvas = tkinter.Canvas(self.top, height=self.height,
                                     width=horizontal_span_distance * self.scale_factor + 2 * BridgeGUI.CANVAS_PADDING)

    def draw_external_forces(self, bridge: CalculatedBridge):
        for joint, force in bridge.external_forces.items():
            self.draw_vector(force, joint)

    def display(self):
        self.canvas.pack()
        self.top.mainloop()

    def draw_vector(self, vector, starting_point):
        # Flip y since 0 0 is at top
        unscaled_vector_to_draw = BridgeGUI.flip_vector_on_y(vector).get_unit_vector()

        starting_point_scaled = self.scale_point(starting_point)
        end_point = unscaled_vector_to_draw * BridgeGUI.LENGTH_OF_VECTOR + starting_point_scaled
        self.canvas.create_line(starting_point_scaled.x, starting_point_scaled.y, end_point.x, end_point.y,
                                arrow=tkinter.LAST)
        self.draw_label(starting_point_scaled + unscaled_vector_to_draw * BridgeGUI.LENGTH_TO_LABEL,
                        vector.get_magnitude())

    def draw_label(self, position: Vector, content: float):
        tkinter.Label(self.canvas, text=display_value(content)).place(x=position.x, y=position.y)

    def scale_point(self, vector: Vector):
        return Vector(self.scalex(vector.x), self.scaley(vector.y))

    def scalex(self, value):
        return value * self.scale_factor + BridgeGUI.CANVAS_PADDING

    def scaley(self, value):
        return self.height - value * self.scale_factor - BridgeGUI.CANVAS_PADDING

    @staticmethod
    def flip_vector_on_y(vector):
        return Vector(vector.x, -vector.y)


if __name__ == "__main__":
    myBridge = BridgeFactory.build_equilateral_bridge(4, 40, 1)
    calculatedBridge = CalculatedBridge(myBridge)
    calculatedBridge.calculate_member_forces()
    GUI = BridgeGUI()
    GUI.draw_bridge(calculatedBridge)
    GUI.display()
