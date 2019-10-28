import math
from typing import Dict, Tuple, Set, Optional
import tkinter


def display_value(value):
    return str(round(value, 2))


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


class Beam:
    """
    A pair of points that represents a beam with the special property that the order of the pair does not matter
    """

    def __init__(self, joint1: Vector, joint2: Vector):
        self.joint1 = joint1
        self.joint2 = joint2

    def __eq__(self, other):
        return (self.joint1 == other.joint1 and self.joint2 == other.joint2) or (
                self.joint2 == other.joint1 and self.joint1 == other.joint2)

    def __hash__(self):
        return hash(self.joint1) + hash(self.joint2)

    def __str__(self):
        return f"{repr(self.joint1)}->{repr(self.joint2)}"

    def __repr__(self):
        return self.__str__()


Beams = Dict[Vector, Set[Vector]]


class BridgeData:
    def __init__(self, beams: Beams, external_loads: Dict[Vector, Vector],
                 supports: Dict[Vector, Set[Vector]]):
        """
        :param beams: A dictionary where each joint is a key and each joint points to a set of joints that is connected to the starting joint
        :param external_loads: A dictionary where joints map to the vector load at that joint
        :param supports: A dictionary where joints map to a set of vectors representing reaction forces
        """
        self.supports = supports
        self.joints = beams
        self.external_loads = external_loads


class CalculatedBridge:
    def __init__(self, bridge: BridgeData):
        self.data = bridge
        self.vertical_span: Optional[Tuple[float, float]] = None
        self.horizontal_span: Optional[Tuple[float, float]] = None
        self.member_forces: Dict[Beam, Optional[float]] = {}

        for joint, opposite_joints in self.data.joints.items():
            for opposite_joint in opposite_joints:
                self.member_forces[Beam(joint, opposite_joint)] = None

    def get_vertical_span(self):
        if self.vertical_span is not None:
            return self.vertical_span

        min_value = None
        max_value = None
        for joint in self.data.joints.keys():
            min_value = min(joint.y, min_value) if min_value is not None else joint.y
            max_value = max(joint.y, max_value) if max_value is not None else joint.y

        self.vertical_span = (min_value, max_value)
        return self.vertical_span

    def get_horizontal_span(self):
        if self.horizontal_span is not None:
            return self.horizontal_span

        min_value = None
        max_value = None
        for joint in self.data.joints.keys():
            min_value = min(joint.x, min_value) if min_value is not None else joint.x
            max_value = max(joint.x, max_value) if max_value is not None else joint.x

        self.horizontal_span = (min_value, max_value)
        return self.horizontal_span

    def calculate_member_forces(self):
        # Loop through every joint repeatedly.
        # At each joint look for member forces + external forces + joint load
        # In each component, verify if there's only one unknown_opposing_joints. Repeat in other component if solved
        # Repeat for other joints
        passes = 0
        calculated_joints = set()
        while passes < len(self.data.joints) * 2 and len(calculated_joints) < len(self.data.joints):
            for joint, opposing_joints in self.data.joints.items():
                if joint in calculated_joints:
                    continue

                sum_of_forces = Vector(0, 0)
                unknown_opposing_joints = []

                # Get known forces
                for opposing_joint in opposing_joints:
                    direction_vector = Vector.from_a_to_b(joint, opposing_joint)
                    member_force = self.member_forces[Beam(joint, opposing_joint)]

                    if member_force is None:
                        unknown_opposing_joints.append(opposing_joint)
                    else:
                        sum_of_forces += direction_vector.get_unit_vector() * member_force

                # Get loads
                if joint in self.data.external_loads:
                    sum_of_forces += self.data.external_loads[joint]

                # Get support forces
                if joint in self.data.supports:
                    for force in self.data.supports[joint]:
                        sum_of_forces += force

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

                    if sum_of_forces.is_collinear(direction_vector):
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
        beams: Beams = {}
        side_length = span / number_of_triangles
        height = side_length * math.sqrt(3) / 2
        load_per_joint = Vector(0, -load_per_unit_length * side_length)
        external_loads = {}

        previous_top_corner = None

        for i in range(number_of_triangles):
            # Corners
            left_corner = Vector(i * side_length, 0)
            top_corner = Vector((i + 0.5) * side_length, height)
            right_corner = Vector((i + 1) * side_length, 0)

            # Add joints
            beams[left_corner] = set() if left_corner not in beams else beams[left_corner]
            beams[right_corner] = set() if right_corner not in beams else beams[right_corner]
            beams[top_corner] = set() if top_corner not in beams else beams[top_corner]

            # Connect joints
            beams[left_corner].update([top_corner, right_corner])
            beams[right_corner].update([left_corner, top_corner])
            beams[top_corner].update([left_corner, right_corner])

            if previous_top_corner is not None:
                beams[previous_top_corner].add(top_corner)
                beams[top_corner].add(previous_top_corner)

            if i != 0:
                external_loads[left_corner] = load_per_joint

            previous_top_corner = top_corner

        upward_support = Vector(0, 1) * load_per_joint.get_magnitude() * (number_of_triangles - 1) / 2
        return BridgeData(beams, external_loads,
                          {Vector(0, 0): {upward_support}, Vector(span, 0): {upward_support}})


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
        for joint, support in bridge.data.supports.items():
            for reaction in support:
                self.draw_vector(reaction, joint)

        for joint, force in bridge.data.external_loads.items():
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
    myBridge = BridgeFactory.build_equilateral_bridge(4, 40, 2)
    calculatedBridge = CalculatedBridge(myBridge)
    calculatedBridge.calculate_member_forces()
    GUI = BridgeGUI()
    GUI.draw_bridge(calculatedBridge)
    GUI.display()
