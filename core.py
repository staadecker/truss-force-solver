import math
import quantities as pq
from typing import Dict, Tuple, Set, Optional

from utilities import Point, Vector, FACTOR_OF_SAFETY_YIELDING, FACTOR_OF_SAFETY_BUCKLING, YOUNG_MODULUS, \
    YIELD_STRESS, kN

"""Limitations in bridge:
- One joint cannot have two beams going in the same direction.
- All external forces act on one horizontally axis
"""


class Beam:
    """
    A pair of points that represents a beam with the special property that the order of the pair does not matter
    """

    def __init__(self, joint1: Point, joint2: Point):
        """
        :param joint1: Position of one end of the beam
        :param joint2: Position of the other end of the beam
        """
        self.joint1: Point = joint1
        self.joint2: Point = joint2

    def __eq__(self, other):
        # Order doesn't matter!
        return (self.joint1 == other.joint1 and self.joint2 == other.joint2) or (
                self.joint2 == other.joint1 and self.joint1 == other.joint2)

    def __hash__(self):
        return hash(self.joint1) + hash(self.joint2)  # Addition because commutative and order shouldn't matter

    def __str__(self):
        return f"{repr(self.joint1)}<->{repr(self.joint2)}"

    def __repr__(self):
        return self.__str__()

    def get_direction_vector(self) -> Vector:
        """
        Note that this function can return different direction vectors depending on the order of joint1 and joint2
        """
        return Vector.from_a_to_b(self.joint1, self.joint2)

    def get_length(self):
        return self.get_direction_vector().get_magnitude()


class BeamGroup:
    def __init__(self, name, key: int, color: str):
        """
        :param name: The name of the group.
        :param key: An integer key to differentiate groups.
        :param color: A color that is supported by tkinter
        """
        self.name = name
        self.key = key
        self.color = color

    def __eq__(self, other):
        return self.key == other.key

    def __hash__(self):
        return self.key

    def __str__(self):
        return self.name + " (" + self.color + ")"

    def __repr__(self):
        return self.__str__()


class BeamGroupProperties:
    def __init__(self):
        self.max_compression = 0 * kN
        self.max_tension = 0 * kN
        self.min_area = 0 * pq.mm ** 2  # Minimum area of HSS beams such that yielding does not occur
        self.min_i = 0 * pq.mm ** 4  # Minimum second moment of inertia of beams such that buckling does not occur


class BeamProperties:
    def __init__(self, beam_group: BeamGroup):
        self.beam_group = beam_group
        self.member_force = None


class BridgeData:
    def __init__(self, beams: Dict[Beam, BeamProperties]):
        """
        :param beams: A dictionary containing all beams and there respective properties
        """
        self.beams: Dict[Beam, BeamProperties] = beams

        self.joints: Dict[Point, Set[Point]] = {}
        self.build_joint_map()

        self.joint_loads: Dict[Point, Vector] = {}

    def build_joint_map(self):
        # For each beam, add joint connection to joint map
        for beam in self.beams:
            if beam.joint1 not in self.joints:
                self.joints[beam.joint1] = set()

            if beam.joint2 not in self.joints:
                self.joints[beam.joint2] = set()

            self.joints[beam.joint1].add(beam.joint2)
            self.joints[beam.joint2].add(beam.joint1)


class BridgeFactory:
    def __init__(self):
        self.hss_top_chord_set = BeamGroup("Top chords", 0, "red")
        self.hss_web_set = BeamGroup("Web", 1, "blue")
        self.hss_bottom_chord_set = BeamGroup("Bottom chords", 2, "green")

    def get_beams_for_equilateral_bridge(self, number_of_panels: int, span):
        beams = {}
        side_length = span / number_of_panels
        height = side_length * math.sqrt(3) / 2

        previous_top_corner = None

        for i in range(number_of_panels):
            # Corners
            left_corner = Point(side_length * i, 0 * pq.m)
            top_corner = Point(side_length * (i + 0.5), height)
            right_corner = Point(side_length * (i + 1), 0 * pq.m)

            beams[Beam(left_corner, right_corner)] = BeamProperties(self.hss_bottom_chord_set)

            beams[Beam(left_corner, top_corner)] = BeamProperties(self.hss_web_set)
            beams[Beam(right_corner, top_corner)] = BeamProperties(self.hss_web_set)

            if previous_top_corner is not None:
                beams[Beam(previous_top_corner, top_corner)] = BeamProperties(self.hss_top_chord_set)

            previous_top_corner = top_corner

        return beams

    def get_beams_for_howe_bridge(self, number_of_panels: int, span, angle_ratio: float):
        """
        :param number_of_panels: The number of times the sequence repeats
        :param span: The span of the bridge
        :param angle_ratio: The height divided by the width of one panel
        """
        beams = {}

        chord_length = span / number_of_panels
        height = chord_length * angle_ratio

        if number_of_panels % 2 == 1:
            raise Exception("Uneven number of panels. Can't build beams.")

        for i in range(number_of_panels):
            top_left = Point(chord_length * i, height)
            top_right = Point(chord_length * (i + 1), height)
            bottom_left = Point(chord_length * i, 0 * pq.m)
            bottom_right = Point(chord_length * (i + 1), 0 * pq.m)

            beams[Beam(bottom_left, bottom_right)] = BeamProperties(self.hss_bottom_chord_set)

            if i < number_of_panels / 2:
                beams[Beam(bottom_left, top_right)] = BeamProperties(self.hss_web_set)
            else:
                beams[Beam(top_left, bottom_right)] = BeamProperties(self.hss_web_set)

            if i == 0:
                beams[Beam(bottom_right, top_right)] = BeamProperties(self.hss_web_set)
            elif i == number_of_panels - 1:
                pass
            else:
                beams[Beam(top_left, top_right)] = BeamProperties(self.hss_top_chord_set)
                beams[Beam(bottom_right, top_right)] = BeamProperties(self.hss_web_set)

        return beams

    def get_beams_for_pratt_bridge(self, number_of_panels: int, span, angle_ratio: float):
        """
        :param number_of_panels: The number of times the sequence repeats
        :param span: The span of the bridge
        :param angle_ratio: The height divided by the width of one panel
        """
        beams = {}

        chord_length = span / number_of_panels
        height = chord_length * angle_ratio

        if number_of_panels % 2 == 1:
            raise Exception("Uneven number of panels. Can't build beams.")

        for i in range(number_of_panels):
            top_left = Point(chord_length * i, height)
            top_right = Point(chord_length * (i + 1), height)
            bottom_left = Point(chord_length * i, 0 * pq.m)
            bottom_right = Point(chord_length * (i + 1), 0 * pq.m)

            # Bottom chord
            beams[Beam(bottom_left, bottom_right)] = BeamProperties(self.hss_bottom_chord_set)

            # Vertical beam
            if i != 0:
                beams[Beam(bottom_left, top_left)] = BeamProperties(self.hss_web_set)

            # Top chord
            if i != 0 and i != number_of_panels - 1:
                beams[Beam(top_left, top_right)] = BeamProperties(self.hss_top_chord_set)

            # Slanted beams
            if i == 0:
                beams[Beam(bottom_left, top_right)] = BeamProperties(self.hss_web_set)
            elif i < number_of_panels / 2:
                beams[Beam(top_left, bottom_right)] = BeamProperties(self.hss_web_set)
            elif i < number_of_panels - 1:
                beams[Beam(top_right, bottom_left)] = BeamProperties(self.hss_web_set)
            else:
                beams[Beam(top_left, bottom_right)] = BeamProperties(self.hss_web_set)

        return beams

    def get_beams_for_k_bridge(self, number_of_panels: int, span, angle_ratio: float):
        """
        :param number_of_panels: The number of times the sequence repeats
        :param span: The span of the bridge
        :param angle_ratio: The height divided by the width of one panel
        """
        beams = {}

        chord_length = span / number_of_panels
        height = chord_length * angle_ratio

        if number_of_panels % 2 == 1:
            raise Exception("Uneven number of panels. Can't build beams.")

        for i in range(number_of_panels):
            top_left = Point(chord_length * i, height)
            top_right = Point(chord_length * (i + 1), height)
            bottom_left = Point(chord_length * i, 0 * pq.m)
            bottom_right = Point(chord_length * (i + 1), 0 * pq.m)
            mid_left = Point(chord_length * i, height / 2)
            mid_right = Point(chord_length * (i + 1), height / 2)

            # Bottom chord
            beams[Beam(bottom_left, bottom_right)] = BeamProperties(self.hss_bottom_chord_set)

            # Vertical beam
            if i != 0:
                if i != number_of_panels / 2:
                    beams[Beam(bottom_left, mid_left)] = BeamProperties(self.hss_web_set)
                    beams[Beam(mid_left, top_left)] = BeamProperties(self.hss_web_set)
                else:
                    beams[Beam(bottom_left, top_left)] = BeamProperties(self.hss_web_set)

            # Top chord
            if i != 0 and i != number_of_panels - 1:
                beams[Beam(top_left, top_right)] = BeamProperties(self.hss_top_chord_set)

            # Slanted beams
            if i == 0:
                beams[Beam(bottom_left, top_right)] = BeamProperties(self.hss_web_set)
            elif i < number_of_panels / 2:
                beams[Beam(mid_left, top_right)] = BeamProperties(self.hss_web_set)
                beams[Beam(mid_left, bottom_right)] = BeamProperties(self.hss_web_set)
            elif i < number_of_panels - 1:
                beams[Beam(mid_right, top_left)] = BeamProperties(self.hss_web_set)
                beams[Beam(mid_right, bottom_left)] = BeamProperties(self.hss_web_set)
            else:
                beams[Beam(top_left, bottom_right)] = BeamProperties(self.hss_web_set)

        return beams

    @staticmethod
    def add_uniformly_distributed_load(bridge, load_per_unit_length, support_joints: Set[Point]):
        external_forces_y_pos = None

        for support in support_joints:
            external_forces_y_pos = support.y
            break

        load_bearing_joints_x_pos = []
        for joint in bridge.joints:
            if joint.y == external_forces_y_pos:
                load_bearing_joints_x_pos.append(joint.x)

        load_bearing_joints_x_pos = sorted(load_bearing_joints_x_pos)
        supports_x_pos = sorted([support.x for support in support_joints])

        # Loop from left to right throughout the joints
        for i, x_pos in enumerate(load_bearing_joints_x_pos):
            if i == 0:
                dist_to_neighbouring_joints = (load_bearing_joints_x_pos[i + 1] - x_pos)
            elif i < len(load_bearing_joints_x_pos) - 1:
                dist_to_neighbouring_joints = (load_bearing_joints_x_pos[i + 1] - load_bearing_joints_x_pos[i - 1])
            else:
                dist_to_neighbouring_joints = (x_pos - load_bearing_joints_x_pos[i - 1])

            bridge.joint_loads[
                Point(x_pos, external_forces_y_pos)] = Vector(0 * pq.N,
                                                              dist_to_neighbouring_joints / -2
                                                              * load_per_unit_length)

        for i, x_pos in enumerate(supports_x_pos):
            if i == 0:
                dist_to_neighbouring_joints = (supports_x_pos[i + 1] - x_pos)
            elif i < len(supports_x_pos) - 1:
                dist_to_neighbouring_joints = (supports_x_pos[i + 1] - supports_x_pos[i - 1])
            else:
                dist_to_neighbouring_joints = (x_pos - supports_x_pos[i - 1])

            bridge.joint_loads[Point(x_pos, external_forces_y_pos)] += Vector(0 * pq.N,
                                                                              dist_to_neighbouring_joints / 2
                                                                              * load_per_unit_length)

    @staticmethod
    def add_point_load(bridge, loaded_joint: Point, supports, load_force: float):
        """
        Joint load must be applied between two supports
        """

        external_forces_y_pos = None
        for support in supports:
            external_forces_y_pos = support.y
            break

        if external_forces_y_pos != loaded_joint.y:
            raise Exception("Joint load is not horizontally inline with supports.")

        supports_x_pos = sorted([support.x for support in supports])
        for i, x_pos in enumerate(supports_x_pos):
            left_neighbour = None if i == 0 else supports_x_pos[i - 1]
            right_neighbour = None if i == len(supports_x_pos) - 1 else supports_x_pos[i + 1]

            # If outside of neighbours, no weight on me
            if left_neighbour is not None and loaded_joint.x <= left_neighbour:
                continue
            if right_neighbour is not None and loaded_joint.x >= right_neighbour:
                continue

            # Between current joint and right joint
            if loaded_joint.x > x_pos:
                if right_neighbour is None:
                    raise Exception(
                        "Joint load seems to be applied to the right of all supports. Must be between supports")

                reaction_force = load_force * (loaded_joint.x - x_pos) / (right_neighbour - x_pos)
                bridge.joint_loads[Point(x_pos, loaded_joint.y)] = Vector(0 * kN, reaction_force)

            # Between current joint and right joint
            elif loaded_joint.x < x_pos:
                if left_neighbour is None:
                    raise Exception(
                        "Joint load seems to be applied to the left of all supports. Must be between supports.")

                reaction_force = load_force * (x_pos - loaded_joint.x) / (x_pos - left_neighbour)
                bridge.joint_loads[Point(x_pos, loaded_joint.y)] = Vector(0 * kN, reaction_force)

            else:
                bridge.joint_loads[Point(x_pos, loaded_joint.y)] = Vector(0 * kN, load_force)

        bridge.joint_loads[loaded_joint] = Vector(0* kN, -load_force)



class BridgeCalculator:
    def __init__(self, bridge: BridgeData):
        self.bridge = bridge
        self.enclosing_rectangle: Optional[Tuple] = None  # (x_min, y_min, x_max, y_max)
        self.beam_groups: Dict[BeamGroup, BeamGroupProperties] = {}

    def get_enclosing_rectangle(self):
        if self.enclosing_rectangle is not None:
            return self.enclosing_rectangle

        (min_x, min_y, max_x, max_y) = (None, None, None, None)
        for joint in self.bridge.joints.keys():
            min_x = min(joint.x, min_x) if min_x is not None else joint.x
            min_y = min(joint.y, min_y) if min_y is not None else joint.y

            max_x = max(joint.x, max_x) if max_x is not None else joint.x
            max_y = max(joint.y, max_y) if max_y is not None else joint.y

        self.enclosing_rectangle = (min_x, min_y, max_x, max_y)
        return self.enclosing_rectangle

    def calculate_member_forces(self):

        # Loop through every joint repeatedly.
        # At each joint look for member forces + external forces + joint load
        # In each component, verify if there's only one unknown_opposing_joints. Repeat in other component if solved
        # Repeat for other joints
        passes = 0
        calculated_joints = set()
        while passes < len(self.bridge.joints) * 2 and len(calculated_joints) < len(self.bridge.joints):
            for joint, opposing_joints in self.bridge.joints.items():
                if joint in calculated_joints:
                    continue

                sum_of_forces = Vector(0 * pq.N, 0 * pq.N)
                unknown_opposing_joints = []

                # Get forces in members
                for opposing_joint in opposing_joints:
                    direction_vector = Vector.from_a_to_b(joint, opposing_joint)
                    member_force = self.bridge.beams[Beam(joint, opposing_joint)].member_force

                    if member_force is None:
                        unknown_opposing_joints.append(opposing_joint)
                    else:
                        sum_of_forces += direction_vector.get_unit_vector() * member_force

                # Get external forces
                if joint in self.bridge.joint_loads:
                    sum_of_forces += self.bridge.joint_loads[joint]

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
                        self.bridge.beams[Beam(unknown_opposing_joint, joint)].member_force = 0 * pq.N
                    elif sum_of_forces.is_collinear(direction_vector):
                        self.bridge.beams[Beam(unknown_opposing_joint, joint)].member_force = sum_of_forces.cos_theta(
                            direction_vector) * sum_of_forces.get_magnitude() * -1
                    else:
                        print(
                            f"Error. Sum of forces ({sum_of_forces})"
                            f" is not in the same direction as the only unknown beam (dir: {direction_vector})"
                            f" for joint {joint}.")
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
                    self.bridge.beams[Beam(joint, unknown_joint_1)].member_force = force_1
                    self.bridge.beams[Beam(joint, unknown_joint_2)].member_force = force_2

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
                        self.bridge.beams[Beam(joint, unknown_joint_3)].member_force = force_3
                    elif unknown_force_2.is_collinear(unknown_force_3):
                        (force_1, force_2) = BridgeCalculator.solve_for_two_unknowns(sum_of_forces, unknown_force_1,
                                                                                     unknown_force_2)
                        self.bridge.beams[Beam(joint, unknown_joint_1)].member_force = force_1
                    elif unknown_force_3.is_collinear(unknown_force_1):
                        (force_1, force_2) = BridgeCalculator.solve_for_two_unknowns(sum_of_forces, unknown_force_1,
                                                                                     unknown_force_2)
                        self.bridge.beams[Beam(joint, unknown_joint_2)].member_force = force_2

            passes += 1

    def calculate_highest_member_forces(self):
        for beam, beam_property in self.bridge.beams.items():
            current_hss_set = self.bridge.beams[beam].beam_group
            if current_hss_set not in self.beam_groups:
                self.beam_groups[current_hss_set] = BeamGroupProperties()

            if beam_property.member_force is not None:
                if beam_property.member_force > 0:
                    self.beam_groups[current_hss_set].max_tension = max(self.beam_groups[current_hss_set].max_tension,
                                                                        beam_property.member_force)

                elif beam_property.member_force < 0:
                    self.beam_groups[current_hss_set].max_compression = min(
                        self.beam_groups[current_hss_set].max_compression,
                        beam_property.member_force)

    def calculate_min_area_and_i(self):
        for beam_group_property in self.beam_groups.values():
            beam_group_property.min_area = max(beam_group_property.min_area,
                                               max(beam_group_property.max_tension, abs(
                                                   beam_group_property.max_compression)) * FACTOR_OF_SAFETY_YIELDING / YIELD_STRESS)

        for beam, beam_property in self.bridge.beams.items():
            if beam_property.member_force is not None:
                if beam_property.member_force < 0:
                    current_hss_set = self.bridge.beams[beam].beam_group

                    # Compression
                    i = abs(beam_property.member_force) * (beam.get_length() ** 2) * FACTOR_OF_SAFETY_BUCKLING / (
                            YOUNG_MODULUS *
                            (math.pi ** 2))
                    self.beam_groups[current_hss_set].min_i = max(self.beam_groups[current_hss_set].min_i, i)

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
