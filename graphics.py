import tkinter

from core import BridgeCalculator
from utilities import Vector, Point, PairOfValues, kN, display_float


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
    INFORMATION_X_SPACING = 400
    INFORMATION_LOOP_AFTER = 7

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

        scale_factor_vertical = vertical_span ** -1 * BridgeGUI.HEIGHT_TARGET
        scale_factor_horizontal = horizontal_span ** -1 * BridgeGUI.WIDTH_TARGET

        self.scale_factor = min(scale_factor_horizontal, scale_factor_vertical)

        self.height = (vertical_span * self.scale_factor + 2 * BridgeGUI.CANVAS_PADDING) * 2
        width = horizontal_span * self.scale_factor + 2 * BridgeGUI.CANVAS_PADDING

        self.canvas = tkinter.Canvas(self.top, height=float(self.height), width=float(width))

    def draw_beams(self, bridge: BridgeCalculator):
        for beam, beam_property in bridge.bridge.beams.items():
            self.draw_line(self.scale_point(beam.joint1), self.scale_point(beam.joint2),
                           color=beam_property.beam_group.color)
            if beam_property.member_force is not None:
                label_position = Vector.from_point(self.scale_point(beam.joint2)) + BridgeGUI.flip_vector_on_y(
                    Vector.from_a_to_b(beam.joint2, beam.joint1)) / 2 * self.scale_factor
                self.draw_label(label_position, beam_property.member_force.rescale(kN))

    def draw_external_forces(self, bridge: BridgeCalculator):
        for joint, joint_property in bridge.bridge.joints.items():
            if joint_property.external_force is not None:
                self.draw_vector(joint_property.external_force, joint)

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
        self.draw_label(label_position, vector.get_magnitude().rescale(kN))

    def draw_label(self, position: Vector, content: float):
        tkinter.Label(self.canvas, text=display_float(content)).place(
            x=float(position.x.simplified - BridgeGUI.WIDTH_OF_LABEL),
            y=float(position.y.simplified - BridgeGUI.HEIGHT_OF_LABEL))

    def draw_line(self, a: PairOfValues, b: PairOfValues, arrow=False, color=None):
        self.canvas.create_line(float(a.x.simplified), float(a.y.simplified), float(b.x.simplified),
                                float(b.y.simplified), arrow=tkinter.LAST if arrow else None, fill=color)

    def add_information(self, information: str):
        x_pos = BridgeGUI.CANVAS_PADDING + BridgeGUI.INFORMATION_X_SPACING * (
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
        return Vector(vector.x, vector.y * -1)