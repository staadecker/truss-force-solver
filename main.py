from core import BridgeFactory, BridgeCalculator, BridgeData
from utilities import Point, display_float, kN
import quantities as pq
from graphics import BridgeGUI

if __name__ == "__main__":
    # Bridge constants
    AREA_LOAD = (5 + 1 + 0.75) * kN / pq.m ** 2
    WIDTH = 3.7 * pq.m
    SPAN = 107.96 / 3 * pq.m
    HEIGHT_WIDTH_RATIO = 4 / 3
    NUMBER_OF_PANELS = 8

    bridge_factory = BridgeFactory()
    myBeams = bridge_factory.get_beams_for_k_bridge(NUMBER_OF_PANELS, SPAN, HEIGHT_WIDTH_RATIO)
    beams = bridge_factory.get_beams_for_pratt_bridge(NUMBER_OF_PANELS, SPAN, HEIGHT_WIDTH_RATIO)
    supports = {
        Point(0 * pq.m, 0 * pq.m),
        Point(SPAN, 0 * pq.m)
    }
    myBridge = BridgeData(myBeams, supports, AREA_LOAD, WIDTH)
    # BridgeFactory.add_supports_to_bridge(myBridge, {Point(SPAN/2, 0)})
    # BridgeFactory.add_supports_to_bridge(myBridge, {Point(SPAN / 3, 0), Point(SPAN / 3 * 2, 0)})
    bridge_calculator = BridgeCalculator(myBridge)
    bridge_calculator.calculate_member_forces()
    bridge_calculator.calculate_highest_member_forces()
    bridge_calculator.calculate_min_area_and_i()

    GUI = BridgeGUI()
    GUI.draw_bridge(bridge_calculator)

    GUI.add_information(f"Area Load: {display_float(AREA_LOAD)}")
    GUI.add_information(f"Width: {display_float(WIDTH)}")
    GUI.add_information(f"Span: {display_float(SPAN)}")
    GUI.add_information(f"Height/Width ratio: {display_float(HEIGHT_WIDTH_RATIO)}")

    for beam_group, beam_group_property in bridge_calculator.beam_groups.items():
        GUI.add_information(
            f"{beam_group.name} ({beam_group.color}): Minimum area: {display_float(beam_group_property.min_area.rescale('mm**2'))}")
        GUI.add_information(f"{beam_group.name} ({beam_group.color}): Minimum I: "
                            f"{display_float(beam_group_property.min_i.rescale('mm**4') / 10 ** 6)}*10**6")

    GUI.display()
