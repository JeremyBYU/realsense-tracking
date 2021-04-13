#!/usr/bin/env python3
from inspect import cleandoc
import itertools
from functools import partial
from logging import Logger
import py_cui
from landing.helper.helper_logging import setup_logger, add_tui_handler

logger = setup_logger(server=True, add_stream_handler=False)


class App:
    character_gen = itertools.cycle(("X", "-", "█", "[", "#"))

    def __init__(self, root_: py_cui.PyCUI, config):
        self.root = root_
        self.counter = 0
        self.config = config


        # Default configuration
        # self.default = self.root.add_slider("Default", 0, 0, column_span=2, min_val=-50, max_val=50)
        self.single_scan_label = self.root.add_label("Single Scan", 0, 0, column_span=1)
        self.activate_single_scan_btn = self.root.add_button(
            "Start", 0, 1, column_span=1, command=partial(self.toggle_single, on=True))
        self.deactivate_single_scan_btn = self.root.add_button(
            "Stop", 0, 2, column_span=1, command=partial(self.toggle_single, on=False))

        self.integrated_scan_label = self.root.add_label("Integration", 1, 0, column_span=1)
        self.start_integrated_btn = self.root.add_button(
            "Start", 1, 1, column_span=1, command=partial(self.request_integrated, request='integrated_start'))
        self.stop_integrated_btn = self.root.add_button(
            "Stop", 1, 2, column_span=1, command=partial(self.request_integrated, request='integrated_stop'))

        self.extract_mesh_btn = self.root.add_button(
            "Extract mesh", 2, 1, column_span=1, command=partial(self.request_integrated, request='integrated_extract'))
        self.find_td_btn = self.root.add_button(
            "Find TP from Integrated Mesh", 2, 2, column_span=1, command=partial(self.request_integrated, request='integrated_touchdown_point'))

        self.root.add_label("", 3, 0, column_span=3)
        self.landing_label = self.root.add_label("Landing", 4, 0, column_span=1)
        self.land_single_btn = self.root.add_button(
            "Land On Single Scan TP", 4, 1, column_span=1, row_span=2, command=partial(self.request_land, request='land_single'))
        self.land_integrated_btn = self.root.add_button(
            "Land on Integrated TP", 4, 2, column_span=1, row_span=2, command=partial(self.request_land, request='land_integrated'))

        
        self.command_status = self.root.add_text_block("Command Status", 0, 3, row_span=2, column_span=3)
        self.live_status = self.root.add_text_block("Live Status", 2, 3, row_span=2, column_span=3)
        self.mesh_status = self.root.add_text_block("Misc Data", 4, 3, row_span=2, column_span=3)

        # self.root.add_label("Integration Active", 2, 3, column_span=1, pady=100, )
        # self.integrated_status = self.root.add_label("N/A", 2, 4, column_span=2)

        self.log_scroll_cell = self.root.add_text_block('Log Output', 6, 0, row_span=4, column_span=6)
        add_tui_handler(self.log_scroll_cell)

        from landing.LandingTUI.LandingServiceMinimal import LandingService
        self.ls = LandingService(config)
        # setups
        self.root.set_on_draw_update_func(self.set_step)
        logger.info("Test")


    def toggle_single(self, on=True):
        logger.info("Toggling Single %s", on)
        self.ls.activate_single_scan_touchdown(on)

    def request_integrated(self, request='integrated_start'):
        logger.info("Requesting Integration: %s", request)
        self.ls.integration_service_forward(request)

    def request_land(self, request='integrated_start'):
        logger.info("Requesting Landing: %s", request)
        self.ls.initiate_landing(request)

    def set_step(self):
        self.counter += 1
        # logger.info(f"Yo {self.counter}")


def main(config):
    root = py_cui.PyCUI(10, 6)
    root.set_title("Landing TUI")
    root.set_refresh_timeout(1)
    s = App(root, config)
    root.start()

    # from landing.LandingTUI.LandingServiceMinimal import LandingService


if __name__ == '__main__':
    main()
