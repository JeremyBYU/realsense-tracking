#!/usr/bin/env python3
from inspect import cleandoc
import itertools
from functools import partial
import numpy as np
from logging import Logger
import py_cui
from landing.helper.helper_logging import setup_logger, add_tui_handler
from landing.helper.helper_utility import apply_transform
from landing.serial.common import MessageLandingCommand, MessagePoseUpdate

logger = setup_logger(server=True, add_stream_handler=False)


def update_value_label(value, suffix=None, precision=2, prefix=None):
    value_str = "N/A"
    if value is None:
        value_str = "N/A"
    elif isinstance(value, float):
        value_str = f"{value:.{precision}f}"
    elif isinstance(value, int):
        value_str = f"{value}"
    elif isinstance(value, list) or isinstance(value, np.ndarray):
        value_str = " ".join([f"{i:.{precision}f}" for i in value])
    else:
        value_str = f"{value}"

    if suffix != None:
        value_str += f"; {suffix:.1f}"
    if prefix != None:
        value_str = f"{prefix}; {value_str}"

    return value_str

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
            "Extract mesh", 2, 0, column_span=1, command=partial(self.request_integrated, request='integrated_extract'))
        self.find_td_btn = self.root.add_button(
            "Find TP from Integrated Mesh", 2, 1, column_span=1, command=partial(self.request_integrated, request='integrated_touchdown_point'))
        self.send_mesh_td_btn = self.root.add_button(
            "Save Mesh", 2, 2, column_span=1, command=self.send_mesh)

        self.remove_integrated_btn = self.root.add_button(
            "Remove Mesh", 3, 0, column_span=1, command=partial(self.request_integrated, request='integrated_remove'))
        # Test Serial Commands
        self.root.add_button("Send Test Command", 3, 1, column_span=2, command=partial(self.root.show_text_box_popup, title="Enter Command, e.g. pu;10;0.1;0.2;0.3;1;2;3", command=self.send_serial_command_test))


        self.landing_label = self.root.add_label("Landing", 4, 0, column_span=1)
        self.land_single_btn = self.root.add_button(
            "Land On Single Scan TP", 4, 1, column_span=1, row_span=2, command=partial(self.request_land, request='land_single'))
        self.land_integrated_btn = self.root.add_button(
            "Land on Integrated TP", 4, 2, column_span=1, row_span=2, command=partial(self.request_land, request='land_integrated'))

        self.command_status = self.root.add_text_block("Server Status", 0, 3, row_span=2, column_span=3)
        self.live_status = self.root.add_text_block("Live Data", 2, 3, row_span=4, column_span=3)
        # self.misc_status = self.root.add_text_block("Misc Data", 5, 3, row_span=1, column_span=3)

        # self.root.add_label("Integration Active", 2, 3, column_span=1, pady=100, )
        # self.integrated_status = self.root.add_label("N/A", 2, 4, column_span=2)

        self.log_scroll_cell = self.root.add_text_block('Log Output', 6, 0, row_span=4, column_span=6)
        add_tui_handler(self.log_scroll_cell)

        from landing.LandingTUI.LandingServiceMinimal import LandingService
        self.ls = LandingService(config)
        # setups
        self.root.set_on_draw_update_func(self.update_gui)

        self.root.run_on_exit(self.exit_fn)

    def exit_fn(self):
        # pass
        self.ls.save_data()

    def toggle_single(self, on=True):
        self.ls.activate_single_scan_touchdown(on)

    def request_integrated(self, request='integrated_start'):
        self.ls.integration_service_forward(request)

    def send_mesh(self):
        self.ls.send_mesh()

    def request_land(self, request='integrated_start'):
        self.ls.initiate_landing(request)

    def update_gui(self):
        self.counter += 1
        self.update_command_status()
        self.update_live_status()

        self.ls.pub_data()

    def update_command_status(self):
        label_str = f"""
        Single Scan Active:   {self.ls.active_single_scan}
        Integration Active:   {self.ls.active_integration}
        Integration Complete: {self.ls.completed_integration}
        """
        label_str = cleandoc(label_str)
        self.command_status.set_text(label_str)

    def send_serial_command_test(self, command:str):
        logger.info("%s", command)
        commands = command.split(';')
        if len(commands) != 8 and len(commands) != 5:
            logger.error("Incorrect command! Expected 7 or 4 semicolons")
            return
        command_str = commands[0]
        if command_str == "pu":
            ts = int(commands[1])
            floats = [float(float_str) for float_str in commands[2:]]
            msg = MessagePoseUpdate(pose_update=(ts, *floats))
            self.ls.send_serial_msg(msg)
        elif command_str == "lc":
            ts = int(commands[1])
            floats = [float(float_str) for float_str in commands[2:]]
            msg = MessageLandingCommand(landing_command=(ts, *floats))
            self.ls.send_serial_msg(msg)
        else:
            logger.error("Incorrect command. Expceted 'pu' or 'lc'")


    def update_live_status(self):

        single_touchdown_point = None
        single_touchdown_dist = None
        single_touchdown_point_body = None
        if len(self.ls.single_scan_touchdowns) > 0:
            last_touchdown = self.ls.single_scan_touchdowns[-1]
            tp = last_touchdown['touchdown_point']
            if tp is not None:
                single_touchdown_point = tp['point']
                single_touchdown_dist = tp['dist']
                single_touchdown_point_body = apply_transform([single_touchdown_point], np.linalg.inv(last_touchdown['H_body_w_ned']))[0, :3]

        if self.ls.extracted_mesh_message is not None:
            n_vertices = self.ls.extracted_mesh_message.mesh.n_vertices
        else:
            n_vertices = None
        
        if self.ls.integrated_touchdown_point_ned:
            integrated_touchdown_point_body = apply_transform([self.ls.integrated_touchdown_point_ned], np.linalg.inv(self.ls.H_body_w_ned))[0, :3]
            # np.array(self.ls.integrated_touchdown_point_ned) - np.array(self.ls.pose_translation_ned)
        else:
            integrated_touchdown_point_body = None

        label_str = f"""
        Pose NED        Frame: ned;  XYZ: {update_value_label(self.ls.pose_translation_ned)}; RPY: {update_value_label(self.ls.pose_rotation_ned_euler)}
        Single TP       Frame: {self.ls.config['single_scan']['command_frame']}; XYZ: {update_value_label(single_touchdown_point)}; Dist: {update_value_label(single_touchdown_dist)}
                        Frame: body; XYZ: {update_value_label(single_touchdown_point_body)}; Dist: {update_value_label(single_touchdown_dist)}
        Integrated TP   Frame: ned;  XYZ: {update_value_label(self.ls.integrated_touchdown_point_ned)}; Dist: {update_value_label(self.ls.integrated_touchdown_dist_ned)}
                        Frame: body; XYZ: {update_value_label(integrated_touchdown_point_body)}; Dist: {update_value_label(self.ls.integrated_touchdown_dist_ned)}

        MISC:
        Mesh Vertices:           {update_value_label(n_vertices)}
        Polylidar Process Alive:  {self.ls.landing_process.is_alive()}
        """
        label_str = cleandoc(label_str)
        self.live_status.set_text(label_str)


def main(config):
    root = py_cui.PyCUI(10, 6)
    root.set_title("Landing TUI")
    root.set_refresh_timeout(0.1)
    s = App(root, config)
    root.start()




if __name__ == '__main__':
    main()
