import sys
import numpy as np
from pathlib import Path
import queue
from functools import partial
import time
from landing.helper.helper_logging import setup_logger
logger = setup_logger(server=False)
import open3d as o3d
from scipy.spatial.transform import Rotation as R

from PyQt5.QtWidgets import (QApplication, QGridLayout, QPushButton, QVBoxLayout,
                             QWidget, QFormLayout, QHBoxLayout, QLabel, QMessageBox
                             )
from PyQt5.QtCore import Qt, QObject, QRunnable, pyqtSignal, pyqtSlot, QTimer
from PyQt5.QtGui import QImage, QPixmap, QFont

import ecal.core.core as ecal_core
from ecal.core.service import Client
from ecal.core.subscriber import ProtoSubscriber


THIS_FILE = Path(__file__)
THIS_DIR = THIS_FILE.parent

build_dir = THIS_DIR.parent / f"dk-x86_64-build"
sys.path.insert(1, str(build_dir))
build_dir = THIS_DIR.parent / f"cmake-build"
sys.path.insert(1, str(build_dir))
from PoseMessage_pb2 import PoseMessage
from ImageMessage_pb2 import ImageMessage
from LandingMessage_pb2 import LandingMessage
from Integrate_pb2 import Mesh
from TouchdownMessage_pb2 import TouchdownMessage, SINGLE, INTEGRATED, MeshAndTouchdownMessage

from landing.helper.helper_utility import get_mesh_data_from_message, convert_polygon_message_to_shapely
from landing.helper.helper_meshes import create_o3d_mesh_from_data
from landing.helper.o3d_util import create_linemesh_from_shapely, get_segments


class Window(QWidget):

    def __init__(self):

        super().__init__()

        self.setup_gui()
        self.pose_translation_ned = None
        self.pose_rotation_ned = None

        # Single Scan Variables
        self.active_single_scan = True
        self.single_touchdown_point = None
        self.single_touchdown_dist = None

        # Integration Variables
        self.active_integration = False
        self.completed_integration = False
        self.extracted_mesh_vertices = 0
        self.integrated_touchdown_point = None
        self.integrated_touchdown_dist = None
        self.integrated_polygon = None
        self.o3d_mesh = None

        # Set up ECAL
        self.setup_ecal()

        self.image_queue = queue.Queue()
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_gui_state)
        self.timer.start(100)

    def update_status_label(self, status_label, active=True):
        status_label.setText("True" if active else "False")
        if active:
            status_label.setStyleSheet("background-color: lightgreen")
        else:
            status_label.setStyleSheet("background-color: lightcoral")

    def update_value_label(self, value_label, value, suffix=None, precision=2, prefix=None):
        value_str = "N/A"
        if value is None:
            value_str = "N/A"
        elif isinstance(value, float):
            value_str = f"{value:.{precision}f}"
        elif isinstance(value, int):
            value_str = f"{value}"
        elif isinstance(value, list):
            value_str = " ".join([f"{i:.{precision}f}" for i in value])
        else:
            value_str = f"{value}"

        if suffix != None:
            value_str += f"; {suffix:.1f}"
        if prefix != None:
            value_str = f"{prefix}; {value_str}"

        value_label.setText(value_str)

    def update_gui_state(self):
        try:
            update_type, data = self.image_queue.get(block=False)
            if update_type == 'image':
                im = data
                q_im = QImage(im.data, im.shape[1], im.shape[0], im.shape[1] * 3, QImage.Format_RGB888)
                pix = QPixmap(q_im)
                self.image_holder.setPixmap(pix)
        except queue.Empty:
            pass
        except Exception:
            logger.exception("Error reading image")
        # This should be safe to update at all times
        self.update_status_label(self.single_right_status, self.active_single_scan)
        self.update_status_label(self.integrated_right_status, self.active_integration)
        self.update_status_label(self.integrated_right_complete_status, self.completed_integration)

        self.update_value_label(self.pose_ned_status, self.pose_translation_ned)
        self.update_value_label(self.pose_ned_status, self.pose_rotation_ned, prefix=self.pose_ned_status.text(), precision=1)

        self.update_value_label(self.single_tp_status, self.single_touchdown_point, suffix=self.single_touchdown_dist)
        self.update_value_label(self.integrated_tp_status, self.integrated_touchdown_point,
                                suffix=self.integrated_touchdown_dist)
        self.update_value_label(self.mesh_vertices_status, self.extracted_mesh_vertices)

        # Enable/Disable buttons based on state
        if self.completed_integration:
            self.extract_mesh_button.setEnabled(True)
        else:
            self.extract_mesh_button.setEnabled(False)

        if self.extracted_mesh_vertices > 0:
            self.touchdown_mesh_button.setEnabled(True)
            self.show_mesh_button.setEnabled(True)
        else:
            self.touchdown_mesh_button.setEnabled(False)
            self.show_mesh_button.setEnabled(False)


        if self.single_touchdown_dist is not None and self.single_touchdown_dist > 0.1:
            self.land_single_button.setEnabled(True)
        else:
            self.land_single_button.setEnabled(False)

        if self.integrated_touchdown_dist is not None and self.integrated_touchdown_dist > 0.1:
            self.land_integrated_button.setEnabled(True)
        else:
            self.land_integrated_button.setEnabled(False)


    def toggle_single(self, on=False):
        logger.info("Attempting to set single scanning to %s", on)
        request_string = "active" if on else "inactive"
        request_string = bytes(request_string, "ascii")
        _ = self.landing_client.call_method("ActivateSingleScanTouchdown", request_string)

    def integration_request(self, request_type='start'):
        logger.info("Attempting to make Landing Server request of %s", request_type)
        request_string = bytes(request_type, "ascii")
        if request_type == "send_mesh":
            _ = self.landing_client.call_method("SendMesh", request_string)
        else:
            _ = self.landing_client.call_method("IntegrationServiceForward", request_string)

    def land_request(self, request_type='land_single'):
        logger.info("Attempting to make Landing Server request of %s", request_type)
        request_string = bytes(request_type, "ascii")
        _ = self.landing_client.call_method("InitiateLanding", request_string)

    def show_mesh(self):
        if self.o3d_mesh is not None and len(self.o3d_mesh.triangles) > 0 :
            axis_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(0.2)
            geoms = []
            if self.integrated_polygon:
                line_meshes = create_linemesh_from_shapely(self.integrated_polygon)
                geoms.extend(get_segments(line_meshes))
            
            if self.integrated_touchdown_point is not None and self.integrated_touchdown_point[0] != 0.0:
                tp = o3d.geometry.TriangleMesh.create_icosahedron(0.05).translate(self.integrated_touchdown_point)
                tp.paint_uniform_color([0, 0.0, 1])
                geoms.append(tp)
            o3d.visualization.draw_geometries([self.o3d_mesh, axis_frame, *geoms], width=800, height=600)
        else:
            msg = QMessageBox()
            msg.setWindowTitle("ERROR!")
            msg.setIcon(QMessageBox.Critical)
            msg.setText(
                "Whooops! I don't have the mesh anymore on the client. However the server still does. Click 'ExtractMesh' Button for me to receive it again.   ")
            x = msg.exec_()  # this will show our messagebox


    def setup_gui(self):
        self.setWindowTitle("Landing Client")
        # self.setFixedSize(800, 640)

        self.im_w = 320
        self.im_h = 240
        # Create a QGridLayout instance
        layout = QGridLayout()
        layout.setColumnMinimumWidth(0, 100)
        layout.setColumnMinimumWidth(1, 450)
        layout.setRowMinimumHeight(0, 100)
        layout.setRowMinimumHeight(1, 100)
        layout.setColumnStretch(1, 100)
        layout.setHorizontalSpacing(50)

        # Add widgets to the layout

        ##### BEGIN TOP LEFT #######
        layout_top_left = QVBoxLayout()
        layout_top_left.setSpacing(5)
        # Set Top Left Label Header
        top_left_heading = QLabel("Commands")
        font = QFont()
        font.setBold(True)
        top_left_heading.setFont(font)
        top_left_heading.setAlignment(Qt.AlignCenter)

        # Add Single Scan
        layout_top_left_single = QHBoxLayout()
        layout_top_left_single.setAlignment(Qt.AlignTop)
        self.single_start_button = QPushButton("Start")
        self.single_start_button.clicked.connect(partial(self.toggle_single, on=True))
        self.single_label = QLabel("Single Scan")
        # self.single_label.setAlignment(Qt.AlignCenter) # no work
        self.single_stop_button = QPushButton("Stop")
        self.single_stop_button.clicked.connect(partial(self.toggle_single, on=False))

        layout_top_left_single.addWidget(self.single_start_button)
        layout_top_left_single.addWidget(self.single_label)
        layout_top_left_single.addWidget(self.single_stop_button)

        # Add Integrated
        layout_top_left_integrated = QHBoxLayout()
        layout_top_left_integrated.setAlignment(Qt.AlignTop)
        self.integrated_start_button = QPushButton("Start")
        self.integrated_start_button.clicked.connect(partial(self.integration_request, request_type='integrated_start'))
        self.integrated_label = QLabel("Intregrated")
        self.integrated_stop_button = QPushButton("Stop")
        self.integrated_stop_button.clicked.connect(partial(self.integration_request, request_type='integrated_stop'))

        layout_top_left_integrated.addWidget(self.integrated_start_button)
        layout_top_left_integrated.addWidget(self.integrated_label)
        layout_top_left_integrated.addWidget(self.integrated_stop_button)

        # Add Mesh Extraction
        layout_top_left_mesh_extraction = QHBoxLayout()
        layout_top_left_mesh_extraction.setAlignment(Qt.AlignTop)
        self.extract_mesh_button = QPushButton("Extract Mesh from Integration")
        self.extract_mesh_button.clicked.connect(partial(self.integration_request, request_type='integrated_extract'))
        self.extract_mesh_button.setEnabled(False)
        self.touchdown_mesh_button = QPushButton("Find TP from Integrated Mesh")
        self.touchdown_mesh_button.clicked.connect(
            partial(self.integration_request, request_type='integrated_touchdown_point'))
        self.touchdown_mesh_button.setEnabled(False)

        layout_top_left_mesh_extraction.addWidget(self.extract_mesh_button)
        layout_top_left_mesh_extraction.addWidget(self.touchdown_mesh_button)

        # Add Landing Commands!
        layout_top_left_landing_commands = QHBoxLayout()
        layout_top_left_landing_commands.setAlignment(Qt.AlignTop)
        self.land_single_button = QPushButton("Land on Single Scan TP")
        self.land_single_button.clicked.connect(partial(self.land_request, request_type='land_single'))
        self.land_single_button.setEnabled(False)
        self.land_integrated_button = QPushButton("Land on Integrated TP")
        self.land_integrated_button.clicked.connect(partial(self.land_request, request_type='land_integrated'))
        self.land_integrated_button.setEnabled(False)

        layout_top_left_landing_commands.addWidget(self.land_single_button)
        layout_top_left_landing_commands.addWidget(self.land_integrated_button)

        layout_top_left.addWidget(top_left_heading)
        layout_top_left.addLayout(layout_top_left_single)
        layout_top_left.addLayout(layout_top_left_integrated)
        layout_top_left.addLayout(layout_top_left_mesh_extraction)
        layout_top_left.addLayout(layout_top_left_landing_commands)

        layout_top_left.addStretch()
        ##### END TOP LEFT #######

        ##### BEGIN TOP RIGHT #######
        layout_top_right = QVBoxLayout()
        top_right_heading = self.create_header(text="Command Status")

        # Add Single Scan Status
        layout_top_right_single = self.simple_status_widgets('single_right', 'Single Scan Active', 'N/A')
        # Add Integrated Status
        layout_top_right_integrated = self.simple_status_widgets('integrated_right', 'Integration Active', 'N/A')
        # Add Integrated Complete
        layout_top_right_integrated_complete = self.simple_status_widgets(
            'integrated_right_complete', 'Integration Complete', 'N/A')

        # Top Middle
        mid_right_heading = self.create_header(text="Live Status")
        layout_top_right_pose_ned = self.simple_status_widgets('pose_ned', 'Pose NED (XYZRPY)', 'N/A')
        layout_top_right_single_tp = self.simple_status_widgets('single_tp', 'Single TP', 'N/A')
        layout_top_right_integrated_tp = self.simple_status_widgets('integrated_tp', 'Integrated TP', 'N/A')

        # Right Bottom (Mesh Information)
        bottom_right_heading = self.create_header(text="Integrated Mesh Data")
        # Add Extracted Vertices
        layout_bottom_right_mesh_vertices = self.simple_status_widgets('mesh_vertices', '# Mesh Vertices', 'N/A')
        # Button Layout
        layout_bottom_right_mesh_display = QHBoxLayout()
        layout_bottom_right_mesh_display.setAlignment(Qt.AlignTop)
        # Add Receive Mesh Button
        self.receive_mesh_button = QPushButton("Receive Mesh from Server")
        self.receive_mesh_button.clicked.connect(partial(self.integration_request, request_type='send_mesh'))
        self.receive_mesh_button.setEnabled(True)
        # Add Show Mesh Button
        self.show_mesh_button = QPushButton("Show Mesh from Server")
        self.show_mesh_button.clicked.connect(self.show_mesh)
        self.show_mesh_button.setEnabled(False)
        layout_bottom_right_mesh_display.addWidget(self.receive_mesh_button)
        layout_bottom_right_mesh_display.addWidget(self.show_mesh_button)


        layout_top_right.addWidget(top_right_heading)
        layout_top_right.addLayout(layout_top_right_single)
        layout_top_right.addLayout(layout_top_right_integrated)
        layout_top_right.addLayout(layout_top_right_integrated_complete)
        layout_top_right.addSpacing(20)

        layout_top_right.addWidget(mid_right_heading)
        layout_top_right.addLayout(layout_top_right_pose_ned)
        layout_top_right.addLayout(layout_top_right_single_tp)
        layout_top_right.addLayout(layout_top_right_integrated_tp)
        layout_top_right.addSpacing(20)

        layout_top_right.addWidget(bottom_right_heading)
        layout_top_right.addLayout(layout_bottom_right_mesh_vertices)
        layout_top_right.addLayout(layout_bottom_right_mesh_display)

        ##### END TOP RIGHT #######

        # Add Top Left (0,0)

        ##### BEGIN BOTTOM LEFT #######
        self.image_holder = QLabel()  # this is an image, but we can set the pixels of the label
        self.image_holder.resize(self.im_h, self.im_w)
        self.image_holder.setAlignment(Qt.AlignCenter)
        im = np.ones((self.im_h, self.im_w, 3), dtype=np.uint8)
        im.fill(255)
        q_im = QImage(im.data, im.shape[1], im.shape[0], im.shape[1] * 3, QImage.Format_RGB888)
        pix = QPixmap(q_im)
        self.image_holder.setPixmap(pix)
        ##### END BOTTOM LEFT #######

        # Fill the grid
        layout.addLayout(layout_top_left, 0, 0)
        layout.addLayout(layout_top_right, 0, 1, 2, 1)
        layout.addWidget(self.image_holder, 1, 0)

        # Set layout ot the grid
        self.setLayout(layout)

    def simple_status_widgets(self, name, label_text, value_text, left_suffix="_label", right_suffix="_status"):
        layout = QHBoxLayout()
        layout.setAlignment(Qt.AlignTop)
        setattr(self, name + left_suffix, QLabel(label_text))
        setattr(self, name + right_suffix, QLabel(value_text))
        layout.addWidget(getattr(self, name + left_suffix))
        layout.addWidget(getattr(self, name + right_suffix))
        return layout

    def create_header(self, text="Live Stats"):
        top_right_heading = QLabel(text)
        font = QFont()
        font.setBold(True)
        top_right_heading.setFont(font)
        top_right_heading.setAlignment(Qt.AlignCenter)
        return top_right_heading

    def callback_pose(self, topic_name, pose: PoseMessage, time_):
        pass
        # print(pose.translation)

    # def callback_mesh_message(self, topic_name, mesh: Mesh, time_):
    #     logger.info("New Mesh Message, vertices %d", mesh.n_vertices)
    #     try:
    #         raw_data = get_mesh_data_from_message(mesh)
    #         self.o3d_mesh = create_o3d_mesh_from_data(*raw_data)

    #         # o3d_mesh = create_o3d_mesh_from_data(*raw_data)
    #         # logger.info("Triangle Size: %d", len(o3d_mesh.triangles))
    #         # if len(o3d_mesh.triangles) > 0:
    #         #     o3d.visualization.draw_geometries([o3d_mesh])
    #     except Exception:
    #         logger.exception("Error with mesh message")

    def callback_landing_image(self, topic_name, image: ImageMessage, time_):
        logger.debug("New Landing Image, active single scan: %s", self.active_single_scan)
        try:
            if self.active_single_scan:
                self.update_image(image)
        except Exception:
            logger.exception("Error with landing image")

    def callback_landing_message(self, topic_name, landing_message: LandingMessage, time_):
        try:
            # Pose Updates
            pose_ned = landing_message.pose_translation_ned
            rot_ned = landing_message.pose_rotation_ned
            pose_t265 = landing_message.pose_translation_t265
            rot_t265 = landing_message.pose_rotation_ned
            self.pose_translation_ned = [pose_ned.x, pose_ned.y, pose_ned.z]
            self.pose_translation_t265 = [pose_t265.x, pose_t265.y, pose_t265.z]
            if rot_ned.w > 0:
                self.pose_rotation_ned = [rot_ned.x, rot_ned.y, rot_ned.z, rot_ned.w]
                self.pose_rotation_ned = R.from_quat(self.pose_rotation_ned).as_euler('xyz', degrees=True).flatten().tolist()
                self.pose_rotation_t265 = [rot_t265.x, rot_t265.y, rot_t265.z, rot_t265.w]
                self.pose_rotation_t265 = R.from_quat(self.pose_rotation_t265).as_euler('xyz', degrees=True).flatten().tolist()

            # Single Scan Updates
            sig_tp = landing_message.single_touchdown_point
            self.active_single_scan = landing_message.active_single_scan
            self.single_touchdown_point = [sig_tp.x, sig_tp.y, sig_tp.z]
            self.single_touchdown_dist = landing_message.single_touchdown_dist

            # Integration Updates
            int_tp = landing_message.integrated_touchdown_point
            self.active_integration = landing_message.active_integration
            self.completed_integration = landing_message.completed_integration
            self.extracted_mesh_vertices = landing_message.extracted_mesh_vertices
            self.integrated_touchdown_point = [int_tp.x, int_tp.y, int_tp.z]
            self.integrated_touchdown_dist = landing_message.integrated_touchdown_dist
        except:
            logger.exception("Error")


    def callback_rgbd_image(self, topic_name, image: ImageMessage, time_):
        if not self.active_single_scan:
            self.update_image(image)

    # def callback_touchdown_message(self, topic_name, touchdown_message: TouchdownMessage, time_):
    #     if touchdown_message.type == INTEGRATED:
    #         logger.info("Received Interated Touchdown Message")
    #         if touchdown_message.landing_site:
    #             self.integrated_polygon = convert_polygon_message_to_shapely(touchdown_message.landing_site)
            


    def update_image(self, image: ImageMessage):
        im = np.frombuffer(image.image_data, dtype=np.uint8).reshape((image.height, image.width, 3))
        im = im[..., ::-1].copy()
        self.image_queue.put(('image', im))

    def landing_client_resp_callback(self, service_info, response):
        logger.info("Received Callback form server: %s", service_info)
        if service_info['ret_state'] == 0 and service_info['method_name'] == 'SendMesh':
            mesh_td = MeshAndTouchdownMessage()
            mesh_td.ParseFromString(response)
            try:
                raw_data = get_mesh_data_from_message(mesh_td.mesh)
                self.o3d_mesh = create_o3d_mesh_from_data(*raw_data)
                if mesh_td.HasField('touchdown'):
                    logger.info("Has Touchdown in message, extracting it as well...")
                    self.integrated_polygon = convert_polygon_message_to_shapely(mesh_td.touchdown.landing_site)
                
            except Exception:
                logger.exception("Error with mesh message")



    def setup_ecal(self):
        ecal_core.initialize(sys.argv, "Landing_Client")
        # set process state
        ecal_core.set_process_state(1, 1, "Healthy")

        self.landing_client = Client("LandingService")
        # and add it to the client
        self.landing_client.add_response_callback(self.landing_client_resp_callback)

        # create subscriber for pose information and connect callback
        self.sub_pose = ProtoSubscriber("PoseMessage", PoseMessage)
        self.sub_pose.set_callback(self.callback_pose)

        # create subscriber for depth information and connect callback
        self.sub_rgb_image = ProtoSubscriber("RGBDMessage", ImageMessage)
        self.sub_rgb_image.set_callback(self.callback_rgbd_image)

        # create subscriber for depth information and connect callback
        self.sub_landing_image = ProtoSubscriber("RGBDLandingMessage", ImageMessage)
        self.sub_landing_image.set_callback(self.callback_landing_image)

        # create subscriber for Landing Message information and connect callback
        self.sub_landing_message = ProtoSubscriber("LandingMessage", LandingMessage)
        self.sub_landing_message.set_callback(self.callback_landing_message)


        # NOT reliable with UDP connection, moved to service
        # create subscriber for Mesh Message and connect callback
        # self.sub_mesh_message = ProtoSubscriber("MeshMessage", Mesh)
        # self.sub_mesh_message.set_callback(self.callback_mesh_message)

        # NOT reliable with UDP connection, moved to service
        # # create subscriber for Touchdown Message and connect callback
        # # currently only using this to get plot polygons for integrated meshes
        # self.sub_touchdown_message = ProtoSubscriber("TouchdownMessage", TouchdownMessage)
        # self.sub_touchdown_message.set_callback(self.callback_touchdown_message)


if __name__ == "__main__":

    app = QApplication(sys.argv)
    window = Window()
    window.show()
    sys.exit(app.exec_())
