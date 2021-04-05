import sys
import numpy as np
from pathlib import Path
import queue
from functools import partial
import time
from landing.helper.helper_logging import logger

from PyQt5.QtWidgets import (QApplication, QGridLayout, QPushButton, QVBoxLayout,
                             QWidget, QFormLayout, QHBoxLayout, QLabel,
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
from PoseMessage_pb2 import PoseMessage
from ImageMessage_pb2 import ImageMessage
from LandingMessage_pb2 import LandingMessage


class Window(QWidget):

    def __init__(self):

        super().__init__()

        self.setup_gui()
        self.active_single_scan = True
        self.active_integration = False
        self.completed_integration = False
        self.pose_translation_ned = [0.0, 0.0, 0.0]
        self.pose_rotation_ned = [0.0, 0.0, 0.0, 1.0]
        self.pose_touchdown_point = [0.0, 0.0, 0.0]
        self.pose_touchdown_dist = 0.0
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
        # self.single_right_status.setText("True" if self.active_single_scan else "False")
        # if self.active_single_scan:
        #     self.single_right_status.setStyleSheet("background-color: lightgreen")
        # else:
        #     self.single_right_status.setStyleSheet("background-color: lightcoral")

    def toggle_single(self, on=False):
        logger.info("Attempting to set single scanning to %s", on)
        request_string = "active" if on else "inactive"
        request_string = bytes(request_string, "ascii")
        _ = self.landing_client.call_method("ActivateSingleScanTouchdown", request_string)

    def setup_gui(self):
        self.setWindowTitle("QGridLayout Example")
        # self.setFixedSize(800, 640)

        self.im_w = 320
        self.im_h = 240
        # Create a QGridLayout instance
        layout = QGridLayout()
        layout.setColumnMinimumWidth(0, 100)
        layout.setColumnMinimumWidth(1, 300)
        layout.setRowMinimumHeight(0, 250)
        layout.setRowMinimumHeight(1, 250)
        layout.setColumnStretch(1, 100)
        layout.setHorizontalSpacing(50)

        # Add widgets to the layout

        ##### BEGIN TOP LEFT #######
        layout_top_left = QVBoxLayout()
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
        self.integrated_label = QLabel("Intregrated")
        self.integrated_stop_button = QPushButton("Stop")

        layout_top_left_integrated.addWidget(self.integrated_start_button)
        layout_top_left_integrated.addWidget(self.integrated_label)
        layout_top_left_integrated.addWidget(self.integrated_stop_button)

        layout_top_left.addWidget(top_left_heading)
        layout_top_left.addLayout(layout_top_left_single)
        layout_top_left.addLayout(layout_top_left_integrated)
        ##### END TOP LEFT #######

        ##### BEGIN TOP RIGHT #######
        layout_top_right = QVBoxLayout()
        # Set Top Left Label Header
        top_right_heading = QLabel("Live Stats")
        font = QFont()
        font.setBold(True)
        top_right_heading.setFont(font)
        top_right_heading.setAlignment(Qt.AlignCenter)

        # Add Single Scan Status
        layout_top_right_single = QHBoxLayout()
        layout_top_right_single.setAlignment(Qt.AlignTop)
        self.single_right_label = QLabel("Single Scan Active")
        self.single_right_status = QLabel("N/A")

        layout_top_right_single.addWidget(self.single_right_label)
        layout_top_right_single.addWidget(self.single_right_status)

        # Add Integrated Status
        layout_top_right_integrated = QHBoxLayout()
        layout_top_right_integrated.setAlignment(Qt.AlignTop)
        self.integrated_right_label = QLabel("Integrated Active")
        self.integrated_right_status = QLabel("N/A")

        layout_top_right_integrated.addWidget(self.integrated_right_label)
        layout_top_right_integrated.addWidget(self.integrated_right_status)

        layout_top_right.addWidget(top_right_heading)
        layout_top_right.addLayout(layout_top_right_single)
        layout_top_right.addLayout(layout_top_right_integrated)
        ##### END TOP LEFT #######

        # Add Top Left (0,0)

        ##### BEGIN BOTTOM LEFT #######
        self.image_holder = QLabel()  # this is an image, but we can set the pixels of the label
        self.image_holder.resize(self.im_h, self.im_w)
        im = np.ones((self.im_h, self.im_w, 3), dtype=np.uint8)
        im.fill(255)
        q_im = QImage(im.data, im.shape[1], im.shape[0], im.shape[1] * 3, QImage.Format_RGB888)
        pix = QPixmap(q_im)
        self.image_holder.setPixmap(pix)
        ##### END BOTTOM LEFT #######

        # Fill the grid
        layout.addLayout(layout_top_left, 0, 0)
        layout.addLayout(layout_top_right, 0, 1)
        layout.addWidget(self.image_holder, 1, 0)

        # Set layout ot the grid
        self.setLayout(layout)

    def callback_pose(self, topic_name, pose: PoseMessage, time_):
        pass
        # print(pose.translation)

    def callback_landing_image(self, topic_name, image: ImageMessage, time_):
        logger.info("New Landing Image, active single scan: %s", self.active_single_scan)
        try:
            if self.active_single_scan:
                self.update_image(image)
        except Exception:
            logger.exception("Error with landing image")

    def callback_landing_message(self, topic_name, landing_message: LandingMessage, time_):
        # Add queues message
        self.active_single_scan = landing_message.active_single_scan
        self.active_integration = landing_message.active_integration
        pose_ned = landing_message.pose_translation_ned
        sig_tp = landing_message.single_touchdown_point
        sig_dist = landing_message.single_touchdown_dist

        self.pose_translation_ned = [pose_ned.x, pose_ned.y, pose_ned.z]
        self.single_touchdown_point = [sig_tp.x, sig_tp.y, sig_tp.z]

        # self.image_queue.put(('labels', None))

    def callback_rgbd_image(self, topic_name, image: ImageMessage, time_):
        if not self.active_single_scan:
            self.update_image(image)

    def update_image(self, image: ImageMessage):
        im = np.frombuffer(image.image_data, dtype=np.uint8).reshape((image.height, image.width, 3))
        im = im[..., ::-1].copy()
        self.image_queue.put(('image', im))

    def landing_client_resp_callback(self, service_info, response):
        response = response.decode("utf-8")
        print(service_info)
        print(response)

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

        # create subscriber for depth information and connect callback
        self.sub_landing_message = ProtoSubscriber("LandingMessage", LandingMessage)
        self.sub_landing_message.set_callback(self.callback_landing_message)


if __name__ == "__main__":

    app = QApplication(sys.argv)
    window = Window()
    window.show()
    sys.exit(app.exec_())
