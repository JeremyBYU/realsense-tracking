import sys
import numpy as np
from pathlib import Path

from PyQt5.QtWidgets import (QApplication, QGridLayout, QPushButton, QVBoxLayout,
                             QWidget, QFormLayout, QHBoxLayout, QLabel,
                             )
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QImage, QPixmap

import ecal.core.core as ecal_core
import ecal.core.service as ecal_service
from ecal.core.subscriber import ProtoSubscriber


THIS_FILE = Path(__file__)
THIS_DIR = THIS_FILE.parent
# CONFIG_DIR = THIS_DIR.parent / "config" / "landing"
# LANDING_CONFIG_FILE = CONFIG_DIR / "landing.yml"

build_dir = THIS_DIR.parent / f"dk-x86_64-build"
sys.path.insert(1, str(build_dir))
from PoseMessage_pb2 import PoseMessage
from ImageMessage_pb2 import ImageMessage






class Window(QWidget):

    def __init__(self):

        super().__init__()
        self.setWindowTitle("QGridLayout Example")
        # self.setFixedSize(800, 640)

        self.im_w = 320
        self.im_h = 240
        # Create a QGridLayout instance
        layout = QGridLayout()
        layout.setColumnMinimumWidth(0, 300)
        layout.setColumnMinimumWidth(1, 300)
        layout.setRowMinimumHeight(0, 250)
        layout.setRowMinimumHeight(1, 250)

        # Add widgets to the layout
        layout_top_left = QVBoxLayout()

        layout_top_left_single = QHBoxLayout()
        layout_top_left_single.setAlignment(Qt.AlignTop)
        self.single_start_button = QPushButton("Start")
        self.single_label = QLabel("Single Scan")
        self.single_stop_button = QPushButton("Stop")

        layout_top_left_single.addWidget(self.single_start_button)
        layout_top_left_single.addWidget(self.single_label)
        layout_top_left_single.addWidget(self.single_stop_button)

        layout_top_left_integrated = QHBoxLayout()
        layout_top_left_integrated.setAlignment(Qt.AlignTop)
        self.integrated_start_button = QPushButton("Start")
        self.integrated_label = QLabel("Intregrated")
        self.integrated_stop_button = QPushButton("Stop")

        layout_top_left_integrated.addWidget(self.integrated_start_button)
        layout_top_left_integrated.addWidget(self.integrated_label)
        layout_top_left_integrated.addWidget(self.integrated_stop_button)

        layout_top_left.addLayout(layout_top_left_single)
        layout_top_left.addLayout(layout_top_left_integrated)

        # Add Top Left
        layout.addLayout(layout_top_left, 0, 0)

        self.image_holder = QLabel()
        self.image_holder.resize(self.im_h, self.im_w)
        im = np.ones((self.im_h, self.im_w, 3), dtype=np.uint8)
        im.fill(255)
        q_im = QImage(im.data, im.shape[1], im.shape[0], im.shape[1] * 3, QImage.Format_RGB888)
        pix = QPixmap(q_im)
        self.image_holder.setPixmap(pix)

        layout.addWidget(self.image_holder, 1, 0)
        self.setLayout(layout)

        self.setup_ecal()


    def callback_pose(self, topic_name, pose: PoseMessage, time_):
        print(pose.translation)
 
    def callback_depth(self, topic_name, image: ImageMessage, time_):
        image_color_np = np.frombuffer(image.image_data, dtype=np.uint8).reshape((image.height, image.width, 3))
        print(image_color_np)

    def setup_ecal(self):
        ecal_core.initialize(sys.argv, "Landing_Client")
        # set process state
        ecal_core.set_process_state(1, 1, "Healthy")


        # create subscriber for pose information and connect callback
        self.sub_pose = ProtoSubscriber("PoseMessage", PoseMessage)
        self.sub_pose.set_callback(self.callback_pose)

        # create subscriber for depth information and connect callback
        self.sub_depth = ProtoSubscriber("RGBDMessage", ImageMessage)
        self.sub_depth.set_callback(self.callback_depth)


if __name__ == "__main__":

    app = QApplication(sys.argv)
    window = Window()
    window.show()
    sys.exit(app.exec_())
