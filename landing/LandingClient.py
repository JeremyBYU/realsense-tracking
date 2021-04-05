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




# class WorkerSignals(QObject):
#     '''
#     Defines the signals available from a running worker thread.

#     Supported signals are:

#     finished
#         No data

#     error
#         tuple (exctype, value, traceback.format_exc() )

#     result
#         object data returned from processing, anything

#     progress
#         int indicating % progress

#     '''
#     finished = pyqtSignal()
#     progress = pyqtSignal(int)


# class Worker(QRunnable):
#     '''
#     Worker thread

#     Inherits from QRunnable to handler worker thread setup, signals and wrap-up.

#     :param callback: The function callback to run on this worker thread. Supplied args and
#                      kwargs will be passed through to the runner.
#     :type callback: function
#     :param args: Arguments to pass to the callback function
#     :param kwargs: Keywords to pass to the callback function

#     '''

#     def __init__(self, fn, *args, **kwargs):
#         super(Worker, self).__init__()

#         # Store constructor arguments (re-used for processing)
#         self.fn = fn
#         self.args = args
#         self.kwargs = kwargs
#         self.signals = WorkerSignals()

#         # Add the callback to our kwargs
#         self.kwargs['progress_callback'] = self.signals.progress

#     @pyqtSlot()
#     def run(self):
#         '''
#         Initialise the runner function with passed args, kwargs.
#         '''

#         # Retrieve args/kwargs here; and fire processing using them
#         try:
#             result = self.fn(*self.args, **self.kwargs)
#             self.signals.finished.emit()  # Done
#         except:
#             logger.exception("Error")

class Window(QWidget):

    def __init__(self):

        super().__init__()

        self.setup_gui()
        self.active_single_scan = False
        # Set up ECAL
        self.setup_ecal()

        self.image_queue = queue.Queue()
        self.timer=QTimer()
        self.timer.timeout.connect(self.check_for_image)
        self.timer.start(100)
        
        # worker = Worker(self.execute_this_fn, self.image_queue) # Any other args, kwargs are passed to the run function
        # worker.signals.result.connect(self.print_output)
        # worker.signals.finished.connect(self.thread_complete)
        # worker.signals.progress.connect(self.progress_fn)

        # Execute
        # self.threadpool.start(worker)
    def check_for_image(self):
        try:
            im = self.image_queue.get(block=False)
            q_im = QImage(im.data, im.shape[1], im.shape[0], im.shape[1] * 3, QImage.Format_RGB888)
            pix = QPixmap(q_im)
            self.image_holder.setPixmap(pix)
        except queue.Empty:
            pass
        except Exception:
            logger.exception("Error reading image")

    # def progress_fn(self, n):
    #     logger.info("Progress %d", n)

    # def execute_this_fn(self, queue, progress_callback):
    #     print(queue)
    #     print(progress_callback)
    #     for n in range(0, 5):
    #         time.sleep(1)
    #         progress_callback.emit(n)

    #     return "Done."

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

        ## Add Single Scan
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

         ## Add Integrated
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

        ## Add Single Scan Status
        layout_top_right_single = QHBoxLayout()
        layout_top_right_single.setAlignment(Qt.AlignTop)
        self.single_right_label = QLabel("Single Scan Active")
        self.single_right_status = QLabel("N/A")

        layout_top_right_single.addWidget(self.single_right_label)
        layout_top_right_single.addWidget(self.single_right_status)

         ## Add Integrated Status
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
        self.image_holder = QLabel() # this is an image, but we can set the pixels of the label
        self.image_holder.resize(self.im_h, self.im_w)
        im = np.ones((self.im_h, self.im_w, 3), dtype=np.uint8)
        im.fill(255)
        q_im = QImage(im.data, im.shape[1], im.shape[0], im.shape[1] * 3, QImage.Format_RGB888)
        pix = QPixmap(q_im)
        self.image_holder.setPixmap(pix)
        ##### END BOTTOM LEFT #######


        ## Fill the grid
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
        self.active_single_scan = landing_message.active_single_scan
        self.single_right_status.setText("True" if self.active_single_scan else "False")
        if self.active_single_scan:
            self.single_right_status.setStyleSheet("background-color: lightgreen")
        else:
            self.single_right_status.setStyleSheet("background-color: lightcoral")


    def callback_rgbd_image(self, topic_name, image: ImageMessage, time_):
        if not self.active_single_scan:
            self.update_image(image)
    
    def update_image(self, image: ImageMessage):
        im = np.frombuffer(image.image_data, dtype=np.uint8).reshape((image.height, image.width, 3))
        im = im[...,::-1].copy()
        

        self.image_queue.put(im)
        # pix = QPixmap(q_im)
        # self.image_holder.setPixmap(pix)

        logger.info("Did it work...")


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
