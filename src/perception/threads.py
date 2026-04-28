from PyQt6.QtGui import QImage
from PyQt6.QtCore import QThread, pyqtSignal
import sys
import cv2
import numpy as np
from PyQt6.QtWidgets import (QApplication, QMainWindow, QLabel, QPushButton,
                             QVBoxLayout, QHBoxLayout, QWidget)
from PyQt6.QtCore import QThread, pyqtSignal, Qt
from PyQt6.QtGui import QImage, QPixmap
from qasync import asyncSlot

from localization import Detection


class PerceptionThread(QThread):
    # Signal 1: Sends the annotated image to the GUI to be displayed
    new_frame_signal = pyqtSignal(QImage)

    # Signal 2: Sends a dictionary (or tuple) of driving commands to the Controller
    drive_command_signal = pyqtSignal(dict)

    def __init__(self):
        super().__init__()
        self._run_flag = True
        self.detector = Detection(cv2.aruco.DICT_4X4_100)
        # self.planner = PathPlanner()

    def run(self):
        camera = cv2.VideoCapture(0)

        try:
            while self._run_flag and not self.isInterruptionRequested():
                # Capture image
                ret, color_frame = camera.read()
                if not ret or color_frame is None:
                    self.msleep(10)
                    continue

                # Process Image (ArUco Detection)
                corners, ids, rejected, annotated_frame = self.detector.detect_markers(
                    color_frame)
                img = self.detector.draw_markers(corners, ids, annotated_frame)

                #  Path Planning

                # # Emit Control Commands to the Forklift
                # if commands:
                #     self.drive_command_signal.emit(commands)

                # 5. Format and Emit Image to the GUI
                rgb_image = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                h, w, ch = rgb_image.shape
                qt_image = QImage(rgb_image.data, w, h, ch *
                                  w, QImage.Format.Format_RGB888)
                self.new_frame_signal.emit(qt_image)
        finally:
            camera.release()

    def stop(self):
        self._run_flag = False
        self.requestInterruption()
        self.wait()
