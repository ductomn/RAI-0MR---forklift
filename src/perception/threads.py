from PyQt6.QtGui import QImage
from PyQt6.QtCore import QThread, pyqtSignal, pyqtSlot
import cv2
import time
import asyncio

from localization import Detection
from pathPlaning_Astar.PathMain import MainPathPlaning


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
        self.show_path = False
        self.override = False
        self.go = False

        # Path planing mandatory porameters
        self.mainPathPlaning = (
            MainPathPlaning()
        )  # some parameters are needed to change as needed
        self.epsilon = 1  # Max error of theta + position
        self.dt = 1  # Time interval of path planing
        self.stateSpace = [10, 10]  # This defimes max dimensions of povements [x y]
        self.lastTime = None

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
                    color_frame
                )
                img = self.detector.draw_markers(corners, ids, annotated_frame)

                if not self.override and len(corners) >= 2:
                    #  Path Planning
                    now = time.time()
                    if self.lastTime is None or (now - self.lastTime) >= self.dt:
                        self.lastTime = now

                        # TODO: use specific marker IDs to identify forklift and goal state, not based on order
                        realState = self.detector.get_position_simple(
                            corners[0]
                        )  # TODO check if corners are indexed like this -> 1. forklift and 2. goalState
                        goalState = self.detector.get_position_simple(
                            corners[1]
                        )  # TODO check if corners are indexed like this -> 1. forklift and 2. goalState

                        print(f"Real State: {realState}, Goal State: {goalState}")

                        if self.mainPathPlaning is not None:
                            # If error of real state and planed state >= epsilon -> replan
                            if self.mainPathPlaning.error(2 * self.epsilon, realState):
                                # stop movements
                                # TODO: send stop command to forklift (with emit signal)

                                # replan
                                self.mainPathPlaning.startPlaning(
                                    self.dt,
                                    realState,
                                    goalState,
                                    self.stateSpace,
                                    self.epsilon,
                                )

                            elif not self.mainPathPlaning.path:
                                # replan
                                self.mainPathPlaning.startPlaning(
                                    self.dt,
                                    realState,
                                    goalState,
                                    self.stateSpace,
                                    self.epsilon,
                                )

                        # Execute movements
                        if self.mainPathPlaning.index < len(
                            self.mainPathPlaning.actions
                        ):
                            # get actual action
                            v, theta = self.mainPathPlaning.actions[
                                self.mainPathPlaning.index
                            ]

                            # Execute actions
                            # TODO: send drive commands to forklift (with emit signal)

                    #  Show Path Visualization if enabled
                    if self.show_path:
                        # Perform path planning logic here
                        # put text on the image to indicate path planning is active
                        cv2.putText(
                            img,
                            "Path Planning Active",
                            (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            1,
                            (0, 255, 0),
                            2,
                        )

                        # print("Showing path - Add path planning logic here")

                    # # Emit Control Commands to the Forklift
                    # if self.go and commands:
                    #     self.drive_command_signal.emit(commands)

                # Convert annotated image to QImage and emit to GUI
                rgb_image = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                h, w, ch = rgb_image.shape
                qt_image = QImage(
                    rgb_image.data, w, h, ch * w, QImage.Format.Format_RGB888
                )
                self.new_frame_signal.emit(qt_image)
        finally:
            camera.release()

    def stop(self):
        self._run_flag = False
        self.requestInterruption()
        self.wait()

    @pyqtSlot()
    def toggle_showPath(self):
        self.show_path = not self.show_path

    @pyqtSlot()
    def toggle_go(self):
        self.go = not self.go

    @pyqtSlot()
    def toggle_override(self):
        self.override = not self.override
