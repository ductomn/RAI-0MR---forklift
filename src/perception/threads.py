from PyQt6.QtGui import QImage
from PyQt6.QtCore import QThread, pyqtSignal, pyqtSlot
import cv2
import time
import math
import asyncio

from localization import Detection
from pathPlaning_Astar.PathMain import MainPathPlaning
from forklift_control import ForkliftClient


class PerceptionThread(QThread):
    # Signal 1: Sends the annotated image to the GUI to be displayed
    new_frame_signal = pyqtSignal(QImage)

    # Signal 2: Sends a dictionary (or tuple) of driving commands to the Controller
    drive_command_signal = pyqtSignal(dict)

    def __init__(self, forklift: ForkliftClient):
        super().__init__()
        self._run_flag = True
        self.detector = Detection(cv2.aruco.DICT_4X4_100)
        # self.planner = PathPlanner()
        self.show_path = False
        self.override = False
        self.go = False
        self.forklift: ForkliftClient = forklift  # passed class for controll

        # Path planing mandatory porameters
        self.mainPathPlaning = (
            MainPathPlaning()
        )  # some parameters are needed to change as needed
        self.epsilon = 1  # Max error of theta + position
        self.dt = 5  # Time interval of path planing
        self.stateSpace = [600, 400]  # This defimes max dimensions of povements [x y]
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
                corners, ids, _, annotated_frame = self.detector.detect_markers(
                    color_frame
                )
                img = self.detector.draw_markers(corners, ids, annotated_frame)

                if not self.override and len(corners) >= 2:
                    #  Path Planning
                    now = time.time()
                    if self.lastTime is None or (now - self.lastTime) >= self.dt:
                        self.lastTime = now

                        # get real states in order as defined in detect_markers
                        realState = self.detector.get_position_simple(corners[0])
                        goalState = self.detector.get_position_simple(corners[1])

                        # print(f"Real State: {realState}, Goal State: {goalState}")

                        if self.mainPathPlaning is not None:
                            # If error of real state and planed state >= epsilon -> replan
                            if self.mainPathPlaning.error(2 * self.epsilon, realState):
                                # stop movements
                                # TODO: send stop command to forklift (with emit signal)
                                self.forklift.stop_steering()
                                self.forklift.stop_throttle()

                                # replan
                                self.mainPathPlaning.startPlaning(
                                    self.dt,
                                    realState,
                                    goalState,
                                    self.stateSpace,
                                    self.epsilon,
                                )
                                # Good path
                                # print("path found")
                                # print(self.mainPathPlaning.path)

                        # Execute movements
                        if self.mainPathPlaning.index < len(
                            self.mainPathPlaning.actions
                        ):
                            # get actual action
                            v, steer = self.mainPathPlaning.actions[
                                self.mainPathPlaning.index
                            ]

                            # Execute actions
                            self.forklift.send_steering(steer)
                            self.forklift.send_throttle(-v * 10)

                #  Show Path Visualization if enabled
                if self.show_path and not self.override and self.mainPathPlaning.path:
                    # Perform path planning logic here -> 'no' ps.DC
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
                    for i in range(len(self.mainPathPlaning.path)):
                        x, y, theta = self.mainPathPlaning.path[i]

                        # convert float -> int pixels
                        pos = (int(x), int(y))

                        # draw
                        cv2.circle(img, pos, 5, (0, 0, 255), -1)

                        arrow = 10
                        end_x = int(x + arrow * math.cos(theta))
                        end_y = int(y + arrow * math.sin(theta))

                        cv2.arrowedLine(
                            img,
                            pos,
                            (end_x, end_y),
                            (255, 0, 0),  # blue
                            2,
                        )

                        # Draw line to next point
                        if i < len(self.mainPathPlaning.path) - 1:
                            x2, y2, _ = self.mainPathPlaning.path[i + 1]

                            cv2.line(
                                img,
                                pos,
                                (int(x2), int(y2)),
                                (0, 255, 0),  # green
                                2,
                            )

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
