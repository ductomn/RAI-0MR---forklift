from PyQt6.QtGui import QImage
from PyQt6.QtCore import QThread, pyqtSignal, pyqtSlot
import cv2
import time
import math
import numpy as np
import perception.camera as cam

from perception.localization import Detection
from pathPlaning_Astar.PathMain import MainPathPlaning
from drive.forklift_control import ForkliftClient


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

        # Path planing mandatory parameters
        self.mainPathPlaning = (
            MainPathPlaning()
        )  # some parameters are needed to change as needed
        self.epsilon = 20  # Max error of theta + position
        self.dt = 0.5  # Time interval of path planing
        self.stateSpace = [600, 400]  # This defimes max dimensions of povements [x y]
        self.markersize = (
            45  # This is the size of the ArUco marker in mm for real state estimation
        )
        self.px_mm = 0
        self.lastTime = None

    def run(self):
        # camera = cv2.VideoCapture(0)
        camera = cam.ImageProcessor(640, 480, 30)
        camera.start()
        try:
            while self._run_flag and not self.isInterruptionRequested():
                # Capture image
                # _, frame = camera.read()
                # if not camera.isOpened():
                #     self.msleep(10)
                #     continue
                frame = camera.get_frames()
                if not camera.is_running():
                    self.msleep(10)
                    continue

                # Process Image (ArUco Detection)
                corners, ids, _, annotated_frame = self.detector.detect_markers(frame)
                img = self.detector.draw_markers(corners, ids, annotated_frame)
                frameHeight = frame.shape[0]
                goalState = None

                if not self.override and len(corners) >= 2:
                    #  Path Planning
                    now = time.time()
                    if self.lastTime is None or (now - self.lastTime) >= self.dt:
                        self.lastTime = now

                        # get real states in mm
                        realState = self.detector.get_position_simple_mm(
                            corners[0], corners, self.markersize, frameHeight
                        )
                        goalState = self.detector.get_position_simple_mm(
                            corners[1], corners, self.markersize, frameHeight
                        )
                        goalState = self.mainPathPlaning.newGoalState(goalState)
                        resized_stateSpace, self.px_mm = (
                            self.detector.resize_statespace_mm(
                                corners, self.markersize, self.stateSpace
                            )
                        )

                        self.mainPathPlaning.inGoal(self.epsilon, realState, goalState)

                        # print(f"Real State: {realState}, Goal State: {goalState}")

                        if self.mainPathPlaning is not None:
                            # If error of real state and planed state >= epsilon -> replan
                            if (
                                self.mainPathPlaning.error(2 * self.epsilon, realState)
                                and not self.mainPathPlaning.goalReached
                            ):
                                # stop movements
                                self.forklift.stop_steering()
                                time.sleep(0.1)
                                self.forklift.stop_throttle()

                                # replan
                                self.mainPathPlaning.startPlaning(
                                    self.dt,
                                    realState,
                                    goalState,
                                    resized_stateSpace,
                                    self.epsilon,
                                )
                                # Good path
                                # print("path found")
                                # print(self.mainPathPlaning.path)

                        # Execute movements
                        if (
                            self.mainPathPlaning.index
                            < len(self.mainPathPlaning.actions)
                            and not self.mainPathPlaning.goalReached
                            and self.go
                        ):
                            # get actual action
                            v, steer = self.mainPathPlaning.actions[
                                self.mainPathPlaning.index
                            ]

                            print(
                                f"Executing action: v={v}, steer={steer}, int_v={int(v * 0.617)}"
                            )

                            # Execute actions
                            self.forklift.send_steering(
                                int(np.rad2deg(steer) * 1.12) + 90
                            )
                            time.sleep(0.1)
                            self.forklift.send_throttle(int(v * 0.617))
                        if self.mainPathPlaning.goalReached:
                            # add here code after all movements were done
                            self.forklift.send_steering(90)
                            time.sleep(0.1)
                            self.forklift.send_throttle(0)

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
                        x_px = int(x * self.px_mm)
                        y_px = int(
                            frameHeight - y * self.px_mm
                        )  # flip Y back to pixel space
                        # convert float -> int pixels
                        pos = (x_px, y_px)

                        # draw
                        cv2.circle(img, pos, 5, (0, 0, 255), -1)

                        arrow = 10
                        end_x = int(x_px + arrow * math.cos(theta))
                        end_y = int(y_px - arrow * math.sin(theta))

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
                            x2 = int(x2 * self.px_mm)
                            y2 = int(frameHeight - y2 * self.px_mm)
                            cv2.line(
                                img,
                                pos,
                                (int(x2), int(y2)),
                                (0, 255, 0),  # green
                                2,
                            )

                if (not self.override and not self.go) or len(corners) < 2:
                    self.forklift.stop_steering()
                    self.forklift.stop_throttle()

                # Convert annotated image to QImage and emit to GUI
                rgb_image = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                h, w, ch = rgb_image.shape
                qt_image = QImage(
                    rgb_image.data, w, h, ch * w, QImage.Format.Format_RGB888
                )
                self.new_frame_signal.emit(qt_image)
        finally:
            # camera.release()
            camera.stop()

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
