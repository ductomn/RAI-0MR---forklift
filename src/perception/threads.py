from PyQt6.QtGui import QImage
from PyQt6.QtCore import QThread, pyqtSignal, pyqtSlot
import cv2
import time
import math
import numpy as np
import perception.camera as cam

from perception.localization import Detection
from pathPlaning.PathMain import MainPathPlaning
# from pathPlaning.PathMainDijkstra import MainPathPlaning
from drive.forklift_control import ForkliftClient


class PerceptionThread(QThread):
    # Signal 1: Sends the annotated image to the GUI to be displayed
    new_frame_signal = pyqtSignal(QImage)

    def __init__(self, forklift: ForkliftClient):
        super().__init__()
        self._run_flag = True
        self.detector = Detection(cv2.aruco.DICT_4X4_100)
        # self.planner = PathPlanner()
        self.show_path = False
        self.override = False
        self.go = False
        self.forklift: ForkliftClient = forklift  # passed class for controll
        self.mode = 0  # Choose witch path planer i am using
        self.pickUpDone = False

        # Path planing mandatory parameters
        self.mainPathPlaning = (
            MainPathPlaning()
        )  # some parameters are needed to change as needed
        self.epsilon = 10  # Max error of position
        self.epsilonTheta = 0.7
        self.dt = 0.2  # Time interval of path planing
        self.stateSpace = [600, 400]  # This defimes max dimensions of movements [x y]
        self.markersize = 45  # This is the size of the ArUco marker
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
                # height, width = frame.shape[:2]
                # self.stateSpace = [width, height]

                # if not camera.isOpened():
                #     self.msleep(10)
                #     continue

                frame = camera.get_frames()
                height, width = frame.shape[:2]
                self.stateSpace = [width, height]
                
                if not camera.is_running():
                    self.msleep(10)
                    continue

                # Process Image (ArUco Detection)
                corners, ids, _, annotated_frame = self.detector.detect_markers(frame)
                img = self.detector.draw_markers(corners, ids, annotated_frame)
                frameHeight = frame.shape[0]

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

                        self.mainPathPlaning.inGoal(
                            self.epsilon, self.epsilonTheta, realState, goalState
                        )

                        #print(f"Real State: {realState}, Goal State: {goalState}")

                        if self.mainPathPlaning is not None:
                            # If error of real state and planed state >= epsilon -> replan
                            if (
                                self.mainPathPlaning.error(2 * self.epsilon,4 * self.epsilonTheta, realState)
                                and not self.mainPathPlaning.goalReached
                            ):
                                self.choosePathPlaner(
                                    self.mode, realState, goalState, resized_stateSpace
                                )
                                now = time.time()  # start timer only after replan
                                self.lastTime = now

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

                            # print(
                            #     f"Executing action: v={v}, steer={steer}, int_v={int(v * 0.617)}"
                            # )

                            # Execute actions
                            self.forklift.send_steering(
                                int(np.rad2deg(steer) * 1.11) + 100
                            )
                            time.sleep(0.1)
                            self.forklift.send_throttle(int(v * 0.617))

                        if self.mainPathPlaning.goalReached and not self.pickUpDone:
                            # when in goal pick up pallet
                            self.pickUpSeq()

                #  Show Path Visualization if enabled
                if self.show_path and not self.override and self.mainPathPlaning.path:
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

                if (not self.go) and not self.override:  # len(corners) < 2 or
                    self.forklift.stop_steering()
                    self.forklift.stop_throttle()

                if self.override:
                    self.pickUpDone = False
                    self.mainPathPlaning.goalReached = False

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

    def choosePathPlaner(self, mode, realState, goalState, stateSpace):
        # stop movements
        self.forklift.stop_steering()
        time.sleep(0.1)
        self.forklift.stop_throttle()

        # choose which path planer will be used
        match mode:
            case 0:
                print("Replaning with A* hybrit (DC)")
                # replan
                self.mainPathPlaning.startPlaning(
                    self.dt,
                    realState,
                    goalState,
                    stateSpace,
                    self.epsilon,
                    self.epsilonTheta
                )

            case 1:
                print("Replaning with WHUt")

            case 2:
                print("Replaning with Whut")

        # Good path
        print("path found")
        print(self.mainPathPlaning.path)

    def pickUpSeq(self):
        # Stop movements
        self.forklift.stop_steering()
        time.sleep(0.01)
        self.forklift.stop_throttle()

        print("goal reached")
        time.sleep(2)

        # 1. mast down and tilt forward
        self.forklift.mastControl_down()
        time.sleep(1)
        self.forklift.mastControl_stop()
        time.sleep(0.01)

        for _ in range(20):
            self.forklift.mastTilt_forward()
            time.sleep(0.01)

        # 2. start go forward
        self.forklift.send_throttle(int(70 * 0.61))
        time.sleep(2)

        # 3. stop throttle and pick up pallet
        self.forklift.stop_throttle()
        time.sleep(0.01)

        for _ in range(20):
            self.forklift.mastTilt_backward()
            time.sleep(0.01)

        self.forklift.mastControl_up()
        time.sleep(1)

        # 4. stop mast
        self.forklift.mastControl_stop()

        self.pickUpDone = True

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
