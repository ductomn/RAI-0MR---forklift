import math
import threading
import time

import cv2
import numpy as np
from PyQt6.QtCore import QThread, pyqtSignal, pyqtSlot
from PyQt6.QtGui import QImage

import perception.camera as camera_module
from drive.forklift_control import ForkliftClient
from pathPlaning.PathMain import MainPathPlaning
from perception.localization import Detection


_NO_CAMERA_REQUEST = object()


class MainThread(QThread):
    new_frame_signal = pyqtSignal(QImage)
    camera_status_signal = pyqtSignal(str, bool)
    marker_status_signal = pyqtSignal(str, bool)
    planner_status_signal = pyqtSignal(str, bool)
    forklift_status_signal = pyqtSignal(str, bool)
    go_state_signal = pyqtSignal(bool)

    PLANNER_NAMES = ("Hybrid A*", "Kinodynamic RRT", "Dijkstra")
    DISPLAY_FPS = 20.0
    REPLAN_POSITION_MM = 20.0
    REPLAN_ANGLE_RAD = math.radians(8.0)
    PALLET_STOP_DISTANCE_MM = MainPathPlaning.GOAL_STANDOFF_MM

    def __init__(self, forklift: ForkliftClient = None):
        super().__init__()
        self._run_flag = True
        self._camera_lock = threading.Lock()
        self._pending_camera = _NO_CAMERA_REQUEST
        self._active_camera = None
        self._active_camera_source = None
        self._planning_lock = threading.Lock()
        self._planning_result = None
        self._planning_busy = False
        self._planning_generation = 0
        self._next_plan_time = 0.0
        self._plan_reference_start = None
        self._plan_reference_goal_marker = None
        self._last_frame_emit = 0.0

        self.detector = Detection(cv2.aruco.DICT_4X4_100)
        self.show_path = False
        self.override = False
        self.go = False
        self.forklift = forklift
        self.mode = 0
        self.pickUpDone = False

        self.mainPathPlaning = MainPathPlaning()
        self.epsilon = 7
        self.dt = 0.5
        self.stateSpace = [600, 400]
        self.markersize = 45
        self.px_mm = 0
        self.lastTime = None
        self._motors_stopped = True
        self._last_marker_state = None

    def run(self):
        failed_frames = 0
        try:
            while self._run_flag and not self.isInterruptionRequested():
                self._apply_camera_request()
                camera = self._active_camera
                if camera is None:
                    self.msleep(50)
                    continue

                if not camera.is_opened():
                    self._camera_failed("Camera disconnected")
                    continue

                ok, frame = camera.read()
                if not ok or frame is None:
                    failed_frames += 1
                    if failed_frames >= 10:
                        self._camera_failed("Camera stopped returning frames")
                    else:
                        self.msleep(20)
                    continue
                failed_frames = 0

                height, width = frame.shape[:2]
                self.stateSpace = [width, height]

                try:
                    corners, ids, _, annotated_frame = self.detector.detect_markers(frame)
                    image = self.detector.draw_markers(corners, ids, annotated_frame)
                    self._report_markers(len(corners))
                    self._collect_plan_result()
                    self._process_planning(corners, height)
                    self._draw_pallet_border(image, corners)
                    self._draw_path(image, height)
                except Exception as error:
                    image = frame
                    self.planner_status_signal.emit(
                        f"Frame processing error: {error}", False
                    )

                # Do not build an unbounded queue of stale frames in the GUI
                # thread when detection or path planning temporarily runs slow.
                emit_time = time.monotonic()
                if emit_time - self._last_frame_emit >= 1.0 / self.DISPLAY_FPS:
                    self._last_frame_emit = emit_time
                    rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
                    h, w, channels = rgb_image.shape
                    qt_image = QImage(
                        rgb_image.data,
                        w,
                        h,
                        channels * w,
                        QImage.Format.Format_RGB888,
                    ).copy()
                    self.new_frame_signal.emit(qt_image)
        finally:
            self._close_camera()
            self._safe_stop()

    def _process_planning(self, corners, frame_height):
        if self.override:
            self.pickUpDone = False
            self.mainPathPlaning.goalReached = False
            return

        if len(corners) < 2:
            if self.go:
                self._disable_go("Stopped: forklift and pallet markers are required")
            return

        now = time.time()
        if self.lastTime is not None and (now - self.lastTime) < self.dt:
            return
        self.lastTime = now

        real_state = self.detector.get_position_simple_mm(
            corners[0], corners, self.markersize, frame_height
        )
        goal_marker_state = self.detector.get_position_simple_mm(
            corners[1], corners, self.markersize, frame_height
        )
        goal_state = self.mainPathPlaning.newGoalState(goal_marker_state)
        resized_state_space, self.px_mm = self.detector.resize_statespace_mm(
            corners, self.markersize, self.stateSpace
        )

        planning_requested = self.show_path or self.go
        if planning_requested and self._plan_pose_changed(
            real_state, goal_marker_state
        ):
            self._invalidate_plan_for_pose(real_state, goal_marker_state)

        self.mainPathPlaning.inGoal(
            2 * self.epsilon, self.epsilon / 5, real_state, goal_state
        )
        pallet_distance = float(
            np.linalg.norm(
                np.asarray(real_state[:2]) - np.asarray(goal_marker_state[:2])
            )
        )
        if pallet_distance <= self.PALLET_STOP_DISTANCE_MM:
            # This guard is independent of the moving virtual goal: once the
            # forklift enters the pallet border it cannot chase a pushed pallet.
            self.mainPathPlaning.goalReached = True
            self.mainPathPlaning.path = []
            self.mainPathPlaning.actions = []
            self.mainPathPlaning.plan_complete = False
            self._safe_stop()

        needs_plan = planning_requested and not self.mainPathPlaning.path
        if self.go and self.mainPathPlaning.path:
            needs_plan = not self.mainPathPlaning.plan_complete
            if not needs_plan:
                needs_plan = self.mainPathPlaning.error(
                    2 * self.epsilon, self.epsilon / 5, real_state
                )
        if (
            needs_plan
            and not self.mainPathPlaning.goalReached
            and now >= self._next_plan_time
        ):
            self.choosePathPlaner(
                self.mode,
                real_state,
                goal_state,
                resized_state_space,
                goal_marker_state,
            )
            # A failed search should not immediately consume another CPU core.
            self._next_plan_time = now + 2.0

        if (
            self.mainPathPlaning.plan_complete
            and not self._planning_busy
            and self.mainPathPlaning.index < len(self.mainPathPlaning.actions)
            and not self.mainPathPlaning.goalReached
            and self.go
        ):
            if not self.forklift or not self.forklift.is_connected:
                self._disable_go("Stopped: forklift is disconnected")
                self.forklift_status_signal.emit("Disconnected", False)
                return

            velocity, steering = self.mainPathPlaning.actions[
                self.mainPathPlaning.index
            ]
            try:
                self.forklift.send_steering(
                    int(np.rad2deg(steering) * 1.12) + 100
                )
                time.sleep(0.1)
                self.forklift.send_throttle(int(velocity / 0.617))
                self._motors_stopped = False
                self.planner_status_signal.emit("Following planned path", True)
            except Exception as error:
                self._disable_go(f"Drive command failed: {error}")
                self.forklift_status_signal.emit("Connection lost", False)

        if self.mainPathPlaning.goalReached and not self.pickUpDone:
            self.planner_status_signal.emit("Goal reached", True)
            if self.go and self.forklift and self.forklift.is_connected:
                try:
                    self.pickUpSeq()
                except Exception as error:
                    self._disable_go(f"Pickup failed: {error}")

        if not self.go:
            self._safe_stop()

    def _draw_pallet_border(self, image, corners):
        if not (self.show_path and len(corners) >= 2 and self.px_mm):
            return
        center_x, center_y = self.detector.get_center(corners[1])
        center = (int(center_x), int(center_y))
        radius = max(1, int(self.PALLET_STOP_DISTANCE_MM * self.px_mm))
        cv2.circle(image, center, radius, (0, 200, 255), 2)
        cv2.putText(
            image,
            f"Pallet stop border: {self.PALLET_STOP_DISTANCE_MM:.0f} mm",
            (max(5, center[0] - radius), max(20, center[1] - radius - 6)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 200, 255),
            1,
        )

    def _draw_path(self, image, frame_height):
        if not (
            self.show_path
            and not self.override
            and self.mainPathPlaning.path
            and self.px_mm
        ):
            return

        cv2.putText(
            image,
            f"{self.PLANNER_NAMES[self.mode]} active",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 255, 0),
            2,
        )
        path = self.mainPathPlaning.path
        for index, (x, y, theta) in enumerate(path):
            x_px = int(x * self.px_mm)
            y_px = int(frame_height - y * self.px_mm)
            position = (x_px, y_px)
            cv2.circle(image, position, 5, (0, 0, 255), -1)
            arrow_end = (
                int(x_px + 10 * math.cos(theta)),
                int(y_px - 10 * math.sin(theta)),
            )
            cv2.arrowedLine(image, position, arrow_end, (255, 0, 0), 2)
            if index < len(path) - 1:
                next_x, next_y, _ = path[index + 1]
                next_position = (
                    int(next_x * self.px_mm),
                    int(frame_height - next_y * self.px_mm),
                )
                cv2.line(image, position, next_position, (0, 255, 0), 2)

    def choosePathPlaner(
        self, mode, real_state, goal_state, state_space, goal_marker_state=None
    ):
        """Start a plan without blocking camera capture and GUI frame delivery."""
        with self._planning_lock:
            if self._planning_busy:
                return
            self._planning_busy = True
            generation = self._planning_generation

        self._plan_reference_start = list(real_state)
        self._plan_reference_goal_marker = list(
            goal_marker_state if goal_marker_state is not None else goal_state
        )
        self._safe_stop()
        name = self.PLANNER_NAMES[mode]
        self.planner_status_signal.emit(f"Planning with {name}…", True)

        def calculate():
            planner = MainPathPlaning()
            error = None
            try:
                if mode == 0:
                    planner.startAstarHybrid(
                        self.dt,
                        real_state,
                        goal_state,
                        state_space,
                        self.epsilon,
                        self.epsilon / 5,
                    )
                elif mode == 1:
                    planner.startKinodynamicRRT(
                        self.dt, real_state, goal_state, state_space, self.epsilon
                    )
                elif mode == 2:
                    planner.startDijkstra(
                        self.dt,
                        real_state,
                        goal_state,
                        state_space,
                        self.epsilon,
                        self.epsilon / 5,
                    )
            except Exception as caught_error:
                error = str(caught_error)

            result = (
                generation,
                name,
                planner.path,
                planner.actions,
                planner.plan_complete,
                error,
            )
            with self._planning_lock:
                self._planning_result = result

        threading.Thread(target=calculate, daemon=True).start()

    def _collect_plan_result(self):
        with self._planning_lock:
            result = self._planning_result
            if result is None:
                return
            self._planning_result = None
            self._planning_busy = False

        generation, name, path, actions, complete, error = result
        if generation != self._planning_generation:
            return
        if error:
            self.planner_status_signal.emit(f"{name} failed: {error}", False)
            return

        self.mainPathPlaning.path = path
        self.mainPathPlaning.actions = actions
        self.mainPathPlaning.index = 1
        self.mainPathPlaning.plan_complete = complete
        if not complete:
            self._next_plan_time = time.time() + 2.0
        if complete:
            self.planner_status_signal.emit(f"{name} path ready", True)
        elif path:
            self.planner_status_signal.emit(
                f"{name} search limit reached; showing partial path", False
            )
        else:
            self.planner_status_signal.emit(f"{name} did not find a path", False)

    @classmethod
    def _pose_differs(cls, pose, reference):
        if reference is None:
            return False
        position_error = float(
            np.linalg.norm(np.asarray(pose[:2]) - np.asarray(reference[:2]))
        )
        angle_error = abs((pose[2] - reference[2] + np.pi) % (2 * np.pi) - np.pi)
        return (
            position_error >= cls.REPLAN_POSITION_MM
            or angle_error >= cls.REPLAN_ANGLE_RAD
        )

    def _plan_pose_changed(self, real_state, goal_marker_state):
        goal_changed = self._pose_differs(
            goal_marker_state, self._plan_reference_goal_marker
        )
        # While driving, forklift movement is expected and the path tracker
        # handles drift. In preview mode moving either marker must redraw path.
        start_changed = not self.go and self._pose_differs(
            real_state, self._plan_reference_start
        )
        return goal_changed or start_changed

    def _invalidate_plan_for_pose(self, real_state, goal_marker_state):
        self._invalidate_plan()
        self._plan_reference_start = list(real_state)
        self._plan_reference_goal_marker = list(goal_marker_state)

    def _invalidate_plan(self):
        self._planning_generation += 1
        self._next_plan_time = 0.0
        self.mainPathPlaning.clear()

    def request_camera(self, source: str):
        with self._camera_lock:
            self._pending_camera = source

    def disconnect_camera(self):
        with self._camera_lock:
            self._pending_camera = None

    def _apply_camera_request(self):
        with self._camera_lock:
            source = self._pending_camera
            self._pending_camera = _NO_CAMERA_REQUEST
        if source is _NO_CAMERA_REQUEST:
            return

        self._close_camera()
        if source is None:
            self.camera_status_signal.emit("Disconnected", False)
            self._disable_go("Stopped: camera disconnected")
            return

        self.camera_status_signal.emit("Connecting…", False)
        try:
            camera = camera_module.open_camera(source)
            if not camera.is_opened():
                camera.close()
                raise RuntimeError("device could not be opened")
            self._active_camera = camera
            self._active_camera_source = source
            self.camera_status_signal.emit("Connected", True)
            self.planner_status_signal.emit("Waiting for two ArUco markers", False)
        except Exception as error:
            self.camera_status_signal.emit(f"Camera error: {error}", False)
            self._disable_go("Stopped: camera could not be opened")

    def _close_camera(self):
        self._invalidate_plan()
        camera = self._active_camera
        self._active_camera = None
        self._active_camera_source = None
        if camera is not None:
            try:
                camera.close()
            except Exception as error:
                self.camera_status_signal.emit(f"Close error: {error}", False)

    def _camera_failed(self, message):
        self._close_camera()
        self.camera_status_signal.emit(message, False)
        self._disable_go(f"Stopped: {message.lower()}")

    def _report_markers(self, count):
        state = min(count, 2)
        if state == self._last_marker_state:
            return
        self._last_marker_state = state
        if count >= 2:
            self.marker_status_signal.emit(f"Ready ({count} detected)", True)
        else:
            self.marker_status_signal.emit(f"Need 2 markers ({count} detected)", False)

    def _safe_stop(self):
        if self._motors_stopped:
            return
        if self.forklift and self.forklift.is_connected:
            try:
                self.forklift.stop_throttle()
                self.forklift.stop_steering()
            except Exception:
                self.forklift_status_signal.emit("Connection lost", False)
        self._motors_stopped = True

    def _disable_go(self, reason):
        if self.go:
            self.go = False
            self.go_state_signal.emit(False)
        self._safe_stop()
        self.planner_status_signal.emit(reason, False)

    def emergency_stop(self):
        self.go = False
        self.override = False
        self.go_state_signal.emit(False)
        self._safe_stop()
        self.planner_status_signal.emit("Emergency stop", False)

    def pickUpSeq(self):
        self._safe_stop()
        time.sleep(2)
        self.forklift.mastControl_down()
        time.sleep(1)
        self.forklift.mastControl_stop()
        for _ in range(20):
            self.forklift.mastTilt_forward()
            time.sleep(0.01)
        self.forklift.send_throttle(int(70 * 0.61))
        self._motors_stopped = False
        time.sleep(2)
        self.forklift.stop_throttle()
        self._motors_stopped = True
        for _ in range(20):
            self.forklift.mastTilt_backward()
            time.sleep(0.01)
        self.forklift.mastControl_up()
        time.sleep(1)
        self.forklift.mastControl_stop()
        self.pickUpDone = True

    def stop(self):
        self._run_flag = False
        self.requestInterruption()
        self.wait()

    @pyqtSlot(bool)
    def set_show_path(self, enabled):
        self.show_path = enabled

    @pyqtSlot(bool)
    def set_go(self, enabled):
        if enabled and self.override:
            self.planner_status_signal.emit(
                "Disable manual override before autonomous control", False
            )
            self.go_state_signal.emit(False)
            return
        self.go = enabled
        if not enabled:
            self._safe_stop()
            self.go_state_signal.emit(False)

    @pyqtSlot(bool)
    def set_override(self, enabled):
        self.override = enabled
        if enabled:
            self.go = False
            self.go_state_signal.emit(False)
            self._invalidate_plan()
            self._safe_stop()
            self.planner_status_signal.emit("Manual override active", True)
        else:
            self._safe_stop()
            self.planner_status_signal.emit("Manual override disabled", False)

    @pyqtSlot(int)
    def set_planner(self, mode):
        if not 0 <= mode < len(self.PLANNER_NAMES):
            return
        self.mode = mode
        self.go = False
        self.go_state_signal.emit(False)
        self._invalidate_plan()
        self.lastTime = None
        self._safe_stop()
        self.planner_status_signal.emit(
            f"{self.PLANNER_NAMES[mode]} selected; waiting to plan", False
        )

    # Compatibility slots used by older integrations.
    @pyqtSlot()
    def toggle_showPath(self):
        self.set_show_path(not self.show_path)

    @pyqtSlot()
    def toggle_go(self):
        self.set_go(not self.go)

    @pyqtSlot()
    def toggle_override(self):
        self.set_override(not self.override)
