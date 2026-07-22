import sys
import threading
from urllib.parse import urlsplit

from PyQt6.QtCore import QObject, QTimer, pyqtSignal
from PyQt6.QtWidgets import QApplication

from drive.forklift_control import ForkliftClient
from main_thread import MainThread
from perception.camera import discover_camera_sources
from ui.main_window import MainWindow


class AppController(QObject):
    connection_result_signal = pyqtSignal(str, bool)

    def __init__(self):
        super().__init__()
        self.gui = MainWindow()
        self.forklift = ForkliftClient()
        self.perception_thread = MainThread(forklift=self.forklift)
        self._connection_busy = False
        self._connection_generation = 0

        self.pressed_keys = set()
        self.current_throttle = 0
        self.current_steering = 100
        self.current_mast = 0
        self.direction_cooldown = 0

        self.control_timer = QTimer(self)
        self.control_timer.timeout.connect(self._process_drive_commands)
        self.control_timer.start(50)

        self._connect_signals()
        self.refresh_cameras()

        app = QApplication.instance()
        if app is not None:
            app.aboutToQuit.connect(self.shutdown)

    def _connect_signals(self):
        worker = self.perception_thread
        gui = self.gui

        worker.new_frame_signal.connect(gui.display_image)
        worker.camera_status_signal.connect(gui.set_camera_status)
        worker.marker_status_signal.connect(gui.set_marker_status)
        worker.planner_status_signal.connect(gui.set_planner_status)
        worker.forklift_status_signal.connect(gui.set_forklift_status)
        worker.go_state_signal.connect(gui.set_go_checked)

        gui.show_path_signal.connect(worker.set_show_path)
        gui.go_signal.connect(worker.set_go)
        gui.override_signal.connect(self.set_override)
        gui.planner_changed_signal.connect(worker.set_planner)
        gui.manual_drive_signal.connect(self.handle_manual_drive)
        gui.camera_refresh_signal.connect(self.refresh_cameras)
        gui.camera_connect_signal.connect(worker.request_camera)
        gui.camera_disconnect_signal.connect(worker.disconnect_camera)
        gui.forklift_connect_signal.connect(self.connect_forklift)
        gui.forklift_disconnect_signal.connect(self.disconnect_forklift)
        gui.emergency_stop_signal.connect(self.emergency_stop)
        self.connection_result_signal.connect(gui.set_forklift_status)

    def refresh_cameras(self):
        self.gui.set_camera_sources(discover_camera_sources())

    def connect_forklift(self, host):
        if self._connection_busy:
            return
        self._connection_busy = True
        self._connection_generation += 1
        generation = self._connection_generation
        uri = self._normalize_uri(host)
        self.gui.set_forklift_status("Connecting…", False)

        def connect():
            try:
                self.forklift.open(uri)
                if generation != self._connection_generation:
                    self.forklift.close()
                    return
                self.connection_result_signal.emit("Connected", True)
            except Exception as error:
                self.forklift.close()
                if generation == self._connection_generation:
                    self.connection_result_signal.emit(
                        f"Connection failed: {error}", False
                    )
            finally:
                self._connection_busy = False

        threading.Thread(target=connect, daemon=True).start()

    def disconnect_forklift(self):
        self._connection_generation += 1
        self.perception_thread.set_go(False)
        self.pressed_keys.clear()
        self._stop_vehicle()
        self.forklift.close()
        self.connection_result_signal.emit("Disconnected", False)

    def set_override(self, enabled):
        self.pressed_keys.clear()
        self._stop_vehicle()
        self.current_throttle = 0
        self.current_steering = 100
        self.current_mast = 0
        self.perception_thread.set_override(enabled)

    def emergency_stop(self):
        self.pressed_keys.clear()
        self.perception_thread.emergency_stop()
        self._stop_vehicle()
        self.current_throttle = 0
        self.current_steering = 90
        self.current_mast = 0

    def _stop_vehicle(self):
        if not self.forklift.is_connected:
            return
        try:
            self.forklift.stop_throttle()
            self.forklift.stop_steering()
            self.forklift.mastControl_stop()
        except Exception as error:
            self.connection_result_signal.emit(f"Stop failed: {error}", False)

    @staticmethod
    def _normalize_uri(host):
        uri = host.strip()
        if not uri.startswith(("ws://", "wss://")):
            uri = f"ws://{uri}"
        parsed = urlsplit(uri)
        if parsed.path in ("", "/"):
            uri = uri.rstrip("/") + "/CarInput"
        return uri

    def shutdown(self):
        self.control_timer.stop()
        self.pressed_keys.clear()
        self.perception_thread.set_go(False)
        if self.perception_thread.isRunning():
            self.perception_thread.stop()
        self._stop_vehicle()
        self.forklift.close()

    def handle_manual_drive(self, key, is_pressed):
        if not self.perception_thread.override or not self.forklift.is_connected:
            return
        if is_pressed:
            self.pressed_keys.add(key)
        else:
            self.pressed_keys.discard(key)

    def _process_drive_commands(self):
        if not self.perception_thread.override or not self.forklift.is_connected:
            return

        raw_target_throttle = 0
        if "w" in self.pressed_keys:
            raw_target_throttle = 200
        elif "s" in self.pressed_keys:
            raw_target_throttle = -200

        target_throttle = raw_target_throttle
        if self.direction_cooldown > 0:
            target_throttle = 0
            self.direction_cooldown -= 1
        else:
            direction_changed = (
                self.current_throttle < 0 < raw_target_throttle
                or self.current_throttle > 0 > raw_target_throttle
            )
            if direction_changed:
                self.direction_cooldown = 3
                target_throttle = 0

        target_steering = 100
        if "a" in self.pressed_keys:
            target_steering = 130
        elif "d" in self.pressed_keys:
            target_steering = 70

        target_mast = 0
        if "j" in self.pressed_keys:
            target_mast = 6
        elif "k" in self.pressed_keys:
            target_mast = 5

        try:
            if "h" in self.pressed_keys:
                self.forklift.mastTilt_backward()
            elif "l" in self.pressed_keys:
                self.forklift.mastTilt_forward()

            self.current_throttle = self._send_changed(
                target_throttle,
                self.current_throttle,
                self.forklift.send_throttle,
            )
            self.current_steering = self._send_changed(
                target_steering,
                self.current_steering,
                self.forklift.send_steering,
            )
            self.current_mast = self._send_changed(
                target_mast,
                self.current_mast,
                self.forklift.mastControl,
            )
        except Exception as error:
            self.pressed_keys.clear()
            self.connection_result_signal.emit(f"Connection lost: {error}", False)

    @staticmethod
    def _send_changed(target, current, send_function):
        if target != current:
            send_function(target)
            return target
        return current


def main():
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    controller = AppController()
    controller.gui.show()
    controller.perception_thread.start()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
