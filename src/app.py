import sys
from PyQt6.QtWidgets import QApplication
from PyQt6.QtCore import QObject, QTimer

from ui.main_window import MainWindow
from perception.threads import PerceptionThread
from forklift_control import ForkliftClient

# websocket debugging
import websocket

websocket.enableTrace(True)


class AppController(QObject):
    def __init__(self):
        super().__init__()

        self.gui = MainWindow()

        # connect to forklift and pass conector
        uri = "ws://192.168.4.1/CarInput"
        self.forklift = ForkliftClient(uri)
        self.perception_thread = PerceptionThread(forklift=self.forklift)

        self.pressed_keys = set()  # Track active keys

        # Track current states to send only on change
        self.current_throttle = 0
        self.current_steering = 90
        self.direction_cooldown = 0

        # Timer to process continuous driving and prevent socket flooding
        self.control_timer = QTimer()
        self.control_timer.timeout.connect(self._process_drive_commands)
        self.control_timer.start(50)  # Runs every 50ms (20 Hz)

        # Route the image to the GUI
        self.perception_thread.new_frame_signal.connect(self.gui.display_image)

        # Route the math to the hardware via qasync
        # self.perception_thread.drive_command_signal.connect(self.handle_autonomous_drive)

        self.gui.toggle_showPath_signal.connect(self.perception_thread.toggle_showPath)
        self.gui.go_signal.connect(self.perception_thread.toggle_go)
        self.gui.override_signal.connect(self.perception_thread.toggle_override)
        self.gui.manual_drive_signal.connect(self.handle_manual_drive)

        app = QApplication.instance()
        if app is not None:
            app.aboutToQuit.connect(self.shutdown)

    def shutdown(self):
        if self.perception_thread.isRunning():
            self.perception_thread.stop()
        self.control_timer.stop()

    def handle_autonomous_drive(self, commands):
        """This automatically triggers whenever the planner calculates a new move."""
        if "throttle" in commands:
            self.forklift.send_throttle(commands["throttle"])
        if "steering" in commands:
            self.forklift.send_steering(commands["steering"])

    def handle_manual_drive(self, key, is_pressed):
        if not self.perception_thread.override:
            return

        # Just update the state set. The QTimer handles actual sending.
        if is_pressed:
            self.pressed_keys.add(key)
        else:
            self.pressed_keys.discard(key)

    def _process_drive_commands(self):
        if not self.perception_thread.override:
            return

        raw_target_throttle = 0
        if "w" in self.pressed_keys:
            raw_target_throttle = -200
        elif "s" in self.pressed_keys:
            raw_target_throttle = 200

        # Safety logic: Prevent instant direction changes
        target_throttle = raw_target_throttle

        if self.direction_cooldown > 0:
            target_throttle = 0
            self.direction_cooldown -= 1
        else:
            # Check if we are trying to reverse direction instantly
            is_moving_forward = self.current_throttle < 0
            is_moving_backward = self.current_throttle > 0
            wants_to_go_backward = raw_target_throttle > 0
            wants_to_go_forward = raw_target_throttle < 0

            if (is_moving_forward and wants_to_go_backward) or (
                is_moving_backward and wants_to_go_forward
            ):
                # Detected a direction swap
                # Set cooldown to 3 ticks (3 * 50ms = 150ms delay)
                self.direction_cooldown = 3
                target_throttle = 0  # Force stop immediately

        target_steering = 90
        if "a" in self.pressed_keys:
            target_steering = 120
        elif "d" in self.pressed_keys:
            target_steering = 60

        # Helper function to send and handle reconnects
        def safe_send(action_type, target_val, current_val, send_func):
            if target_val != current_val:
                try:
                    send_func(target_val)
                    return target_val
                except Exception as e:
                    print(f"Network error sending {action_type}: {e}")
                    print("Attempting to reconnect to forklift...")
                    try:
                        self.forklift.close()
                        self.forklift.open()
                        send_func(target_val)
                        return target_val
                    except Exception as reconnect_error:
                        print(f"Reconnect failed: {reconnect_error}")
                        self.pressed_keys.clear()
            return current_val

        self.current_throttle = safe_send(
            "throttle",
            target_throttle,
            self.current_throttle,
            self.forklift.send_throttle,
        )
        self.current_steering = safe_send(
            "steering",
            target_steering,
            self.current_steering,
            self.forklift.send_steering,
        )


if __name__ == "__main__":
    app = QApplication(sys.argv)
    controller = AppController()
    controller.gui.show()
    controller.perception_thread.start()
    sys.exit(app.exec())

    # app = QApplication(sys.argv)

    # # CRITICAL: Use qasync loop instead of app.exec()
    # # for proper async handling in PyQt6 (handle_manual_drive)
    # loop = qasync.QEventLoop(app)
    # asyncio.set_event_loop(loop)

    # controller = AppController()
    # controller.gui.show()
    # controller.perception_thread.start()

    # with loop:
    #     loop.run_forever()
