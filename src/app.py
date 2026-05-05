import sys
import cv2
import numpy as np
from PyQt6.QtWidgets import (QApplication, QMainWindow, QLabel, QPushButton,
                             QVBoxLayout, QHBoxLayout, QWidget)
from qasync import asyncSlot

from ui.main_window import MainWindow
from perception.threads import PerceptionThread
from forklift_control import ForkliftClient


class AppController:
    def __init__(self):
        self.gui = MainWindow()
        self.perception_thread = PerceptionThread()
        # self.forklift = ForkliftClient("ws://192.168.4.1/CarInput")

        # Route the image to the GUI
        self.perception_thread.new_frame_signal.connect(self.gui.display_image)

        # Route the math to the hardware via qasync
        # self.perception_thread.drive_command_signal.connect(self.handle_autonomous_drive)

        self.gui.toggle_showPath_signal.connect(self.perception_thread.toggle_showPath)    
        self.gui.go_signal.connect(self.perception_thread.toggle_go)
        self.gui.override_signal.connect(self.perception_thread.toggle_override)

        app = QApplication.instance()
        if app is not None:
            app.aboutToQuit.connect(self.shutdown)

    def shutdown(self):
        if self.perception_thread.isRunning():
            self.perception_thread.stop()

    @asyncSlot()
    async def handle_autonomous_drive(self, commands):
        """This automatically triggers whenever the planner calculates a new move."""
        if "throttle" in commands:
            await self.forklift.send_throttle(commands["throttle"])
        if "steering" in commands:
            await self.forklift.send_steering(commands["steering"])


if __name__ == '__main__':
    app = QApplication(sys.argv)
    controller = AppController()
    controller.gui.show()
    controller.perception_thread.start()
    sys.exit(app.exec())
