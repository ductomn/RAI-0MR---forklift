import asyncio
import sys
import cv2
import numpy as np
from PyQt6.QtWidgets import QApplication 
from PyQt6.QtCore import QObject
from qasync import asyncSlot
import qasync


from ui.main_window import MainWindow
from perception.threads import PerceptionThread
from forklift_control import ForkliftClient


class AppController(QObject):
    def __init__(self):

        super().__init__()
        
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
        self.gui.manual_drive_signal.connect(self.handle_manual_drive)

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


    @asyncSlot(str, bool)
    async def handle_manual_drive(self, key, is_pressed):
        if not self.perception_thread.override:
            return
        
        print(f"Manual drive command received: {key} - {'Pressed' if is_pressed else 'Released'}")

        # Map keys to forklift actions (matching your test_drive.py logic)
        if key == 'w':
            await self.forklift.send_throttle(-200 if is_pressed else 0)
        elif key == 's':
            await self.forklift.send_throttle(200 if is_pressed else 0)
        elif key == 'a':
            await self.forklift.send_steering(120 if is_pressed else 90)
        elif key == 'd':
            await self.forklift.send_steering(60 if is_pressed else 90)


        # Alternative approach using a set to track pressed keys for smoother control (optional)
        # # Update the set of pressed keys
        # if is_pressed:
        #     self.pressed_keys.add(key)
        # else:
        #     self.pressed_keys.discard(key)

        # # Handle Throttle (W/S)
        # if key in ['w', 's']:
        #     if 'w' in self.pressed_keys:
        #         await self.forklift.send_throttle(-200) # Forward
        #     elif 's' in self.pressed_keys:
        #         await self.forklift.send_throttle(200)  # Backward
        #     else:
        #         await self.forklift.stop_throttle()     # Stop

        # # Handle Steering (A/D)
        # elif key in ['a', 'd']:
        #     if 'a' in self.pressed_keys:
        #         await self.forklift.send_steering(120)  # Left
        #     elif 'd' in self.pressed_keys:
        #         await self.forklift.send_steering(60)   # Right
        #     else:
        #         await self.forklift.stop_steering()    # Straight


if __name__ == '__main__':
    # app = QApplication(sys.argv)
    # controller = AppController()
    # controller.gui.show()
    # controller.perception_thread.start()
    # sys.exit(app.exec())

    app = QApplication(sys.argv)
    
    # CRITICAL: Use qasync loop instead of app.exec()
    # for proper async handling in PyQt6 (handle_manual_drive)
    loop = qasync.QEventLoop(app)
    asyncio.set_event_loop(loop)
    
    controller = AppController()
    controller.gui.show()
    controller.perception_thread.start()
    
    with loop:
        loop.run_forever()