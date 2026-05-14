from PyQt6.QtCore import pyqtSignal, Qt
from PyQt6.QtWidgets import (QMainWindow, QLabel, QPushButton,
                             QVBoxLayout, QHBoxLayout, QWidget)
from PyQt6.QtGui import QPixmap


class MainWindow(QMainWindow):

    toggle_showPath_signal = pyqtSignal(bool)
    go_signal = pyqtSignal(bool)
    override_signal = pyqtSignal(bool)
    manual_drive_signal = pyqtSignal(str, bool)

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Forklift Control Interface")
        self.resize(700, 600)

        # Create widgets
        self.image_label = QLabel(self)
        self.image_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.image_label.setMinimumSize(640, 480)

        self.btn_showPath = QPushButton("Show path")
        self.btn_go = QPushButton("Go")
        self.btn_override = QPushButton("Override")

        # Connect button signals to slots
        self.btn_showPath.setCheckable(True)
        self.btn_showPath.clicked.connect(self._emit_toggle_showPath)

        self.btn_go.setCheckable(True)
        self.btn_go.clicked.connect(self._emit_go)

        self.btn_override.setCheckable(True)
        self.btn_override.clicked.connect(self._emit_override)

        # Setup Layouts
        vbox = QVBoxLayout()
        vbox.addWidget(self.image_label)

        hbox = QHBoxLayout()
        hbox.addWidget(self.btn_showPath)
        hbox.addWidget(self.btn_go)
        hbox.addWidget(self.btn_override)
        vbox.addLayout(hbox)

        main_widget = QWidget()
        main_widget.setLayout(vbox)
        self.setCentralWidget(main_widget)

    def display_image(self, qt_image):
        """Slot that receives the QImage from the worker thread and updates the GUI."""
        self.image_label.setPixmap(QPixmap.fromImage(qt_image))

    def _emit_toggle_showPath(self, state):
        print(f"Show path button clicked - State: {state}")
        self.toggle_showPath_signal.emit(state)

    def _emit_go(self, state):
        print(f"Go button clicked - State: {state}")
        self.go_signal.emit(state)

    def _emit_override(self, state):
        print(f"Override button clicked - State: {state}")
        self.override_signal.emit(state)

    def keyPressEvent(self, event):
        """Capture key presses and emit signal if not auto-repeating"""
        if not event.isAutoRepeat():
            self.manual_drive_signal.emit(event.text().lower(), True)

    def keyReleaseEvent(self, event):
        """Capture key releases to stop movement"""
        if not event.isAutoRepeat():
            self.manual_drive_signal.emit(event.text().lower(), False)
