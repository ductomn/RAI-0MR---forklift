from PyQt6.QtGui import QImage
from PyQt6.QtCore import QThread, pyqtSignal
import sys
import numpy as np
from PyQt6.QtWidgets import (QApplication, QMainWindow, QLabel, QPushButton,
                             QVBoxLayout, QHBoxLayout, QWidget)
from PyQt6.QtCore import QThread, pyqtSignal, Qt
from PyQt6.QtGui import QImage, QPixmap


class MainWindow(QMainWindow):

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Forklift Control Interface")
        self.resize(700, 600)

        # Create widgets
        self.image_label = QLabel(self)
        self.image_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.image_label.setMinimumSize(640, 480)

        self.btn_forward = QPushButton("Forward")
        self.btn_stop = QPushButton("Stop")

        # Connect button signals to slots (dummy slots for now)
        self.btn_forward.clicked.connect(self.on_forward_clicked)
        self.btn_stop.clicked.connect(self.on_stop_clicked)

        # Setup Layouts
        vbox = QVBoxLayout()
        vbox.addWidget(self.image_label)

        hbox = QHBoxLayout()
        hbox.addWidget(self.btn_forward)
        hbox.addWidget(self.btn_stop)

        vbox.addLayout(hbox)

        main_widget = QWidget()
        main_widget.setLayout(vbox)
        self.setCentralWidget(main_widget)

    def display_image(self, qt_image):
        """Slot that receives the QImage from the worker thread and updates the GUI."""
        self.image_label.setPixmap(QPixmap.fromImage(qt_image))

    def on_forward_clicked(self):
        print("Forward button clicked - Add forklift throttle logic here")

    def on_stop_clicked(self):
        print("Stop button clicked - Add forklift stop logic here")
