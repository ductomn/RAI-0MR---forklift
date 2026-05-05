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

        self.btn_showPath = QPushButton("Show path")
        self.btn_go = QPushButton("Go")
        self.btn_override = QPushButton("Override")

        # Connect button signals to slots (dummy slots for now)
        self.btn_showPath.clicked.connect(self.on_showPath_clicked)
        self.btn_go.clicked.connect(self.on_go_clicked)
        self.btn_override.clicked.connect(self.on_override_clicked)

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

    def on_showPath_clicked(self):
        print("Show path button clicked - Add path visualization logic here")

    def on_go_clicked(self):
        print("Go button clicked - Add forklift movement logic here")

    def on_override_clicked(self):
        print("Override button clicked - Add override logic here")
