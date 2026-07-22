from PyQt6.QtCore import QEvent, Qt, pyqtSignal
from PyQt6.QtGui import QPixmap
from PyQt6.QtWidgets import (
    QApplication,
    QComboBox,
    QFrame,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QPushButton,
    QScrollArea,
    QSizePolicy,
    QVBoxLayout,
    QWidget,
)


class MainWindow(QMainWindow):
    show_path_signal = pyqtSignal(bool)
    go_signal = pyqtSignal(bool)
    override_signal = pyqtSignal(bool)
    manual_drive_signal = pyqtSignal(str, bool)
    camera_refresh_signal = pyqtSignal()
    camera_connect_signal = pyqtSignal(str)
    camera_disconnect_signal = pyqtSignal()
    forklift_connect_signal = pyqtSignal(str)
    forklift_disconnect_signal = pyqtSignal()
    planner_changed_signal = pyqtSignal(int)
    emergency_stop_signal = pyqtSignal()

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Forklift Control Station")
        self.resize(1100, 680)
        self.setMinimumSize(800, 520)
        self._last_pixmap = None
        self._camera_connected = False
        self._forklift_connected = False
        self._markers_ready = False

        root = QWidget()
        root_layout = QVBoxLayout(root)
        root_layout.setContentsMargins(22, 18, 22, 22)
        root_layout.setSpacing(16)
        root_layout.addWidget(self._build_header())

        content = QHBoxLayout()
        content.setSpacing(18)
        content.addWidget(self._build_video_panel(), 3)
        content.addWidget(self._build_control_panel(), 2)
        root_layout.addLayout(content, 1)
        self.setCentralWidget(root)
        self._apply_style()
        self._update_control_availability()
        app = QApplication.instance()
        if app is not None:
            # Manual drive keys must still work when a button or input field
            # owns keyboard focus.
            app.installEventFilter(self)

    def _build_header(self):
        frame = QFrame()
        frame.setObjectName("header")
        layout = QHBoxLayout(frame)
        title_box = QVBoxLayout()
        title = QLabel("FORKLIFT CONTROL STATION")
        title.setObjectName("title")
        subtitle = QLabel("Perception  •  Path planning  •  Vehicle control")
        subtitle.setObjectName("subtitle")
        title_box.addWidget(title)
        title_box.addWidget(subtitle)
        layout.addLayout(title_box)
        layout.addStretch()
        self.btn_emergency = QPushButton("EMERGENCY STOP")
        self.btn_emergency.setObjectName("emergency")
        self.btn_emergency.clicked.connect(self._emergency_stop)
        layout.addWidget(self.btn_emergency)
        return frame

    def _build_video_panel(self):
        panel = QFrame()
        panel.setObjectName("panel")
        layout = QVBoxLayout(panel)
        top = QHBoxLayout()
        heading = QLabel("LIVE PERCEPTION")
        heading.setObjectName("sectionTitle")
        self.camera_status = self._status_label("Camera disconnected", False)
        top.addWidget(heading)
        top.addStretch()
        top.addWidget(self.camera_status)
        layout.addLayout(top)

        self.image_label = QLabel("Select a camera to begin")
        self.image_label.setObjectName("video")
        self.image_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        # Keep the video panel responsive on smaller or scaled displays.
        self.image_label.setMinimumSize(360, 270)
        self.image_label.setSizePolicy(
            QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding
        )
        layout.addWidget(self.image_label, 1)

        footer = QHBoxLayout()
        self.marker_status = self._status_label("Markers unavailable", False)
        self.planner_status = self._status_label("Planner idle", False)
        footer.addWidget(self.marker_status)
        footer.addWidget(self.planner_status, 1)
        layout.addLayout(footer)
        return panel

    def _build_control_panel(self):
        scroll = QScrollArea()
        scroll.setObjectName("controlScroll")
        scroll.setWidgetResizable(True)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        container = QWidget()
        layout = QVBoxLayout(container)
        layout.setContentsMargins(0, 0, 6, 0)
        layout.setSpacing(14)

        layout.addWidget(self._camera_card())
        layout.addWidget(self._connection_card())
        layout.addWidget(self._planner_card())
        layout.addWidget(self._operation_card())
        layout.addStretch()
        scroll.setWidget(container)
        return scroll

    def _card(self, title):
        frame = QFrame()
        frame.setObjectName("card")
        layout = QVBoxLayout(frame)
        heading = QLabel(title)
        heading.setObjectName("sectionTitle")
        layout.addWidget(heading)
        return frame, layout

    def _camera_card(self):
        frame, layout = self._card("CAMERA SOURCE")
        row = QHBoxLayout()
        self.camera_combo = QComboBox()
        self.camera_combo.setEditable(False)
        self.camera_combo.setSizeAdjustPolicy(
            QComboBox.SizeAdjustPolicy.AdjustToMinimumContentsLengthWithIcon
        )
        self.camera_combo.setMinimumContentsLength(10)
        self.camera_combo.setMinimumWidth(80)
        self.btn_refresh_camera = QPushButton("Refresh")
        self.btn_refresh_camera.clicked.connect(
            lambda: self.camera_refresh_signal.emit()
        )
        row.addWidget(self.camera_combo, 1)
        row.addWidget(self.btn_refresh_camera)
        layout.addLayout(row)
        self.btn_camera = QPushButton("Connect camera")
        self.btn_camera.clicked.connect(self._toggle_camera)
        layout.addWidget(self.btn_camera)
        return frame

    def _connection_card(self):
        frame, layout = self._card("FORKLIFT CONNECTION")
        self.host_input = QLineEdit("192.168.4.1")
        self.host_input.setPlaceholderText("IP address or ws:// URI")
        layout.addWidget(self.host_input)
        row = QHBoxLayout()
        self.btn_forklift = QPushButton("Connect forklift")
        self.btn_forklift.clicked.connect(self._toggle_forklift)
        self.forklift_status = self._status_label("Disconnected", False)
        row.addWidget(self.btn_forklift, 1)
        row.addWidget(self.forklift_status)
        layout.addLayout(row)
        return frame

    def _planner_card(self):
        frame, layout = self._card("PATH PLANNER")
        self.planner_combo = QComboBox()
        self.planner_combo.setSizeAdjustPolicy(
            QComboBox.SizeAdjustPolicy.AdjustToMinimumContentsLengthWithIcon
        )
        self.planner_combo.setMinimumContentsLength(10)
        self.planner_combo.addItems(("Hybrid A*", "Kinodynamic RRT", "Dijkstra"))
        self.planner_combo.currentIndexChanged.connect(
            self.planner_changed_signal.emit
        )
        layout.addWidget(self.planner_combo)
        note = QLabel("Changing planner safely stops motion and clears the current path.")
        note.setObjectName("hint")
        note.setWordWrap(True)
        layout.addWidget(note)
        return frame

    def _operation_card(self):
        frame, layout = self._card("OPERATION")
        grid = QGridLayout()
        self.btn_show_path = QPushButton("Show path")
        self.btn_show_path.setCheckable(True)
        self.btn_show_path.toggled.connect(self.show_path_signal)
        self.btn_go = QPushButton("Start autonomous")
        self.btn_go.setCheckable(True)
        self.btn_go.toggled.connect(self._go_toggled)
        self.btn_override = QPushButton("Manual override")
        self.btn_override.setCheckable(True)
        self.btn_override.toggled.connect(self._override_toggled)
        grid.addWidget(self.btn_show_path, 0, 0)
        grid.addWidget(self.btn_go, 0, 1)
        grid.addWidget(self.btn_override, 1, 0, 1, 2)
        layout.addLayout(grid)
        keys = QLabel("DRIVE W/A/S/D   MAST J/K   TILT H/L")
        keys.setObjectName("keyHelp")
        keys.setWordWrap(True)
        keys.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(keys)
        return frame

    def _status_label(self, text, healthy):
        label = QLabel(text)
        label.setObjectName("status")
        label.setProperty("healthy", healthy)
        return label

    def set_camera_sources(self, sources):
        current = self.camera_combo.currentData()
        self.camera_combo.clear()
        for label, source in sources:
            self.camera_combo.addItem(label, source)
        if current:
            index = self.camera_combo.findData(current)
            if index >= 0:
                self.camera_combo.setCurrentIndex(index)

    def display_image(self, image):
        self._last_pixmap = QPixmap.fromImage(image)
        self._scale_video()

    def set_camera_status(self, text, connected):
        self._camera_connected = connected
        self._set_status(self.camera_status, text, connected)
        self.btn_camera.setText("Disconnect camera" if connected else "Connect camera")
        if not connected:
            self._markers_ready = False
            self._set_status(self.marker_status, "Markers unavailable", False)
        self._update_control_availability()

    def set_forklift_status(self, text, connected):
        self._forklift_connected = connected
        if not connected and self.btn_override.isChecked():
            self.btn_override.setChecked(False)
        self._set_status(self.forklift_status, text, connected)
        self.btn_forklift.setText(
            "Disconnect forklift" if connected else "Connect forklift"
        )
        self._update_control_availability()

    def set_marker_status(self, text, ready):
        self._markers_ready = ready
        self._set_status(self.marker_status, text, ready)
        self._update_control_availability()

    def set_planner_status(self, text, healthy):
        self._set_status(self.planner_status, text, healthy)

    def set_go_checked(self, checked):
        self.btn_go.blockSignals(True)
        self.btn_go.setChecked(checked)
        self.btn_go.setText("Stop autonomous" if checked else "Start autonomous")
        self.btn_go.blockSignals(False)

    def _set_status(self, label, text, healthy):
        label.setText(text)
        label.setProperty("healthy", healthy)
        label.style().unpolish(label)
        label.style().polish(label)

    def _toggle_camera(self):
        if self._camera_connected:
            self.camera_disconnect_signal.emit()
            return
        source = self.camera_combo.currentData()
        if source:
            self.camera_connect_signal.emit(str(source))

    def _toggle_forklift(self):
        if self._forklift_connected:
            self.forklift_disconnect_signal.emit()
        else:
            host = self.host_input.text().strip()
            if host:
                self.forklift_connect_signal.emit(host)

    def _go_toggled(self, checked):
        self.btn_go.setText("Stop autonomous" if checked else "Start autonomous")
        self.go_signal.emit(checked)

    def _override_toggled(self, checked):
        self.btn_override.setText(
            "Disable manual override" if checked else "Manual override"
        )
        self.override_signal.emit(checked)
        self._update_control_availability()

    def _emergency_stop(self):
        self.set_go_checked(False)
        self.btn_override.setChecked(False)
        self.emergency_stop_signal.emit()

    def _update_control_availability(self):
        autonomous_ready = (
            self._camera_connected
            and self._forklift_connected
            and self._markers_ready
            and not self.btn_override.isChecked()
        )
        self.btn_go.setEnabled(autonomous_ready or self.btn_go.isChecked())
        self.btn_override.setEnabled(self._forklift_connected)

    def _scale_video(self):
        if self._last_pixmap is None:
            return
        target = self.image_label.size()
        self.image_label.setPixmap(
            self._last_pixmap.scaled(
                target,
                Qt.AspectRatioMode.KeepAspectRatio,
                Qt.TransformationMode.SmoothTransformation,
            )
        )

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self._scale_video()

    def eventFilter(self, watched, event):
        if self.btn_override.isChecked() and event.type() in (
            QEvent.Type.KeyPress,
            QEvent.Type.KeyRelease,
        ):
            key = event.text().lower()
            if key in {"w", "a", "s", "d", "j", "k", "h", "l"}:
                if not event.isAutoRepeat():
                    self.manual_drive_signal.emit(
                        key, event.type() == QEvent.Type.KeyPress
                    )
                return True
        return super().eventFilter(watched, event)

    def _apply_style(self):
        self.setStyleSheet(
            """
            QMainWindow, QWidget { background: #10151c; color: #e7edf5; font-family: sans-serif; }
            QLabel { background: transparent; }
            #header, #panel, #card { background: #18212b; border: 1px solid #293746; border-radius: 10px; }
            #header { border-left: 4px solid #f3a712; }
            #title { font-size: 22px; font-weight: 800; letter-spacing: 1px; }
            #subtitle, #hint { color: #8fa0b3; font-size: 12px; }
            #sectionTitle { color: #b8c5d3; font-size: 12px; font-weight: 700; letter-spacing: 1px; }
            #video { background: #090d12; border: 1px solid #304052; border-radius: 8px; color: #667789; }
            #status { background: #2b2023; color: #ff8e8e; border: 1px solid #65343b; border-radius: 10px; padding: 4px 9px; font-size: 11px; }
            #status[healthy="true"] { background: #142d26; color: #64d9a6; border-color: #28654e; }
            QPushButton { background: #263442; border: 1px solid #3b4d5f; border-radius: 6px; padding: 9px 12px; font-weight: 600; }
            QPushButton:hover { background: #304153; border-color: #5e7489; }
            QPushButton:checked { background: #9a6700; border-color: #f3a712; color: white; }
            QPushButton:disabled { color: #596674; background: #1b242d; border-color: #26333f; }
            #emergency { background: #a4262c; border-color: #dd4c53; padding: 12px 20px; }
            #emergency:hover { background: #c42f37; }
            QComboBox, QLineEdit { background: #0f161e; border: 1px solid #354656; border-radius: 6px; padding: 8px; selection-background-color: #9a6700; }
            QComboBox::drop-down { border: none; width: 28px; }
            QComboBox QAbstractItemView { background: #18212b; color: #e7edf5; selection-background-color: #9a6700; }
            #keyHelp { background: #0f161e; color: #91a2b4; border-radius: 5px; padding: 9px; font-family: monospace; }
            #controlScroll { border: none; background: transparent; }
            QScrollBar:vertical { background: #111820; width: 8px; }
            QScrollBar::handle:vertical { background: #3b4d5f; border-radius: 4px; }
            """
        )
