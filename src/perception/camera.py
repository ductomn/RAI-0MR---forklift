import glob
import os
from typing import List, Tuple

import cv2

try:
    import depthai as dai
except ImportError:  # OAK support is optional at runtime.
    dai = None


CameraSource = Tuple[str, str]


def _linux_capture_devices() -> List[CameraSource]:
    """List V4L2 capture nodes while excluding their metadata companions."""
    sources: List[CameraSource] = []
    for path in sorted(glob.glob("/dev/video*")):
        device = os.path.basename(path)
        sysfs_directory = os.path.join("/sys/class/video4linux", device)
        index_path = os.path.join(sysfs_directory, "index")
        name_path = os.path.join(sysfs_directory, "name")

        try:
            with open(index_path, encoding="utf-8") as index_file:
                # UVC devices normally expose capture as index 0 and metadata
                # as index 1. Metadata nodes cannot return OpenCV frames.
                if index_file.read().strip() != "0":
                    continue
            with open(name_path, encoding="utf-8") as name_file:
                name = name_file.read().strip()
        except OSError:
            name = "USB camera"

        sources.append((f"{name} ({path})", f"opencv:{path}"))
    return sources


def _oak_available() -> bool:
    if dai is None:
        return False
    try:
        return bool(dai.Device.getAllAvailableDevices())
    except Exception:
        return False


def discover_camera_sources() -> List[CameraSource]:
    """Return currently available camera choices without locking devices."""
    sources: List[CameraSource] = []

    if os.name == "posix":
        sources.extend(_linux_capture_devices())
    else:
        # OpenCV has no portable, non-invasive device enumeration API.
        sources.extend(
            (f"OpenCV camera {index}", f"opencv:{index}") for index in range(4)
        )

    if _oak_available():
        sources.append(("OAK camera", "oak"))

    if not sources:
        # Keep an editable fallback for platforms where discovery is unavailable.
        sources.append(("OpenCV camera 0", "opencv:0"))

    return sources


class OpenCVCamera:
    def __init__(self, source):
        if isinstance(source, str) and source.isdigit():
            source = int(source)
        self.capture = cv2.VideoCapture(source)
        # Prefer a compressed, modest-resolution stream. Many USB cameras fall
        # back to high-bandwidth YUYV otherwise, which can look torn or update
        # in bands when the USB link cannot keep up.
        self.capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.capture.set(cv2.CAP_PROP_FPS, 30)
        # A one-frame backend buffer avoids displaying stale frames when image
        # processing briefly takes longer than the camera frame interval.
        self.capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    def is_opened(self) -> bool:
        return self.capture.isOpened()

    def read(self):
        return self.capture.read()

    def close(self) -> None:
        self.capture.release()


class OakCamera:
    def __init__(self, res_x=640, res_y=480, fps=30):
        if dai is None:
            raise RuntimeError("DepthAI is not installed; OAK-D camera is unavailable.")

        self.pipeline = dai.Pipeline()
        camera = self.pipeline.create(dai.node.Camera).build()
        self.video_queue = camera.requestOutput(
            size=(res_x, res_y), fps=fps
        ).createOutputQueue()
        self.pipeline.start()

    def is_opened(self) -> bool:
        return self.pipeline.isRunning()

    def read(self):
        if not self.is_opened():
            return False, None
        frame = self.video_queue.get().getCvFrame()
        return frame is not None, frame

    def close(self) -> None:
        if self.pipeline.isRunning():
            self.pipeline.stop()


def open_camera(source: str):
    """Open a serialized source returned by :func:`discover_camera_sources`."""
    source = source.strip()
    if source == "oak":
        return OakCamera()

    if source.startswith("opencv:"):
        source = source.split(":", 1)[1]

    return OpenCVCamera(source)


# Backward-compatible name used by older scripts in this repository.
class ImageProcessor(OakCamera):
    def start(self):
        # OakCamera starts its pipeline during initialization.
        return None

    def stop(self):
        self.close()

    def is_running(self):
        return self.is_opened()

    def get_frames(self):
        ok, frame = self.read()
        if not ok:
            raise RuntimeError("Could not read a frame from the OAK-D camera.")
        return frame
