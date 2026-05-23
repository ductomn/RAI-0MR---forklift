import cv2
import depthai as dai


class Detection:
    def __init__(self, dictionary):
        self.dictionary = dictionary
        self.parameters = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.dictionary, self.parameters)

    def detect_markers(self, frame):
        [corners, ids, rejected] = self.detector.detectMarkers(frame)
        return corners, ids, rejected

    def draw_markers(self, corners, ids, frame):
        output_image = cv2.aruco.drawDetectedMarkers(frame, corners, ids)
        return output_image


class ImageProcessor:
    def __init__(self, res_x, res_y, fps):
        self.pipeline = dai.Pipeline()
        cam = self.pipeline.create(dai.node.Camera).build()
        self.video_queue = cam.requestOutput(
            size=(res_x, res_y),
            fps=fps,
        ).createOutputQueue()

    def start(self):
        self.pipeline.start()

    def stop(self):
        self.pipeline.stop()

    def is_running(self):
        return self.pipeline.isRunning()

    def get_frames(self):
        video = self.video_queue.get()
        assert isinstance(video, dai.ImgFrame)
        video_frame = video.getCvFrame()
        return video_frame
