import cv2
import depthai as dai
import numpy as np
import localization as loc

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
            #CAMERA QUEUE
            cam = self.pipeline.create(dai.node.Camera).build()
            self.video_queue = cam.requestOutput(size=(res_x, res_y),fps=fps).createOutputQueue()

            #STEREO CAMERAS QUEUE
            mono_left = self.pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
            mono_right = self.pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)
            stereo = self.pipeline.create(dai.node.StereoDepth)

            #stereo.setDepthAlign(self.video_queue)

            mono_left_out = mono_left.requestOutput(size=(res_x, res_y), fps=fps)
            mono_right_out = mono_right.requestOutput(size=(res_x, res_y), fps=fps)
            mono_left_out.link(stereo.left)
            mono_right_out.link(stereo.right)
            
            stereo.setRectification(True)
            stereo.setLeftRightCheck(True)
            self.depth_queue = stereo.depth.createOutputQueue()

            self.colorMap = cv2.applyColorMap(np.arange(256, dtype=np.uint8), cv2.COLORMAP_JET)
            self.colorMap[0] = [0, 0, 0]

            point_cloud = self.pipeline.create(dai.node.PointCloud)
            stereo.depth.link(point_cloud.inputDepth)
            self.point_cloud_queue = point_cloud.outputPointCloud.createOutputQueue()

    def start(self):
            self.pipeline.start()

    def stop(self):
            self.pipeline.stop()

    def is_running(self):
            return self.pipeline.isRunning()


    def get_frames(self):
        depth = self.depth_queue.get()
        video = self.video_queue.get()
        points = self.point_cloud_queue.get()
        assert isinstance(depth, dai.ImgFrame) & isinstance(video, dai.ImgFrame)
        depth_frame = depth.getFrame()
        video_frame = video.getCvFrame()
        points_frame = points.getPoints()
        return video_frame, depth_frame, points_frame