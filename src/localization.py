import cv2

class Detection:
    def __init__(self, dictionary):
        self.dictionary = dictionary
        self.parameters = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.dictionary, self.parameters)
        self.capt = cv2.VideoCapture(1)

    def detect_markers(self):
        ret, frame = self.capt.read()
        
        [corners, ids, rejected] = self.detector.detectMarkers(frame)
        return corners, ids, rejected, frame

    def draw_markers(self, corners, ids, frame):
        output_image = cv2.aruco.drawDetectedMarkers(frame, corners, ids)
        return output_image
    
