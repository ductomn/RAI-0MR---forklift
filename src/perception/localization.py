import cv2
import math
import numpy as np


class Detection:
    def __init__(self, dictionary=cv2.aruco.DICT_4X4_100):
        # Aruco marker detection setup
        self.dictionary = cv2.aruco.getPredefinedDictionary(dictionary)
        self.parameters = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.dictionary, self.parameters)

    def detect_markers(self, frame):
        # find markers
        corners, ids, rejected = self.detector.detectMarkers(frame)

        if ids is not None and len(ids) > 0:
            # sorting algoritmus smaller ID (realState is first) -> smaler indx
            sortedPairs = sorted(zip(ids.flatten(), corners))
            # unpack
            sortedIds, sortedCorners = zip(*sortedPairs)
            # convert back to lists (idk if needed just to be shure XD)
            sCorners = list(sortedCorners)
            sIDs = np.array(sortedIds, dtype=np.int32).reshape(-1, 1)

            return sCorners, sIDs, rejected, frame
        else:
            return corners, ids, rejected, frame

    def draw_markers(self, corners, ids, frame):
        output_image = cv2.aruco.drawDetectedMarkers(frame, corners, ids)
        return output_image

    def get_angle(self, corners_1set):
        """
        Calculate the orientation angle of the SINGLE passed marker in 2D

        This is a simplified approach that only considers the first two corners_1set of the marker.
        """
        marker = corners_1set[0]
        # print(f"marker: {marker}")

        angle = math.atan2(-(marker[1][1] - marker[0][1]), marker[1][0] - marker[0][0])
        # angle_deg = math.degrees(angle)

        # print(f"Orientation (angle in radians): {angle} radians")
        # print(f"Orientation (angle): {angle_deg} degrees")

        return angle  # angle_deg

    def get_center(self, corners_1set):
        """
        Calculate the center point of the SINGLE passed marker in 2D

        This is a simplified approach that only considers the first two corners_1set of the marker.
        """
        marker = corners_1set[0]
        # print(f"marker: {marker}")

        # Calculate the center point of the marker
        center_x = (marker[0][0] + marker[1][0] + marker[2][0] + marker[3][0]) / 4
        center_y = (marker[0][1] + marker[1][1] + marker[2][1] + marker[3][1]) / 4
        # print(f"Center (x, y): ({center_x}, {center_y})")

        return center_x, center_y

    def get_position_simple_mm(self, corners_1set, corners, markersize, frameHeight):
        center_x, center_y = self.get_center(corners_1set)
        angle_rad = self.get_angle(corners_1set)

        size_px = 0
        i = 0
        for m in corners:
            marker = m[0]
            size_px += (math.sqrt((marker[0][0] - marker[1][0])**2 + (marker[0][1] - marker[1][1])**2)
                    + math.sqrt((marker[1][0] - marker[2][0])**2 + (marker[1][1] - marker[2][1])**2)
                    + math.sqrt((marker[2][0] - marker[3][0])**2 + (marker[2][1] - marker[3][1])**2)
                    + math.sqrt((marker[3][0] - marker[0][0])**2 + (marker[3][1] - marker[0][1])**2)
                    ) / 4  # Average size in pixels
            i += 1
        size_px = size_px / i
        px_mm = size_px / markersize

        return [center_x / px_mm, (frameHeight - center_y) / px_mm, angle_rad]
    
    def resize_statespace_mm(self,corners, markersize, state_space):
        size_px = 0
        i = 0
        for m in corners:
            marker = m[0]
            size_px += (math.sqrt((marker[0][0] - marker[1][0])**2 + (marker[0][1] - marker[1][1])**2)
                    + math.sqrt((marker[1][0] - marker[2][0])**2 + (marker[1][1] - marker[2][1])**2)
                    + math.sqrt((marker[2][0] - marker[3][0])**2 + (marker[2][1] - marker[3][1])**2)
                    + math.sqrt((marker[3][0] - marker[0][0])**2 + (marker[3][1] - marker[0][1])**2)
                    ) / 4  # Average size in pixels
            i += 1
        size_px = size_px / i
        px_mm = size_px / markersize
        state_space_mm = [state_space[0] / px_mm, state_space[1] / px_mm]
        return state_space_mm, px_mm
