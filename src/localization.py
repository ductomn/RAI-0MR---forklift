import cv2
import math
import numpy as np


class Detection:
    def __init__(self, dictionary):
        self.camera_settings = {
            "width": 640,
            "height": 480,
            "fps": 30
        }
        # self.capt = cv2.VideoCapture(1)
        # self.capt.set(cv2.CAP_PROP_FRAME_WIDTH, self.camera_settings["width"])
        # self.capt.set(cv2.CAP_PROP_FRAME_HEIGHT, self.camera_settings["height"])
        # self.capt.set(cv2.CAP_PROP_FPS, self.camera_settings["fps"])

        # Aruco marker detection setup
        self.dictionary = dictionary
        self.parameters = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(
            self.dictionary, self.parameters)

        # Camera calibration parameters
        # TODO look at https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html
        # TODO(calibration): Print a checkerboard (e.g. 9x6 inner corners), then
        # capture 20-30 frames at different distances/angles with this same camera.
        # TODO(calibration): Run cv2.calibrateCamera on those images and save
        # camera_matrix + dist_coeffs (npz/yaml) for this exact resolution.
        # TODO(calibration): Load saved camera_matrix + dist_coeffs here instead of
        # using focal_length heuristics. Re-calibrate if camera focus/zoom changes.
        # TODO(calibration): Keep units consistent (meters in solvePnP object points).
        focal_length = float(self.camera_settings["width"])
        self.camera_matrix = np.array(
            [[focal_length, 0.0, self.camera_settings["width"] / 2.0],
             [0.0, focal_length, self.camera_settings["height"] / 2.0],
             [0.0, 0.0, 1.0]],
            dtype=np.float32,
        )
        self.dist_coeffs = np.zeros((4, 1), dtype=np.float32)

    def capture_frame(self):
        ret, frame = self.capt.read()
        if not ret:
            print("Failed to capture frame from camera")
            return None
        return frame

    def detect_markers(self, frame):
        [corners, ids, rejected] = self.detector.detectMarkers(frame)
        return corners, ids, rejected, frame

    def draw_markers(self, corners, ids, frame):
        output_image = cv2.aruco.drawDetectedMarkers(frame, corners, ids)
        return output_image

    def get_angle(self, corners):
        """
        Calculate the orientation angle in 2D

        This is a simplified approach that only considers the first two corners of the marker.
        For more accurate 3D pose estimation, use get_pose
        """
        marker = corners[0][0]
        print(f"marker: {marker}")

        angle = -math.atan2(marker[1][1] - marker[0]
                            [1], marker[1][0] - marker[0][0])
        print(f"Orientation (angle in radians): {angle} radians")
        angle_deg = math.degrees(angle)
        print(f"Orientation (angle): {angle_deg} degrees")

        return angle_deg

    def _rotation_matrix_to_euler_angles(self, rotation_matrix):
        sy = math.sqrt(rotation_matrix[0, 0] ** 2 + rotation_matrix[1, 0] ** 2)
        singular = sy < 1e-6

        if not singular:
            roll = math.atan2(rotation_matrix[2, 1], rotation_matrix[2, 2])
            pitch = math.atan2(-rotation_matrix[2, 0], sy)
            yaw = math.atan2(rotation_matrix[1, 0], rotation_matrix[0, 0])
        else:
            roll = math.atan2(-rotation_matrix[1, 2], rotation_matrix[1, 1])
            pitch = math.atan2(-rotation_matrix[2, 0], sy)
            yaw = 0.0

        return roll, pitch, yaw

    def get_pose(self, corners, ids, frame, marker_size):
        """
        Calculate the 3D pose (translation and rotation) of the marker relative to the camera.

        Needs accurate camera calibration (camera_matrix + dist_coeffs) for correct results. 
        The marker_size should be in the same units as the object points (e.g. meters).
        """

        # TODO(calibration): Use self.camera_matrix/self.dist_coeffs loaded from
        # real calibration data; this per-frame estimate is only a placeholder.
        image_height, image_width = frame.shape[:2]
        focal_length = float(image_width)
        camera_matrix = np.array(
            [[focal_length, 0.0, image_width / 2.0],
             [0.0, focal_length, image_height / 2.0],
             [0.0, 0.0, 1.0]],
            dtype=np.float32,
        )
        dist_coeffs = np.zeros((4, 1), dtype=np.float32)

        marker_length_m = marker_size / 1000.0
        object_points = np.array(
            [
                [-marker_length_m / 2.0, marker_length_m / 2.0, 0.0],
                [marker_length_m / 2.0, marker_length_m / 2.0, 0.0],
                [marker_length_m / 2.0, -marker_length_m / 2.0, 0.0],
                [-marker_length_m / 2.0, -marker_length_m / 2.0, 0.0],
            ],
            dtype=np.float32,
        )

        for marker_index, marker_id in enumerate(ids.flatten()):
            marker_corners = corners[marker_index][0].astype(np.float32)
            success, rvec, tvec = cv2.solvePnP(
                object_points,
                marker_corners,
                camera_matrix,
                dist_coeffs,
                flags=cv2.SOLVEPNP_ITERATIVE,
            )

            print(f"marker id: {int(marker_id)}")

            if not success:
                print("solvePnP failed for this marker")
                continue

            rotation_matrix, _ = cv2.Rodrigues(rvec)
            roll, pitch, yaw = self._rotation_matrix_to_euler_angles(
                rotation_matrix)

            print(f"rvec: {rvec.ravel()}")
            print(f"tvec (m): {tvec.ravel()}")
            print(f"roll (deg): {math.degrees(roll)}")
            print(f"pitch (deg): {math.degrees(pitch)}")
            print(f"yaw (deg): {math.degrees(yaw)}")


def test_localization():
    dictionary = cv2.aruco.getPredefinedDictionary(
        cv2.aruco.DICT_4X4_50)
    detection = Detection(dictionary)

    # frame = detection.capture_frame()
    # if frame is None:
    #     return
    frame = cv2.imread("img/foto.jpg", cv2.IMREAD_COLOR)

    corners, ids, rejected, output_image = detection.detect_markers(frame)
    if ids is not None:
        output_image = detection.draw_markers(corners, ids, output_image)
        angle_deg = detection.get_angle(corners)
        print(f"Marker orientation: {angle_deg} degrees")

        marker_size_mm = 100  # Example marker size in millimeters
        detection.get_pose(corners, ids, output_image, marker_size_mm)

    output_image = cv2.resize(output_image, (800, 600))

    cv2.imshow("Detected Markers", output_image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


if __name__ == "__main__":
    test_localization()
