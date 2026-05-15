import cv2
import math
import numpy as np
import depthai as dai


class Detection:
    def __init__(self, dictionary=cv2.aruco.DICT_4X4_100):
        self.camera_settings = {"width": 640, "height": 480, "fps": 30}
        # self.cam = cv2.VideoCapture(1)
        # self.cam.set(cv2.CAP_PROP_FRAME_WIDTH, self.camera_settings["width"])
        # self.cam.set(cv2.CAP_PROP_FRAME_HEIGHT, self.camera_settings["height"])
        # self.cam.set(cv2.CAP_PROP_FPS, self.camera_settings["fps"])

        # Aruco marker detection setup
        self.dictionary = cv2.aruco.getPredefinedDictionary(dictionary)
        self.parameters = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.dictionary, self.parameters)

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
            [
                [focal_length, 0.0, self.camera_settings["width"] / 2.0],
                [0.0, focal_length, self.camera_settings["height"] / 2.0],
                [0.0, 0.0, 1.0],
            ],
            dtype=np.float32,
        )
        self.dist_coeffs = np.zeros((4, 1), dtype=np.float32)

    def load_calibration(self, width, height, fps):
        # Assuming 'cam' is your depthai.device object
        calibration_handler = self.cam.readCalibration()
        # Get intrinsics for the RGB camera
        camera_matrix = calibration_handler.getCameraIntrinsics(
            dai.CameraBoardSocket.RGB, 1920, 1080
        )
        # Get distortion coefficients
        dist_coeffs = calibration_handler.getDistortionCoefficients(
            dai.CameraBoardSocket.RGB
        )

    def capture_frame(self):
        ret, frame = self.cam.read()
        if not ret:
            print("Failed to capture frame from camera")
            return None
        return frame

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
        For more accurate 3D pose estimation, use get_pose
        """
        marker = corners_1set[0]
        # print(f"marker: {marker}")

        angle = -math.atan2(marker[1][1] - marker[0][1], marker[1][0] - marker[0][0])
        # angle_deg = math.degrees(angle)

        # print(f"Orientation (angle in radians): {angle} radians")
        # print(f"Orientation (angle): {angle_deg} degrees")

        return angle  # angle_deg

    def get_center(self, corners_1set):
        """
        Calculate the center point of the SINGLE passed marker in 2D

        This is a simplified approach that only considers the first two corners_1set of the marker.
        For more accurate 3D pose estimation, use get_pose
        """
        marker = corners_1set[0]
        # print(f"marker: {marker}")

        # Calculate the center point of the marker
        center_x = (marker[0][0] + marker[1][0] + marker[2][0] + marker[3][0]) / 4
        center_y = (marker[0][1] + marker[1][1] + marker[2][1] + marker[3][1]) / 4
        # print(f"Center (x, y): ({center_x}, {center_y})")

        return center_x, center_y

    def get_position_simple(self, corners_1set):
        """
        Simple position (center) and orientation (angle) estimation in 2D
        """

        center_x, center_y = self.get_center(corners_1set)
        angle_deg = self.get_angle(corners_1set)

        return [center_x, center_y, angle_deg]

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
            [
                [focal_length, 0.0, image_width / 2.0],
                [0.0, focal_length, image_height / 2.0],
                [0.0, 0.0, 1.0],
            ],
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
            roll, pitch, yaw = self._rotation_matrix_to_euler_angles(rotation_matrix)

            print(f"rvec: {rvec.ravel()}")
            print(f"tvec (m): {tvec.ravel()}")
            print(f"roll (deg): {math.degrees(roll)}")
            print(f"pitch (deg): {math.degrees(pitch)}")
            print(f"yaw (deg): {math.degrees(yaw)}")


def test_localization():
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
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
