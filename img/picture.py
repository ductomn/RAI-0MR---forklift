import cv2
import math
import numpy as np

##aruco generation
dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)
marker = cv2.aruco.generateImageMarker(dictionary,0,200,1)
cv2.imwrite("img/marker0.png",marker)
marker_size = 100

####################
###ArUco detector###
####################
input_image = cv2.imread("img/foto.jpg", cv2.IMREAD_COLOR)
parameters = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(dictionary,parameters)

###################
###video capture###
###################

##detection
[corners, ids, rejected] = detector.detectMarkers(input_image)
##view
output_image = cv2.aruco.drawDetectedMarkers(input_image, corners, ids)

cv2.imwrite("img/output.jpg", output_image)

print(f"corners: {corners}")
print(f"ids: {ids}")
# print(f"rejected: {rejected}")

px0 = math.sqrt((corners[0][0][0][0] - corners[0][0][1][0])**2 + (corners[0][0][0][1] - corners[0][0][1][1])**2)
px1 = math.sqrt((corners[0][0][1][0] - corners[0][0][2][0])**2 + (corners[0][0][1][1] - corners[0][0][2][1])**2)
px2 = math.sqrt((corners[0][0][2][0] - corners[0][0][3][0])**2 + (corners[0][0][2][1] - corners[0][0][3][1])**2)
px3 = math.sqrt((corners[0][0][3][0] - corners[0][0][0][0])**2 + (corners[0][0][3][1] - corners[0][0][0][1])**2)


centre1 = math.sqrt((corners[0][0][0][0] - corners[0][0][2][0])**2 + (corners[0][0][0][1] - corners[0][0][2][1])**2)


px_mm = (px0+px1+px2+px3)/(4*marker_size)

# print(f"px_mm: {px_mm}")

# print(f"ids[0][0]: {ids[0][0]}")

# Orientation (angle) calculation
marker = corners[0][0]
print(f"marker: {marker}")

angle = -math.atan2(marker[1][1] - marker[0][1], marker[1][0] - marker[0][0])
print(f"Orientation (angle in radians): {angle} radians")
angle = math.degrees(angle)
print(f"Orientation (angle): {angle} degrees")






def rotation_matrix_to_euler_angles(rotation_matrix):
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


image_height, image_width = input_image.shape[:2]
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
    roll, pitch, yaw = rotation_matrix_to_euler_angles(rotation_matrix)

    print(f"rvec: {rvec.ravel()}")
    print(f"tvec (m): {tvec.ravel()}")
    print(f"roll (deg): {math.degrees(roll)}")
    print(f"pitch (deg): {math.degrees(pitch)}")
    print(f"yaw (deg): {math.degrees(yaw)}")
