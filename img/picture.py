import cv2
import math

##aruco generation
dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)
marker = cv2.aruco.generateImageMarker(dictionary,0,200,1)
cv2.imwrite("marker0.png",marker)
marker_size = 100

####################
###ArUco detector###
####################
input_image = cv2.imread("foto.jpg", cv2.IMREAD_COLOR)
parameters = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(dictionary,parameters)

###################
###video capture###
###################

##detection
[corners, ids, rejected] = detector.detectMarkers(input_image)
##view
output_image = cv2.aruco.drawDetectedMarkers(input_image, corners, ids)

cv2.imwrite("output.jpg", output_image)

print(corners)
print(ids)
print(rejected)

px0 = math.sqrt((corners[0][0][0][0] - corners[0][0][1][0])**2 + (corners[0][0][0][1] - corners[0][0][1][1])**2)
px1 = math.sqrt((corners[0][0][1][0] - corners[0][0][2][0])**2 + (corners[0][0][1][1] - corners[0][0][2][1])**2)
px2 = math.sqrt((corners[0][0][2][0] - corners[0][0][3][0])**2 + (corners[0][0][2][1] - corners[0][0][3][1])**2)
px3 = math.sqrt((corners[0][0][3][0] - corners[0][0][0][0])**2 + (corners[0][0][3][1] - corners[0][0][0][1])**2)


centre1 = math.sqrt((corners[0][0][0][0] - corners[0][0][2][0])**2 + (corners[0][0][0][1] - corners[0][0][2][1])**2)


px_mm = (px0+px1+px2+px3)/(4*marker_size)

print(px_mm)

print(ids[0][0])