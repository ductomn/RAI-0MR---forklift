import localization as l
import cv2

detection = l.Detection(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100))

while True:
    [corners, ids, rejected, frame] = detection.detect_markers()
    output_frame = detection.draw_markers(corners, ids, frame)

    cv2.imshow("Frame", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    