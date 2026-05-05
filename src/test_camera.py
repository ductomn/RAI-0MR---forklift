import cv2
import camera as cam

detector = cam.Detection(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100))
img = cam.ImageProcessor(640, 480, 5)
img.start()
while img.is_running():
    video, depth, points = img.get_frames()
    cv2.imshow("disparity", depth)
    
    [corners, ids, rejected] = detector.detect_markers(video)
    video = detector.draw_markers(corners, ids, video)
    #print(points[corners[0][0][0], corners[0][0][1]])
    print("fsxgfhn")

    cv2.imshow("video", video)
    key = cv2.waitKey(1)
    
    if key == ord('q'):
        img.stop()
        break
"""
# Create pipeline
with dai.Pipeline() as pipeline:
    # Define source and output
    ##CAM A - RGB
    cam = pipeline.create(dai.node.Camera).build()
    videoQueue = cam.requestOutput(size=(640, 480),fps=5).createOutputQueue()

    ##STEREO CAMERAS
    monoLeft = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
    monoRight = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)
    stereo = pipeline.create(dai.node.StereoDepth)

    monoLeftOut = monoLeft.requestOutput(size=(640, 480), fps=5)
    monoRightOut = monoRight.requestOutput(size=(640, 480), fps=5)
    monoLeftOut.link(stereo.left)
    monoRightOut.link(stereo.right)

    stereo.setRectification(True)
    stereo.setExtendedDisparity(True)
    stereo.setLeftRightCheck(True)
    disparityQueue = stereo.disparity.createOutputQueue()

    colorMap = cv2.applyColorMap(np.arange(256, dtype=np.uint8), cv2.COLORMAP_JET)
    colorMap[0] = [0, 0, 0]  # to make zero-disparity pixels black

    # Connect to device and start pipeline
    pipeline.start()
    while pipeline.isRunning():
        disparity = disparityQueue.get()
        video = videoQueue.get()
        assert isinstance(disparity, dai.ImgFrame) & isinstance(video, dai.ImgFrame)
        disparityFrame = disparity.getFrame()
        videoFrame = video.getCvFrame()
        maxDisparity = max(maxDisparity, np.max(disparityFrame))
        colorizedDisparity = cv2.applyColorMap(((disparityFrame) * 255).astype(np.uint8), colorMap)
        cv2.imshow("disparity", disparityFrame)
        
        [corners, ids, rejected] = detector.detect_markers(videoFrame)
        videoFrame = detector.draw_markers(corners, ids, videoFrame)

        cv2.imshow("video", videoFrame)
        key = cv2.waitKey(1)
        if key == ord('q'):
            pipeline.stop()
            break """
