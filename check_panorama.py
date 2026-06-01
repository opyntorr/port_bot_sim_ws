import cv2

aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
parameters = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)

img = cv2.imread('src/mi_proyecto_sim/mision_output/stitching_pose_031434_viejo/mosaic_pose.png')
if img is not None:
    corners, ids, _ = detector.detectMarkers(img)
    if ids is not None:
        print(f"Panorama: {ids.flatten().tolist()}")
    else:
        print("Panorama: None")
