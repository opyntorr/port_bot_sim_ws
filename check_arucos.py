import cv2
import glob

aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
parameters = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)

fotos = sorted(glob.glob('src/mi_proyecto_sim/mision_output/20260519_031434/fotos/*.png'))
for f in fotos:
    img = cv2.imread(f)
    corners, ids, _ = detector.detectMarkers(img)
    if ids is not None:
        print(f"{f}: {ids.flatten().tolist()}")
