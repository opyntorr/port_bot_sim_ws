import cv2, numpy as np, glob

imgs = sorted(glob.glob('/home/david/port_bot_sim_ws/src/mi_proyecto_sim/mision_output/55fotos/*.png'))

all_dicts = {
    '4x4_50': cv2.aruco.DICT_4X4_50,
    '4x4_1000': cv2.aruco.DICT_4X4_1000,
    '5x5_50': cv2.aruco.DICT_5X5_50,
    '5x5_1000': cv2.aruco.DICT_5X5_1000,
    '6x6_50': cv2.aruco.DICT_6X6_50,
    '6x6_250': cv2.aruco.DICT_6X6_250,
    '7x7_50': cv2.aruco.DICT_7X7_50,
    '7x7_250': cv2.aruco.DICT_7X7_250,
    'ORIG': cv2.aruco.DICT_ARUCO_ORIGINAL,
    'APRIL_16h5': cv2.aruco.DICT_APRILTAG_16h5,
    'APRIL_25h9': cv2.aruco.DICT_APRILTAG_25h9,
    'APRIL_36h10': cv2.aruco.DICT_APRILTAG_36h10,
    'APRIL_36h11': cv2.aruco.DICT_APRILTAG_36h11,
}

params = cv2.aruco.DetectorParameters()
params.adaptiveThreshWinSizeMin = 3
params.adaptiveThreshWinSizeMax = 53
params.adaptiveThreshWinSizeStep = 4
params.minMarkerPerimeterRate = 0.01
params.maxMarkerPerimeterRate = 4.0
params.polygonalApproxAccuracyRate = 0.1

for dname, did in all_dicts.items():
    d = cv2.aruco.getPredefinedDictionary(did)
    det = cv2.aruco.ArucoDetector(d, params)
    hits = []
    for p in imgs:
        img = cv2.imread(p)
        corners, ids, _ = det.detectMarkers(img)
        if ids is not None:
            for i, c in zip(ids.flatten(), corners):
                sz = c[0].max(axis=0) - c[0].min(axis=0)
                hits.append(f"{p.split('/')[-1]}:id{i}({int(sz[0])}x{int(sz[1])}px)")
    if hits:
        print(f"{dname}: {hits}")

print("Done")
