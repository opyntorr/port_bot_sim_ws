import cv2
import numpy as np

def main():
    img = cv2.imread('map-no_pers.png')
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    h, w = img.shape[:2]

    # 1. Detect grid (orange tape)
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    gm = cv2.inRange(hsv, (8, 80, 60), (28, 255, 255))
    k = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    gm = cv2.morphologyEx(gm, cv2.MORPH_CLOSE, k)

    # 2. Mask exterior (convex hull)
    ys, xs = np.where(gm > 0)
    if len(ys) > 0:
        pts = np.column_stack([xs, ys]).astype(np.float32)
        hull = cv2.convexHull(pts)
        hull_mask = np.zeros((h, w), dtype=np.uint8)
        cv2.fillPoly(hull_mask, [hull.astype(np.int32)], 255)
        
        # Pad grid (scale is ~1/4 of original 2800px)
        tape_pad_px = 8
        k_pad = np.ones((tape_pad_px*2+1, tape_pad_px*2+1), np.uint8)
        hull_mask = cv2.dilate(hull_mask, k_pad)
        interior_mask = hull_mask > 0
    else:
        interior_mask = np.ones((h, w), dtype=bool)
        hull_mask = np.ones((h, w), dtype=np.uint8)*255

    # 3. Obstacle detection (blue/cyan boxes)
    lab = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
    om = (lab[:, :, 2].astype(np.int16) < 110).astype(np.uint8) * 255
    om[~interior_mask] = 0

    occ_close_px = 3
    kc = np.ones((occ_close_px*2+1, occ_close_px*2+1), np.uint8)
    om = cv2.morphologyEx(om, cv2.MORPH_CLOSE, kc)
    om = cv2.morphologyEx(om, cv2.MORPH_OPEN, np.ones((3,3), np.uint8))

    # Geometric fitting for obstacles to get perfectly straight borders
    occ_shapes = np.zeros_like(om)
    cnts, _ = cv2.findContours(om, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for cnt in cnts:
        area = cv2.contourArea(cnt)
        if area < 25: # Drop small noise
            continue
        if area > 3500: # Large blob -> convex hull
            cv2.fillPoly(occ_shapes, [cv2.convexHull(cnt)], 255)
            continue
            
        perim = cv2.arcLength(cnt, True)
        circularity = (4 * np.pi * area / (perim * perim)) if perim > 0 else 0
        if circularity > 0.72:
            (cx_f, cy_f), radius = cv2.minEnclosingCircle(cnt)
            if np.pi * radius * radius <= 3.0 * area:
                cv2.circle(occ_shapes, (int(cx_f), int(cy_f)), int(radius), 255, -1)
                continue
        else:
            rect = cv2.minAreaRect(cnt)
            if rect[1][0] * rect[1][1] <= 3.0 * area:
                cv2.fillPoly(occ_shapes, [cv2.boxPoints(rect).astype(np.int32)], 255)
                continue
        cv2.fillPoly(occ_shapes, [cv2.convexHull(cnt)], 255)

    # 4. Wall mask (white netting)
    m_wall = cv2.inRange(hsv, (0, 0, 180), (179, 60, 255))
    kw = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    wm = cv2.morphologyEx(m_wall, cv2.MORPH_CLOSE, kw)
    wm[~interior_mask] = 0

    wall_boundary_px = 35
    k_wb = np.ones((wall_boundary_px*2+1, wall_boundary_px*2+1), np.uint8)
    deep = cv2.erode(hull_mask, k_wb) > 0
    near_boundary = interior_mask & ~deep
    wm[~near_boundary] = 0
    
    # Clean up small wall noise
    num_w, lbls_w, stats_w, _ = cv2.connectedComponentsWithStats(wm, connectivity=8)
    wm_clean = np.zeros_like(wm)
    for k in range(1, num_w):
        if stats_w[k, cv2.CC_STAT_AREA] >= 20:
            wm_clean[lbls_w == k] = 255

    # 5. Build final map
    # 255 (blanco) para área libre, 0 (negro) para obstáculos/paredes, 127 (gris) para exterior desconocido
    binarized = np.ones((h, w), dtype=np.uint8) * 255
    binarized[~interior_mask] = 127
    binarized[wm_clean > 0] = 0
    binarized[occ_shapes > 0] = 0

    # 6. Crop around grid bounds
    if len(ys) > 0:
        ys_hull, xs_hull = np.where(hull_mask > 0)
        x_min, x_max = xs_hull.min(), xs_hull.max()
        y_min, y_max = ys_hull.min(), ys_hull.max()
        cropped_bin = binarized[y_min:y_max+1, x_min:x_max+1]
    else:
        cropped_bin = binarized

    cv2.imwrite('map_recortado_binario.png', cropped_bin)
    print("Saved as map_recortado_binario.png")

if __name__ == "__main__":
    main()
