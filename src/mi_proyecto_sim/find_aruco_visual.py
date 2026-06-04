"""Find tiles with bright square floor markers (ArUco-style) using visual detection."""
import cv2, numpy as np, glob, os

imgs = sorted(glob.glob('/home/david/port_bot_sim_ws/src/mi_proyecto_sim/mision_output/55fotos/*.png'))

out_dir = '/home/david/port_bot_sim_ws/src/mi_proyecto_sim/mision_output/aruco_debug'
os.makedirs(out_dir, exist_ok=True)

candidates = []

for path in imgs:
    name = os.path.basename(path)
    img = cv2.imread(path)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    h, w = gray.shape

    # Adaptive threshold to isolate bright regions
    thresh = cv2.adaptiveThreshold(gray, 255,
        cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 31, -5)

    cnts, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    found = []
    for cnt in cnts:
        area = cv2.contourArea(cnt)
        if area < 300 or area > 20000:
            continue
        x, y, cw, ch = cv2.boundingRect(cnt)
        # Must be roughly square
        aspect = min(cw, ch) / max(cw, ch + 1e-3)
        if aspect < 0.5:
            continue
        # Must be bright inside
        mask = np.zeros_like(gray)
        cv2.drawContours(mask, [cnt], -1, 255, -1)
        mean_val = float(gray[mask > 0].mean())
        if mean_val < 160:
            continue
        # Must have internal contrast (dark cells inside the bright border)
        inner_x = x + cw//4; inner_y = y + ch//4
        inner_w = cw//2; inner_h = ch//2
        inner = gray[inner_y:inner_y+inner_h, inner_x:inner_x+inner_w]
        if inner.size == 0:
            continue
        inner_std = float(inner.std())
        if inner_std < 20:
            continue
        found.append((x, y, cw, ch, area, mean_val, inner_std))

    if found:
        candidates.append((name, found))
        vis = img.copy()
        for (x, y, cw, ch, area, mv, std) in found:
            cv2.rectangle(vis, (x,y), (x+cw,y+ch), (0,255,0), 2)
            cv2.putText(vis, f'a={int(area)} std={std:.0f}', (x, y-5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1)
        cv2.imwrite(f'{out_dir}/{name}', vis)
        print(f'{name}: {len(found)} candidate(s) → {[(c[0],c[1],c[2],c[3]) for c in found]}')

print(f'\nTotal tiles with candidates: {len(candidates)}')
