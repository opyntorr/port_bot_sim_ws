#!/usr/bin/env python3
"""
Heatmap "fresón" de la zona segura del mapa de mision.

Carga mapa_mision.pgm, calcula distance transform (distancia al obstaculo
mas cercano para cada celda libre), lo dibuja con colormap inferno y rocia
particulas concentradas en las zonas mas seguras. Marca ademas las
posiciones del carrito y la meta leidas de arucos.yaml.
"""
from pathlib import Path
import numpy as np
import yaml
import cv2
import matplotlib.pyplot as plt
from matplotlib.colors import LinearSegmentedColormap
from matplotlib import patheffects

ROOT = Path(__file__).parent
MAPS = ROOT / "src" / "mi_proyecto_sim" / "maps"

# --- Cargar mapa y metadatos ---
with open(MAPS / "mapa_mision.yaml") as f:
    meta = yaml.safe_load(f)

res = float(meta["resolution"])
ox, oy = float(meta["origin"][0]), float(meta["origin"][1])

pgm = cv2.imread(str(MAPS / meta["image"]), cv2.IMREAD_GRAYSCALE)
# Convencion ROS: pgm row 0 = top-left; el origen del world esta abajo-izq.
pgm = cv2.flip(pgm, 0)
h, w = pgm.shape

# Libre = blanco (254), ocupado = negro (0), unknown = 205.
free_mask = pgm > 230
obstacle_mask = pgm < 100  # negro -> obstaculo
unknown_mask = ~(free_mask | obstacle_mask)

# --- Distance transform: distancia (en celdas) al obstaculo mas cercano ---
binary_free = (~obstacle_mask).astype(np.uint8) * 255
dist_px = cv2.distanceTransform(binary_free, cv2.DIST_L2, 5)
dist_m = dist_px * res
# Solo nos interesa la "seguridad" en zona libre conocida; lo demas a 0
safety = np.where(free_mask, dist_m, 0.0)

# --- Setup figura "fresón": fondo negro, glow ---
plt.rcParams["font.family"] = "DejaVu Sans"
fig, ax = plt.subplots(figsize=(11, 11), facecolor="#0a0a12")
ax.set_facecolor("#0a0a12")

# Colormap custom (negro → magenta → naranja → amarillo) — vibe synthwave
cmap = LinearSegmentedColormap.from_list(
    "neon_safe",
    [
        (0.00, "#0a0a12"),
        (0.08, "#1a0030"),
        (0.25, "#7a00b8"),
        (0.50, "#e63f8a"),
        (0.75, "#ff8c42"),
        (1.00, "#fff7b3"),
    ],
)

# Extension en metros para que los ejes esten en coordenadas reales
extent = [ox, ox + w * res, oy, oy + h * res]
im = ax.imshow(safety, cmap=cmap, origin="lower", extent=extent,
               interpolation="bilinear", alpha=0.92)

# Halo (overlay con blur para glow)
blurred = cv2.GaussianBlur(safety, (0, 0), sigmaX=8)
ax.imshow(blurred, cmap=cmap, origin="lower", extent=extent,
          interpolation="bilinear", alpha=0.35)

# Contornos isolineas (cinturones de seguridad)
levels = np.linspace(safety.max() * 0.15, safety.max() * 0.95, 6)
ax.contour(safety, levels=levels, extent=extent, origin="lower",
           colors="#ffffff", alpha=0.18, linewidths=0.7)

# --- Particulas ponderadas por seguridad ---
rng = np.random.default_rng(42)
flat_safety = safety.ravel()
probs = flat_safety / flat_safety.sum() if flat_safety.sum() > 0 else None
n_particles = 4500
idx = rng.choice(flat_safety.size, size=n_particles, replace=True, p=probs)
py_idx, px_idx = np.unravel_index(idx, safety.shape)
# Anadir jitter sub-pixel para que no se vean en grilla
px_x = ox + (px_idx + rng.random(n_particles)) * res
py_y = oy + (py_idx + rng.random(n_particles)) * res
# Tamano y alpha varian con la seguridad local
sizes = 2 + (safety[py_idx, px_idx] / safety.max()) * 28
alphas = 0.25 + 0.7 * (safety[py_idx, px_idx] / safety.max())

ax.scatter(px_x, py_y, s=sizes, c="#ffffff", alpha=alphas,
           edgecolors="none", zorder=3)

# Capa extra: particulas muy brillantes pero pocas, en el "core" seguro
core_mask = safety > safety.max() * 0.85
core_y, core_x = np.where(core_mask)
if len(core_x) > 0:
    sel = rng.choice(len(core_x), size=min(200, len(core_x)), replace=False)
    ax.scatter(
        ox + (core_x[sel] + rng.random(len(sel))) * res,
        oy + (core_y[sel] + rng.random(len(sel))) * res,
        s=60, c="#fff7b3", alpha=0.9,
        edgecolors="#ffffff", linewidths=0.5, zorder=4,
    )

# --- ArUcos: carrito (start) y meta (goal) ---
arucos_path = MAPS / "arucos.yaml"
if arucos_path.exists():
    with open(arucos_path) as f:
        arucos = yaml.safe_load(f)
    color_map = {"carrito": "#00f7ff", "meta": "#ff4081"}
    for name, info in arucos.items():
        x, y = info["world_x"], info["world_y"]
        c = color_map.get(name, "#ffffff")
        ax.scatter([x], [y], s=380, c=c, alpha=0.25, edgecolors="none", zorder=5)
        ax.scatter([x], [y], s=140, c=c, edgecolors="white", linewidths=2.0, zorder=6)
        label = ax.text(x + 0.08, y + 0.08, name.upper(),
                        color=c, fontsize=12, fontweight="bold", zorder=7)
        label.set_path_effects([patheffects.withStroke(linewidth=2.5,
                                                       foreground="#000000")])

# --- Estilo del marco ---
ax.set_xlabel("x [m]", color="#cccccc", fontsize=11)
ax.set_ylabel("y [m]", color="#cccccc", fontsize=11)
ax.tick_params(colors="#cccccc")
for spine in ax.spines.values():
    spine.set_color("#3a3a4a")
ax.set_title("SAFE ZONE / clearance map",
             color="#fff7b3", fontsize=18, fontweight="bold", pad=14,
             loc="left")
ax.text(0.99, 1.01, "// distance-to-obstacle weighted particle field",
        transform=ax.transAxes, ha="right", va="bottom",
        color="#7a7a90", fontsize=9, fontstyle="italic")

# Colorbar
cbar = fig.colorbar(im, ax=ax, fraction=0.038, pad=0.02)
cbar.set_label("clearance [m]", color="#cccccc")
cbar.ax.yaxis.set_tick_params(color="#cccccc")
for t in cbar.ax.get_yticklabels():
    t.set_color("#cccccc")
cbar.outline.set_edgecolor("#3a3a4a")

ax.set_aspect("equal")
plt.tight_layout()

out_path = ROOT / "safe_zone_heatmap.png"
plt.savefig(out_path, dpi=180, facecolor=fig.get_facecolor())
print(f"Guardado: {out_path}")
