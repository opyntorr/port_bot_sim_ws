#!/usr/bin/env python3
# Materiales del JetAuto para Isaac Sim (USD/MDL).
#
# El importador de URDF de Isaac NO trae los <material>/<gazebo> del URDF/Gazebo
# (ademas las mallas STL no llevan color), por eso el robot sale BLANCO. Aqui, tras
# importar, creamos 2 materiales y los bindeamos por nombre de link:
#     base_link            -> aluminio anodizado verde (metalico)
#     todo lo demas visible -> plastico negro mate
#
# Primario: OmniPBR (MDL, mejor look metalico). Fallback: UsdPreviewSurface (USD
# puro, siempre funciona en el render RTX). Idempotente: re-bindea sin duplicar.
#
# Uso (en scene_agv.py / scene_gridmap.py, despues de obtener `stage`):
#     from jetauto_materials import apply_jetauto_materials
#     apply_jetauto_materials(stage, prim_path)
import os
import shutil
from pxr import Gf, Sdf, UsdGeom, UsdShade

_HERE = os.path.dirname(os.path.abspath(__file__))
# La textura de grano se VERSIONA en el repo (assets/textures/), pero Omniverse no resuelve
# bien asset paths con ESPACIOS (la carpeta "...jetauto Vilchis" tiene uno). Para ser PORTABLE:
# copiamos la textura del repo a un cache SIN espacios (~/.local/share/...) y usamos esa ruta.
_REPO_GRAIN = os.path.join(_HERE, "assets", "textures", "grain_normal.png")
_GRAIN = os.path.join(os.path.expanduser("~"), ".local", "share", "jetauto_isaac", "grain_normal.png")
try:
    if not os.path.exists(_GRAIN) and os.path.exists(_REPO_GRAIN):
        os.makedirs(os.path.dirname(_GRAIN), exist_ok=True)
        shutil.copyfile(_REPO_GRAIN, _GRAIN)
except Exception:
    _GRAIN = _REPO_GRAIN if os.path.exists(_REPO_GRAIN) else _GRAIN

# spec = {color, metallic, roughness, extra:{input_OmniPBR: valor}}
# 'extra' solo aplica en OmniPBR (el fallback UsdPreviewSurface lo ignora).
# Verde anodizado INTERMEDIO: satinado metálico + grano FINO y SUTIL (que se note el
# anodizado sin el sparkle fuerte). Dials: roughness=brillo (menos=espejo), bump_factor=grano.
GREEN_ANODIZED = {
    "color": (0.065, 0.48, 0.11), "metallic": 0.85, "roughness": 0.33,
    "extra": {
        "project_uvw": True,
        "world_or_object": False,
        "texture_scale": (60.0, 60.0),   # grano muy fino
        "normalmap_texture": _GRAIN,
        "bump_factor": 0.2,              # grano sutil (0.6 era el fuerte)
    },
}
MATTE_BLACK = {"color": (0.02, 0.02, 0.02), "metallic": 0.0, "roughness": 0.85, "extra": {}}

# Clasificacion por nombre de link (substring de la ruta del prim).
GREEN_LINKS = ("base_link",)
# Links sin visual (frames) — no tienen mallas, se ignoran solos.


def _set_extra(shader, name, val):
    """Crea un input de OmniPBR infiriendo el tipo del valor."""
    if val is None:
        return
    if name.endswith("_texture"):
        if not os.path.exists(val):       # textura ausente (copia sin assets) -> omitir
            return
        shader.CreateInput(name, Sdf.ValueTypeNames.Asset).Set(Sdf.AssetPath(val))
    elif isinstance(val, bool):           # bool antes que int (bool es subclase de int)
        shader.CreateInput(name, Sdf.ValueTypeNames.Bool).Set(val)
    elif isinstance(val, (tuple, list)) and len(val) == 2:
        shader.CreateInput(name, Sdf.ValueTypeNames.Float2).Set(Gf.Vec2f(*val))
    elif isinstance(val, (int, float)):
        shader.CreateInput(name, Sdf.ValueTypeNames.Float).Set(float(val))


def _make_omnipbr(stage, path, spec):
    """Crea un material OmniPBR (MDL). Lanza si la API/comando no existe."""
    import omni.kit.commands
    omni.kit.commands.execute(
        "CreateMdlMaterialPrim",
        mtl_url="OmniPBR.mdl",
        mtl_name="OmniPBR",
        mtl_path=Sdf.Path(path),
    )
    mat = UsdShade.Material.Get(stage, path)
    if not mat:
        raise RuntimeError("OmniPBR no creo el material")
    # encontrar el shader hijo (no asumir el nombre)
    shader = None
    for child in stage.GetPrimAtPath(path).GetChildren():
        if child.IsA(UsdShade.Shader):
            shader = UsdShade.Shader(child)
            break
    if shader is None:
        raise RuntimeError("OmniPBR sin shader hijo")
    shader.CreateInput("diffuse_color_constant", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*spec["color"]))
    shader.CreateInput("metallic_constant", Sdf.ValueTypeNames.Float).Set(float(spec["metallic"]))
    shader.CreateInput("reflection_roughness_constant", Sdf.ValueTypeNames.Float).Set(float(spec["roughness"]))
    for name, val in spec.get("extra", {}).items():
        _set_extra(shader, name, val)
    return mat


def _make_preview_surface(stage, path, spec):
    """Fallback: material UsdPreviewSurface (USD nativo, sin el grano de 'extra')."""
    mat = UsdShade.Material.Define(stage, path)
    shader = UsdShade.Shader.Define(stage, path + "/Shader")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*spec["color"]))
    shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(float(spec["metallic"]))
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(float(spec["roughness"]))
    mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    return mat


def _make_material(stage, path, spec):
    try:
        return _make_omnipbr(stage, path, spec)
    except Exception as e:  # noqa: BLE001
        print(f"[materials] OmniPBR no disponible ({e}); uso UsdPreviewSurface")
        return _make_preview_surface(stage, path, spec)


def apply_jetauto_materials(stage, prim_path, green_links=GREEN_LINKS):
    """Crea y bindea los materiales del JetAuto. Devuelve (n_mallas, n_links).

    CLAVE: el importador de Isaac mete las mallas visuales como INSTANCIAS de prototipos
    (`visuals.proto_mesh_*`) y el material por defecto vive DENTRO del prototipo. Un bind a
    nivel de link NO lo vence (la instancia resuelve su material desde el prototipo). Por eso
    aquí primero DES-INSTANCIAMOS las mallas (SetInstanceable(False) -> aparecen como prims
    reales en el stage) y luego bindeamos DIRECTO en cada malla, que sí pisa el material del
    importador.
    """
    robot_root = "/" + str(prim_path).strip("/").split("/")[0]   # p.ej. /jetauto
    looks = robot_root + "/Looks"
    green = _make_material(stage, looks + "/GreenAnodizedAluminum", GREEN_ANODIZED)
    black = _make_material(stage, looks + "/MatteBlackPlastic", MATTE_BLACK)

    def _is_green(path):
        return any(("/" + gl) in path for gl in green_links)

    def _bind(prim, is_green):
        UsdShade.MaterialBindingAPI.Apply(prim).Bind(
            green if is_green else black, UsdShade.Tokens.strongerThanDescendants)

    # 1) des-instanciar las mallas del robot
    to_deinst = [p for p in stage.Traverse()
                 if str(p.GetPath()).startswith(robot_root + "/") and p.IsInstanceable()]
    for p in to_deinst:
        p.SetInstanceable(False)

    # 2) bind directo en cada malla (ya visibles) + en el link como respaldo
    n_mesh = n_link = 0
    for prim in stage.Traverse():
        path = str(prim.GetPath())
        if not path.startswith(robot_root + "/") or "collision" in path.lower():
            continue
        if prim.IsA(UsdGeom.Gprim):
            _bind(prim, _is_green(path))
            n_mesh += 1
        elif prim.GetName().endswith("_link"):
            _bind(prim, prim.GetName() in green_links)
            n_link += 1

    print(f"[materials] de-instanciados={len(to_deinst)}  mallas bindeadas={n_mesh}  "
          f"links={n_link} (root {robot_root})")
    return n_mesh, n_link
