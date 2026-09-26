"""Render the authored Blender calibration scenes into PNG animation frames.

blender --background x64/guidance/calibration-demos.blend --python tools/render-guidance.py -- --preview
Omit --preview for the complete loops. Use pack-guidance.py after rendering.
"""

import argparse
import json
import math
import shutil
import sys
from pathlib import Path

import bpy
from mathutils import Euler, Matrix, Vector

ROOT = Path(__file__).resolve().parents[1]
FPS = 60
INTRO_FRAMES = 120
FRAMES = 240
SIZE = (192, 168)
SETUPS = {"handheld": "Guide wrist", "headset": "Guide mounted", "contact": "Guide contact"}


def ease(t):
    t = max(0.0, min(1.0, t))
    return t * t * t * (t * (t * 6 - 15) + 10)


def motion(frame):
    # Four eased sweeps with a short hold at each extreme and at the loop seam.
    keys = ((0, 0), (8, 0), (56, 1), (64, 1), (112, 0),
            (128, 0), (176, -1), (184, -1), (232, 0), (240, 0))
    for (start, a), (end, b) in zip(keys, keys[1:]):
        if frame <= end:
            return a + (b - a) * ease((frame - start) / (end - start))
    return 0.0


def render(setup, preview):
    scene = bpy.data.scenes[SETUPS[setup]]
    bpy.context.window.scene = scene
    pivot_name = "Wrist calibration movement" if setup == "handheld" else "Head movement"
    pivot = next(obj for obj in scene.objects if obj.name.startswith(pivot_name))
    rig = next((obj for obj in scene.objects if obj.type == "ARMATURE" and "Head" in obj.pose.bones), None)
    base = pivot.matrix_world.copy()
    center = Vector((-.04, .04, 0)) if setup == "handheld" else base.translation.copy()
    wrist = setup == "handheld"
    size = (320, 192) if wrist else SIZE
    loop_frames = 360 if wrist else FRAMES
    axes = 1 if wrist else 3
    camera_basis = scene.camera.matrix_world.to_quaternion()
    right = camera_basis @ Vector((1, 0, 0))
    up = camera_basis @ Vector((0, 1, 0))
    away = camera_basis @ Vector((0, 0, -1))
    if wrist:
        scene.camera.data.ortho_scale = .62
        # A quiet, camera-facing trace makes the spatial sweep readable.
        path = scene.objects.get("Guide wrist motion path")
        if path is None:
            curve = bpy.data.curves.new("Guide wrist motion path", "CURVE")
            curve.dimensions = "3D"
            curve.bevel_depth = .0007
            curve.bevel_resolution = 2
            spline = curve.splines.new("POLY")
            spline.points.add(255)
            for i, point in enumerate(spline.points):
                phase = i * math.tau / 256
                position = center + away * .12 + right * (.12 * math.sin(phase)) + up * (.045 * math.sin(2 * phase))
                point.co = (*position, 1)
            spline.use_cyclic_u = True
            path = bpy.data.objects.new("Guide wrist motion path", curve)
            scene.collection.objects.link(path)
            material = bpy.data.materials.new("Motion path grey")
            material.use_nodes = True
            nodes = material.node_tree.nodes
            shader = nodes.new("ShaderNodeEmission")
            shader.inputs["Color"].default_value = (.12, .15, .19, 1)
            material.node_tree.links.new(shader.outputs[0], nodes.get("Material Output").inputs["Surface"])
            curve.materials.append(material)
    scene.render.engine = "CYCLES"
    scene.cycles.samples = 48
    scene.cycles.use_denoising = True
    scene.render.use_persistent_data = True
    prefs = bpy.context.preferences.addons["cycles"].preferences
    prefs.compute_device_type = "OPTIX"
    prefs.get_devices()
    for device in prefs.devices:
        device.use = device.type != "CPU"
    scene.cycles.device = "GPU"
    scene.render.resolution_x = size[0] * (2 if preview else 1)
    scene.render.resolution_y = size[1] * (2 if preview else 1)
    scene.render.resolution_percentage = 100
    scene.render.image_settings.file_format = "PNG"
    scene.render.image_settings.color_mode = "RGBA"
    scene.render.film_transparent = True
    folder = ROOT / "x64/guidance" / ("checks" if preview else "frames") / setup
    folder.mkdir(parents=True, exist_ok=True)
    manifest = folder / "complete.json"
    manifest.unlink(missing_ok=True)
    moving_names = {"handheld": ("Touch Pro", "right hand"),
                    "contact": ("Index controller",), "headset": ()}[setup]
    moving = [(scene.objects[name], scene.objects[name].matrix_world.copy()) for name in moving_names]
    approach = Vector((.065, -.035, .025) if setup == "handheld" else (.025, -.065, .015))
    cache = {}

    def save(index, key):
        destination = folder / f"{index:03}.png"
        if key in cache:
            shutil.copyfile(cache[key], destination)
        else:
            scene.render.filepath = str(destination)
            bpy.ops.render.render(write_still=True)
            cache[key] = destination

    for frame in ([0, 48, 119] if preview else range(INTRO_FRAMES)):
        remaining = 1.0 - ease((frame - 9) / 84.0)
        for obj, matrix in moving:
            obj.matrix_world = Matrix.Translation(approach * remaining) @ matrix
        save(frame, ("intro", round(remaining, 8) if moving else 0))
    for obj, matrix in moving:
        obj.matrix_world = matrix
    for axis in range(axes):
        checks = [0, 90, 180, 270] if wrist else [0, 60, 180]
        for frame in (checks if preview else range(loop_frames)):
            if wrist:
                phase = math.tau * ease(frame / loop_frames)
                travel = right * (.12 * math.sin(phase)) + up * (.045 * math.sin(2 * phase)) + away * (.018 * math.sin(phase))
                rotation = Euler((math.radians(22) * math.sin(2 * phase),
                                  math.radians(18) * (math.sin(phase + math.pi / 3) - math.sin(math.pi / 3)),
                                  math.radians(32) * math.sin(phase)), "XYZ").to_matrix().to_4x4()
                key = (axis, frame)
            else:
                angle = motion(frame) * math.radians((35, 23, 24)[axis])
                rotation = Matrix.Rotation(angle, 4, ("Z", "X", "Y")[axis])
                travel = Vector()
                key = (axis, round(angle, 8))
            delta = Matrix.Translation(center + travel) @ rotation @ Matrix.Translation(-center)
            pivot.matrix_world = delta @ base
            if rig:
                bone = rig.pose.bones["Head"]
                bone.matrix = delta @ bone.bone.matrix_local
            save(INTRO_FRAMES + axis * loop_frames + frame, key)
    pivot.matrix_world = base
    if rig:
        rig.pose.bones["Head"].matrix_basis = Matrix.Identity(4)
    if not preview:
        manifest.write_text(json.dumps({"frames": INTRO_FRAMES + loop_frames * axes, "size": size, "fps": FPS}), encoding="utf-8")
    print(f"GUIDANCE_DONE {setup}", flush=True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--preview", action="store_true")
    parser.add_argument("--setup", choices=SETUPS)
    args = parser.parse_args(sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else [])
    for setup in ([args.setup] if args.setup else SETUPS):
        render(setup, args.preview)
