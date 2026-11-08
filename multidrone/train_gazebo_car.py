#!/usr/bin/env python3
"""Train a one-class car detector on the Gazebo vehicle meshes.

Blender renders the hatchback, pickup, and ambulance from above, with a sun
shining straight down, at the drone camera size (320x240, 60 degree field of
view). Ultralytics then fine-tunes a YOLO model whose only class is ``car``.
"""
from __future__ import annotations

import math
import random
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent
MODELS = ROOT / "gazebo_models"
OUT = ROOT / "weights" / "gazebo_car_data"
WEIGHTS = ROOT / "weights" / "gazebo_car.pt"
BLENDER = Path.home() / ".local" / "bin" / "blender"
MESHES = (
    ("hatchback", MODELS / "hatchback/meshes/hatchback.obj", MODELS / "hatchback/materials/textures/hatchback.png"),
    ("ambulance", MODELS / "ambulance/meshes/ambulance.obj", MODELS / "ambulance/materials/textures/ambulance.png"),
    ("pickup", MODELS / "pickup/meshes/pickup.dae", MODELS / "pickup/materials/textures/pickup_diffuse.jpg"),
)
WIDTH = 320
HEIGHT = 240
HFOV = math.radians(60.0)
VIEWS_PER_MESH = 36


def render_dataset() -> None:
    import bpy
    from bpy_extras.object_utils import world_to_camera_view
    from mathutils import Vector

    random.seed(11)
    for split in ("train", "val"):
        (OUT / "images" / split).mkdir(parents=True, exist_ok=True)
        (OUT / "labels" / split).mkdir(parents=True, exist_ok=True)
        for old in list((OUT / "images" / split).glob("*")) + list((OUT / "labels" / split).glob("*")):
            old.unlink()

    bpy.ops.wm.read_factory_settings(use_empty=True)
    scene = bpy.context.scene
    scene.render.engine = "BLENDER_EEVEE_NEXT"
    scene.render.resolution_x = WIDTH
    scene.render.resolution_y = HEIGHT
    scene.render.image_settings.file_format = "JPEG"
    scene.render.image_settings.quality = 92

    world = bpy.data.worlds.new("sky")
    scene.world = world
    world.use_nodes = True
    bg = world.node_tree.nodes["Background"]
    bg.inputs[0].default_value = (0.62, 0.62, 0.60, 1.0)
    bg.inputs[1].default_value = 0.35

    bpy.ops.object.light_add(type="SUN", location=(0.0, 0.0, 30.0))
    sun = bpy.context.object
    sun.data.energy = 6.0
    sun.rotation_euler = (0.0, 0.0, 0.0)

    bpy.ops.mesh.primitive_plane_add(size=80.0, location=(0.0, 0.0, 0.0))
    ground = bpy.context.object
    gmat = bpy.data.materials.new("ground")
    gmat.use_nodes = True
    gmat.node_tree.nodes["Principled BSDF"].inputs["Base Color"].default_value = (0.42, 0.42, 0.40, 1.0)
    ground.data.materials.append(gmat)

    bpy.ops.object.camera_add(location=(0.0, 0.0, 8.0))
    cam = bpy.context.object
    cam.data.sensor_fit = "HORIZONTAL"
    cam.data.angle = HFOV
    scene.camera = cam

    def clear_cars() -> None:
        for obj in list(bpy.data.objects):
            if obj.get("gazebo_car"):
                bpy.data.objects.remove(obj, do_unlink=True)

    def import_mesh(path: Path, texture: Path) -> list:
        before = set(bpy.data.objects)
        if path.suffix.lower() == ".dae":
            bpy.ops.wm.collada_import(filepath=str(path))
        else:
            bpy.ops.wm.obj_import(filepath=str(path))
        new = [obj for obj in bpy.data.objects if obj not in before and obj.type == "MESH"]
        for obj in new:
            obj["gazebo_car"] = True
        dims = []
        for obj in new:
            for corner in obj.bound_box:
                dims.append((obj.matrix_world @ Vector(corner)).length)
        if dims and max(dims) > 20.0:
            for obj in new:
                obj.scale = (0.0254, 0.0254, 0.0254)
            bpy.ops.object.select_all(action="DESELECT")
            for obj in new:
                obj.select_set(True)
            bpy.context.view_layer.objects.active = new[0]
            bpy.ops.object.transform_apply(scale=True)
        if texture.is_file():
            image = bpy.data.images.load(str(texture), check_existing=True)
            for obj in new:
                for mat in obj.data.materials:
                    if mat is None:
                        continue
                    mat.use_nodes = True
                    nodes = mat.node_tree.nodes
                    links = mat.node_tree.links
                    bsdf = next((n for n in nodes if n.type == "BSDF_PRINCIPLED"), None)
                    if bsdf is None:
                        continue
                    bsdf.inputs["Roughness"].default_value = 0.55
                    if "Metallic" in bsdf.inputs:
                        bsdf.inputs["Metallic"].default_value = 0.0
                    tex = nodes.new("ShaderNodeTexImage")
                    tex.image = image
                    links.new(tex.outputs["Color"], bsdf.inputs["Base Color"])
        zs = []
        for obj in new:
            for corner in obj.bound_box:
                zs.append((obj.matrix_world @ Vector(corner)).z)
        if zs:
            drop = min(zs)
            for obj in new:
                obj.location.z -= drop
        bpy.ops.object.empty_add(type="PLAIN_AXES", location=(0.0, 0.0, 0.0))
        root = bpy.context.object
        root["gazebo_car"] = True
        for obj in new:
            obj.parent = root
            obj.matrix_parent_inverse = root.matrix_world.inverted()
        return root, new

    def bbox_yolo(cars: list) -> str | None:
        xs, ys = [], []
        for obj in cars:
            for vert in obj.data.vertices:
                world = obj.matrix_world @ vert.co
                co = world_to_camera_view(scene, cam, world)
                if co.z <= 0.05:
                    continue
                u, v = co.x, 1.0 - co.y
                if -0.15 <= u <= 1.15 and -0.15 <= v <= 1.15:
                    xs.append(u)
                    ys.append(v)
        if len(xs) < 12:
            return None
        x1, x2 = max(0.0, min(xs)), min(1.0, max(xs))
        y1, y2 = max(0.0, min(ys)), min(1.0, max(ys))
        width, height = x2 - x1, y2 - y1
        if width < 0.04 or height < 0.04 or width * height > 0.55:
            return None
        cx = (x1 + x2) / 2.0
        cy = (y1 + y2) / 2.0
        return f"0 {cx:.6f} {cy:.6f} {width:.6f} {height:.6f}\n"

    index = 0
    for _name, mesh, texture in MESHES:
        clear_cars()
        loaded = import_mesh(mesh, texture)
        if not loaded:
            print(f"train_gazebo_car: no mesh from {mesh}", flush=True)
            continue
        root, cars = loaded
        for _view in range(VIEWS_PER_MESH):
            alt = random.uniform(4.0, 18.0)
            reach = alt * math.tan(HFOV / 2.0) * 0.45
            root.rotation_euler = (0.0, 0.0, random.uniform(-math.pi, math.pi))
            root.location.x = random.uniform(-reach, reach)
            root.location.y = random.uniform(-reach, reach)
            bpy.context.view_layer.update()
            cam.location = (0.0, 0.0, alt)
            cam.rotation_euler = (0.0, 0.0, 0.0)
            label = bbox_yolo(cars)
            if label is None:
                continue
            split = "val" if index % 5 == 0 else "train"
            stem = f"car_{index:04d}"
            scene.render.filepath = str(OUT / "images" / split / stem)
            bpy.ops.render.render(write_still=True)
            (OUT / "labels" / split / f"{stem}.txt").write_text(label, encoding="utf-8")
            index += 1
        clear_cars()

    for n in range(12):
        cam.location = (0.0, 0.0, random.uniform(5.0, 16.0))
        cam.rotation_euler = (0.0, 0.0, 0.0)
        stem = f"empty_{n:04d}"
        scene.render.filepath = str(OUT / "images" / "train" / stem)
        bpy.ops.render.render(write_still=True)
    print(f"train_gazebo_car: rendered {index} car views", flush=True)


def train() -> None:
    from ultralytics import YOLO

    yaml = OUT / "data.yaml"
    yaml.write_text(
        "path: "
        + str(OUT)
        + "\ntrain: images/train\nval: images/val\nnames:\n  0: car\n",
        encoding="utf-8",
    )
    model = YOLO("yolo11s.pt")
    model.train(
        data=str(yaml),
        epochs=40,
        imgsz=WIDTH,
        batch=16,
        device=0,
        workers=2,
        project=str(ROOT / "weights"),
        name="gazebo_car_run",
        exist_ok=True,
        pretrained=True,
        mosaic=0.0,
        patience=12,
    )
    best = ROOT / "weights" / "gazebo_car_run" / "weights" / "best.pt"
    WEIGHTS.parent.mkdir(parents=True, exist_ok=True)
    WEIGHTS.write_bytes(best.read_bytes())
    print(f"train_gazebo_car: wrote {WEIGHTS}", flush=True)


def main() -> None:
    if "bpy" in sys.modules or Path(sys.executable).name.startswith("blender"):
        render_dataset()
        return
    if not BLENDER.is_file():
        raise SystemExit(f"train_gazebo_car: blender not found at {BLENDER}")
    subprocess.run([str(BLENDER), "--background", "--python", str(Path(__file__).resolve())], check=True)
    train()


if __name__ == "__main__":
    main()
