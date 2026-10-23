#!/usr/bin/env python3
"""Reproduce the pinned AWS world in a local ignored cache, never in Git.

Borrow: pycollada/trimesh parse COLLADA, preserve node transforms, normals,
UVs and textures. CoACD decomposes closed nonconvex collision solids. Open
collision surfaces become thin triangular convex prisms, preserving openings.
This is an adapter for this audited SDF subset, not a general SDF importer.
"""
from __future__ import annotations

import argparse
import hashlib
import importlib.metadata
from io import BytesIO
import json
from pathlib import Path
import shutil
import subprocess
import xml.etree.ElementTree as ET

import collada
import coacd
import numpy as np
from scipy.spatial.transform import Rotation
import trimesh
from lxml import etree

ROOT = Path(__file__).resolve().parents[2]
LOCK = ROOT / "models/minisegway/retail_navigation/bookstore_world.lock.json"
DEFAULT_CACHE = ROOT / ".venv/assets/bookstore"


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def pose(element) -> np.ndarray:
    values = np.fromstring(element.findtext("pose", "0 0 0 0 0 0"), sep=" ")
    if values.shape != (6,) or not np.isfinite(values).all():
        raise ValueError("unsupported SDF pose")
    transform = np.eye(4)
    transform[:3, :3] = Rotation.from_euler("xyz", values[3:]).as_matrix()
    transform[:3, 3] = values[:3]
    return transform


def load_meshes(path: Path, scale) -> list[trimesh.Trimesh]:
    """Apply DAE scene graph with trimesh; COLLADA length unit is explicit."""
    # pycollada treats XML comment nodes inside <geometry> as unsupported
    # geometry. Remove comments in memory only; preserve source bytes/hashes.
    xml = etree.tostring(etree.parse(str(path), etree.XMLParser(remove_comments=True)))
    document = collada.Collada(BytesIO(xml))
    if document.assetInfo.upaxis != "Z_UP":
        raise ValueError(f"unaudited up axis: {path}")
    unit = document.assetInfo.unitmeter
    if not unit or not np.isfinite(unit):
        raise ValueError(f"missing COLLADA unit: {path}")
    scene = trimesh.load(BytesIO(xml), file_type="dae", force="scene", process=False,
                         resolver=trimesh.resolvers.FilePathResolver(path.parent))
    pieces = []
    for node in sorted(scene.graph.nodes_geometry):
        transform, key = scene.graph[node]
        mesh = scene.geometry[key].copy()
        mesh.apply_transform(transform)
        mesh.apply_scale(np.asarray(scale) * unit)
        pieces.append(mesh)
    return pieces


def collision_parts(mesh: trimesh.Trimesh):
    """Avoid MuJoCo's whole-object convex hull filling aisles and doorways."""
    mesh = mesh.copy()
    mesh.merge_vertices(merge_tex=True, merge_norm=True)
    mesh.update_faces(mesh.nondegenerate_faces())
    for component in mesh.split(only_watertight=False):
        if component.is_convex and component.is_watertight:
            yield component, "upstream_convex_component"
        elif component.is_watertight:
            # Fixed seed/settings recorded in manifest; no hand-placed boxes.
            parts = coacd.run_coacd(coacd.Mesh(component.vertices, component.faces),
                                    threshold=0.02, preprocess_mode="off", seed=0,
                                    resolution=2000, mcts_iterations=100)
            for vertices, faces in parts:
                yield trimesh.Trimesh(vertices=vertices, faces=faces, process=False), "coacd"
        else:
            # The upstream wall is open/non-manifold. A convex hull would seal
            # the entire shop. 2 mm total surface thickness limits the local
            # normal displacement to 1 mm and leaves its exact openings intact.
            for triangle, normal in zip(component.triangles, component.face_normals):
                vertices = np.vstack((triangle - normal * .001, triangle + normal * .001))
                yield trimesh.convex.convex_hull(vertices), "open_surface_2mm_prism"


def prepare(cache: Path, *, fetch: bool = True) -> Path:
    lock = json.loads(LOCK.read_text())
    upstream, output = cache / "upstream", cache / "derived"
    if not upstream.exists():
        if not fetch:
            raise FileNotFoundError("asset cache missing; run prepare_bookstore.py with network access")
        cache.mkdir(parents=True, exist_ok=True)
        subprocess.run(["git", "clone", "--no-checkout", "--branch", "ros2", "--single-branch", lock["repository"], str(upstream)], check=True)
        subprocess.run(["git", "-C", str(upstream), "checkout", "--detach", lock["commit"]], check=True)
    commit = subprocess.check_output(["git", "-C", str(upstream), "rev-parse", "HEAD"], text=True).strip()
    if commit != lock["commit"]:
        raise ValueError(f"asset commit mismatch: {commit}")
    for relative, expected in lock["source_files_sha256"].items():
        if sha256(upstream / relative) != expected:
            raise ValueError(f"asset hash mismatch: {relative}")
    output.mkdir(parents=True, exist_ok=True)
    shutil.copy2(upstream / "LICENSE", output / "UPSTREAM_LICENSE")
    shutil.copy2(upstream / "package.xml", output / "UPSTREAM_package.xml")
    asset = ET.Element("asset")
    world = ET.Element("worldbody")
    prototypes, prototype_report = {}, []
    missing_textures = []
    generated = []
    world_source = ET.parse(upstream / lock["world_file"]).getroot()
    used_prototypes = {model.findtext("include/uri", "").removeprefix("model://")
                       for model in world_source.findall("world/model")}
    coacd.set_log_level("error")
    for sdf_path in sorted((upstream / "models").glob("*/model.sdf")):
        model = ET.parse(sdf_path).getroot().find("model")
        tag = model.get("name")
        if tag not in used_prototypes:
            continue
        print("Converting", tag, flush=True)
        prototype = ET.Element("body")
        counts = {"visual_faces": 0, "collision_faces": 0, "collision_parts": 0, "textured_parts": 0}
        methods = {}
        visual_bounds, collision_bounds = [], []
        for link in model.findall("link"):
            for kind in ("visual", "collision"):
                for index, element in enumerate(link.findall(kind)):
                    node = element.find("geometry/mesh")
                    if node is None:
                        raise ValueError(f"unsupported geometry in {sdf_path}")
                    uri = node.findtext("uri")
                    if not uri.startswith("file://models/"):
                        raise ValueError(f"unsupported mesh URI {uri}")
                    relative = uri.removeprefix("file://")
                    mesh_path = upstream / lock.get("mesh_uri_rewrites", {}).get(relative, relative)
                    scale = np.fromstring(node.findtext("scale", "1 1 1"), sep=" ")
                    for part_index, mesh in enumerate(load_meshes(mesh_path, scale)):
                        mesh.apply_transform(pose(model) @ pose(link) @ pose(element))
                        name = f"{tag}_{kind}_{index}_{part_index}"
                        counts[f"{kind}_faces"] += len(mesh.faces)
                        (visual_bounds if kind == "visual" else collision_bounds).append(mesh.bounds.tolist())
                        if kind == "visual":
                            file = output / f"{name}.obj"
                            # MuJoCo reads OBJ positions/normals/UVs; material is
                            # provided explicitly, not via ignored MTL files.
                            file.write_text(trimesh.exchange.obj.export_obj(mesh, include_color=False, include_texture=True, return_texture=False, write_texture=False))
                            generated.append(file)
                            ET.SubElement(asset, "mesh", name=name, file=str(file.resolve()))
                            material = getattr(mesh.visual, "material", None)
                            image = getattr(material, "baseColorTexture", None)
                            if image is None:
                                image = getattr(material, "image", None)
                            color = getattr(material, "baseColorFactor", None)
                            rgba = np.ones(4) if color is None else np.asarray(color) / 255.
                            mat = ET.SubElement(asset, "material", name=name, rgba=" ".join(map(str, rgba)), specular="0.1", shininess="0.2")
                            if image is not None:
                                texture = output / f"{name}.png"
                                image.convert("RGB").save(texture)
                                generated.append(texture)
                                ET.SubElement(asset, "texture", name=name, type="2d", file=str(texture.resolve()))
                                mat.set("texture", name)
                                counts["textured_parts"] += 1
                            elif isinstance(mesh.visual, trimesh.visual.texture.TextureVisuals):
                                missing_textures.append(name)
                            ET.SubElement(prototype, "geom", name=name, type="mesh", mesh=name, material=name, group="2", contype="0", conaffinity="0")
                        elif "RetailShopFloor" not in tag:
                            for piece_index, (piece, method) in enumerate(collision_parts(mesh)):
                                cname = f"{name}_convex_{piece_index}"
                                file = output / f"{cname}.stl"
                                piece.export(file)
                                generated.append(file)
                                ET.SubElement(asset, "mesh", name=cname, file=str(file.resolve()))
                                # MuJoCo excludes alpha-zero geoms from ray
                                # queries. Hide group 4 in the viewer instead.
                                ET.SubElement(prototype, "geom", name=cname, type="mesh", mesh=cname, group="4", contype="1", conaffinity="1", rgba="0.5 0.5 0.5 1")
                                counts["collision_parts"] += 1
                                methods[method] = methods.get(method, 0) + 1
        prototypes[tag] = prototype
        prototype_report.append({"model": tag, **counts, "collision_methods": methods, "visual_bounds_m": visual_bounds, "collision_bounds_m": collision_bounds})
    import copy
    instances, excluded = [], []
    for model in world_source.findall("world/model"):
        name = model.get("name")
        if name in lock["excluded_instances"]:
            excluded.append({"name": name, "source_pose": model.findtext("pose"), "reason": lock["excluded_instances"][name]})
            continue
        uri = model.findtext("include/uri")
        if uri is None or not uri.startswith("model://"):
            raise ValueError(f"unsupported world model {name}")
        tag = uri.removeprefix("model://")
        transform = pose(model)
        transform[2, 3] += lock["world_translation_m"][2]
        quaternion = Rotation.from_matrix(transform[:3, :3]).as_quat()
        body = ET.SubElement(world, "body", name="retail_" + name, pos=" ".join(map(str, transform[:3, 3])), quat=" ".join(map(str, [quaternion[3], *quaternion[:3]])))
        for child in prototypes[tag]:
            geom = copy.deepcopy(child)
            geom.set("name", name + "_" + geom.get("name"))
            # Align the source floor's thinner visual surface to its source
            # collision top, now z=0. All other placements remain unchanged.
            if "RetailShopFloor" in tag:
                geom.set("pos", "0 0 " + str(lock["floor_visual_local_z_correction_m"]))
            body.append(geom)
        instances.append({"name": name, "prototype": tag, "world_pose_matrix": transform.tolist()})
    lights = []
    for index, light in enumerate(world_source.findall("world/light")):
        transform = pose(light)
        direction = transform[:3, :3] @ np.fromstring(light.findtext("direction", "0 0 -1"), sep=" ")
        position = transform[:3, 3] + lock["world_translation_m"]
        diffuse = np.fromstring(light.findtext("diffuse", "0.7 0.7 0.7 1"), sep=" ")[:3]
        # MJCF's local spotlight/directional light model; geometry is unchanged.
        ET.SubElement(world, "light", name=f"retail_source_light_{index}",
            pos=" ".join(map(str, position)), dir=" ".join(map(str, direction)),
            diffuse=" ".join(map(str, diffuse)), directional="true" if light.get("type") == "directional" else "false")
        lights.append({"name": light.get("name"), "type": light.get("type"), "pose_matrix": transform.tolist()})
    root = ET.Element("mujoco", model="aws_bookstore_assets")
    root.extend([asset, world])
    ET.indent(root)
    include = output / "bookstore.xml"
    ET.ElementTree(root).write(include, encoding="unicode")
    generated.append(include)
    manifest = {"schema": "companionbot.bookstore.assets.v1", "upstream_commit": commit, "lock_sha256": sha256(LOCK), "source_files_verified": len(lock["source_files_sha256"]), "tools": {p: importlib.metadata.version(p) for p in ("trimesh", "pycollada", "coacd")}, "world_translation_m": lock["world_translation_m"], "floor_visual_local_z_correction_m": lock["floor_visual_local_z_correction_m"], "floor_physics": "existing frozen ground plane at z=0, unchanged friction; source floor collision top aligned to zero", "prototypes": prototype_report, "instances": instances, "excluded": excluded, "missing_textures": missing_textures, "generated_sha256": {p.name: sha256(p) for p in generated}, "include_file": str(include.resolve())}
    manifest["lights"] = lights
    manifest["mesh_uri_rewrites"] = lock.get("mesh_uri_rewrites", {})
    manifest["xml_comment_handling"] = "remove XML comments in memory before pycollada; source unchanged"
    if missing_textures:
        raise ValueError(f"missing texture images: {missing_textures}")
    path = output / "manifest.json"
    path.write_text(json.dumps(manifest, indent=2) + "\n")
    return path


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cache", type=Path, default=DEFAULT_CACHE)
    parser.add_argument("--offline", action="store_true")
    args = parser.parse_args()
    print(prepare(args.cache.resolve(), fetch=not args.offline))
